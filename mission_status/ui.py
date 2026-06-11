import math
import os
import select
import sys
import termios
import threading
import tty

from rich.console import Console, Group
from rich.live import Live
from rich.table import Table
from rich.columns import Columns
from rich.panel import Panel
from rich.text import Text
import time

from mission_status.drone_record import DroneRecord, DroneStateStore, MissionRecord, MissionStore

# ── Map rendering constants ───────────────────────────────────────────────────

MAP_W = 50   # character columns
MAP_H = 24   # character rows

_DENSITY_CHARS = ' ░▒▓█'

# Per-robot colors (cycled for > 7 robots)
_ROBOT_COLORS = [
    'bright_green', 'bright_yellow', 'bright_cyan',
    'bright_magenta', 'bright_red', 'bright_blue', 'bright_white',
]

# ── Scroll state ──────────────────────────────────────────────────────────────

_scroll_offset: int = 0
_scroll_lock = threading.Lock()
_scroll_total: list[int] = [0]    # total drone rows (updated each render)
_scroll_vis: list[int] = [10]     # visible rows available (updated each render)


def _start_keyboard_thread() -> threading.Thread:
    """Background thread: read arrow/page keys and update _scroll_offset."""
    def _reader() -> None:
        global _scroll_offset
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                if not select.select([sys.stdin], [], [], 0.1)[0]:
                    continue
                ch = os.read(fd, 1)
                if ch != b'\x1b':
                    continue
                if not select.select([sys.stdin], [], [], 0.05)[0]:
                    continue
                if os.read(fd, 1) != b'[':
                    continue
                if not select.select([sys.stdin], [], [], 0.05)[0]:
                    continue
                ch3 = os.read(fd, 1)
                with _scroll_lock:
                    max_off = max(0, _scroll_total[0] - _scroll_vis[0])
                    if ch3 == b'A':       # up arrow
                        _scroll_offset = max(0, _scroll_offset - 1)
                    elif ch3 == b'B':     # down arrow
                        _scroll_offset = min(max_off, _scroll_offset + 1)
                    elif ch3 in (b'5', b'6'):   # page up / page down
                        if select.select([sys.stdin], [], [], 0.05)[0]:
                            os.read(fd, 1)  # consume trailing '~'
                        step = _scroll_vis[0]
                        if ch3 == b'5':
                            _scroll_offset = max(0, _scroll_offset - step)
                        else:
                            _scroll_offset = min(max_off, _scroll_offset + step)
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass

    t = threading.Thread(target=_reader, daemon=True)
    t.start()
    return t


def build_map_panel(records: list[DroneRecord], rec: MissionRecord) -> Panel:
    world_size = rec.world_size or 512.0
    env_scale = rec.env_scale_factor or 1.0
    phys_extent = world_size / env_scale   # physical metres across the arena

    buf_l = rec.fence_x_buf_l
    buf_r = rec.fence_x_buf_r
    buf_b = rec.fence_y_buf_b
    buf_t = rec.fence_y_buf_t

    # View covers the full geofence extents: [-buf_l, phys_extent+buf_r] x [-buf_b, phys_extent+buf_t]
    view_x_min = -buf_l
    view_w = buf_l + phys_extent + buf_r
    view_y_min = -buf_b
    view_h = buf_b + phys_extent + buf_t

    density_map = rec.density_map
    map_size = rec.density_map_size or 0
    max_density = max(density_map) if density_map else 0.0
    if max_density == 0.0:
        max_density = 1.0

    # Coordinate helpers (physical metres → grid cell)
    def to_col(x: float) -> int:
        return max(0, min(MAP_W - 1, int((x - view_x_min) / view_w * MAP_W)))

    def to_row(y: float) -> int:
        return max(0, min(MAP_H - 1, MAP_H - 1 - int((y - view_y_min) / view_h * MAP_H)))

    # Arena boundary columns/rows in grid space
    a_col_l = to_col(0.0)
    a_col_r = to_col(phys_extent)
    a_row_t = to_row(phys_extent)   # high y → low row (top of display)
    a_row_b = to_row(0.0)           # low y  → high row (bottom of display)

    # ── Build 2D grid of (char, style) ───────────────────────────────────────
    grid: list[list[tuple[str, str]]] = [[(' ', '') for _ in range(MAP_W)] for _ in range(MAP_H)]

    # 1. Density fill inside the arena
    if density_map and map_size > 0:
        for grow in range(MAP_H):
            for gcol in range(MAP_W):
                # Centre of this cell in physical metres
                cx = view_x_min + (gcol + 0.5) / MAP_W * view_w
                cy = view_y_min + (MAP_H - grow - 0.5) / MAP_H * view_h
                if 0.0 <= cx <= phys_extent and 0.0 <= cy <= phys_extent:
                    # Sample density map (row-major, row 0 = y=phys_extent side)
                    mc = max(0, min(map_size - 1, int(cx / phys_extent * map_size)))
                    mr = max(0, min(map_size - 1, int((1.0 - cy / phys_extent) * map_size)))
                    v = density_map[mr * map_size + mc] / max_density
                    char = _DENSITY_CHARS[min(len(_DENSITY_CHARS) - 1, int(v * len(_DENSITY_CHARS)))]
                    r_val = int(v * 180)
                    b_val = int((1.0 - v) * 180)
                    grid[grow][gcol] = (char, f'rgb({r_val},0,{b_val})')

    # 2. Arena boundary — single-line box
    _ARENA_STYLE = 'dim white'
    for gcol in range(a_col_l, a_col_r + 1):
        _h = '─'
        grid[a_row_t][gcol] = (_h, _ARENA_STYLE)
        grid[a_row_b][gcol] = (_h, _ARENA_STYLE)
    for grow in range(a_row_t, a_row_b + 1):
        _v = '│'
        grid[grow][a_col_l] = (_v, _ARENA_STYLE)
        grid[grow][a_col_r] = (_v, _ARENA_STYLE)
    grid[a_row_t][a_col_l] = ('┌', _ARENA_STYLE)
    grid[a_row_t][a_col_r] = ('┐', _ARENA_STYLE)
    grid[a_row_b][a_col_l] = ('└', _ARENA_STYLE)
    grid[a_row_b][a_col_r] = ('┘', _ARENA_STYLE)

    # 3. Geofence boundary — double-line box at view edges
    _FENCE_STYLE = 'yellow'
    for gcol in range(MAP_W):
        grid[0][gcol]        = ('═', _FENCE_STYLE)
        grid[MAP_H - 1][gcol] = ('═', _FENCE_STYLE)
    for grow in range(MAP_H):
        grid[grow][0]         = ('║', _FENCE_STYLE)
        grid[grow][MAP_W - 1] = ('║', _FENCE_STYLE)
    grid[0][0]               = ('╔', _FENCE_STYLE)
    grid[0][MAP_W - 1]       = ('╗', _FENCE_STYLE)
    grid[MAP_H - 1][0]       = ('╚', _FENCE_STYLE)
    grid[MAP_H - 1][MAP_W - 1] = ('╝', _FENCE_STYLE)

    # 4. Robot markers — highest priority
    sorted_ns = sorted(r.namespace for r in records)
    ns_color = {ns: _ROBOT_COLORS[i % len(_ROBOT_COLORS)] for i, ns in enumerate(sorted_ns)}

    robot_cells: dict[tuple[int, int], list[tuple[str, str]]] = {}
    for r in records:
        if r.position is None:
            continue
        key = (to_col(r.position[0]), to_row(r.position[1]))
        robot_cells.setdefault(key, []).append((r.namespace, ns_color[r.namespace]))

    for (gcol, grow), robots_here in robot_cells.items():
        if len(robots_here) == 1:
            _, color = robots_here[0]
            grid[grow][gcol] = ('◉', f'bold {color}')
        else:
            grid[grow][gcol] = (str(len(robots_here)), 'bold bright_white')

    # ── Render grid to rich Text ──────────────────────────────────────────────
    body = Text()
    for grow in range(MAP_H):
        for gcol in range(MAP_W):
            char, style = grid[grow][gcol]
            body.append(char, style=style if style else None)
        body.append('\n')

    # Legend
    if sorted_ns:
        body.append('\n')
        for ns in sorted_ns:
            body.append('◉ ', style=f'bold {ns_color[ns]}')
            body.append(f'{ns}  ')

    cell_m = view_w / MAP_W
    density_status = f'{map_size}×{map_size} density' if density_map else 'no density (waiting)'
    fence_dims = f'fence ±({buf_l:.0f}m, {buf_r:.0f}m, {buf_b:.0f}m, {buf_t:.0f}m)'
    body.append(f'\n[dim]{phys_extent:.0f}m arena  |  cell ≈ {cell_m:.1f}m  |  {fence_dims}  |  {density_status}[/dim]')

    title = 'Mission Map'
    if not density_map:
        title += ' [dim](waiting for /sim services)[/dim]'
    return Panel(body, title=title, border_style='blue', expand=False)


def _flag(val: bool | None) -> str:
    if val is None:
        return "[dim]—[/]"
    return "[green]ON[/]" if val else "[dim]off[/]"


def build_mission_table(rec: MissionRecord):
    origin = Table(title="Mission Origin", show_header=True, expand=False)
    origin.add_column("Field", style="cyan")
    origin.add_column("Value")
    origin.add_row("Lat", f"{rec.origin_lat:.6f}" if rec.origin_lat is not None else "[dim]—[/]")
    origin.add_row("Lon", f"{rec.origin_lon:.6f}" if rec.origin_lon is not None else "[dim]—[/]")
    origin.add_row("Heading", f"{math.degrees(rec.origin_heading):.1f}° ({rec.origin_heading:.3f}r)" if rec.origin_heading is not None else "[dim]—[/]")

    ctrl = Table(title="Mission Control", show_header=True, expand=False)
    ctrl.add_column("Flag", style="cyan")
    ctrl.add_column("Value")
    ctrl.add_row("hw_enable", _flag(rec.hw_enable))
    ctrl.add_row("ob_enable", _flag(rec.ob_enable))
    ctrl.add_row("ob_takeoff", _flag(rec.ob_takeoff))
    ctrl.add_row("ob_land", _flag(rec.ob_land))
    ctrl.add_row("geofence", _flag(rec.geofence))
    ctrl.add_row("pac_offboard_only", _flag(rec.pac_offboard_only))
    ctrl.add_row("pac_lpac_l1", _flag(rec.pac_lpac_l1))
    ctrl.add_row("pac_lpac_l2", _flag(rec.pac_lpac_l2))

    return Group(origin, ctrl)


def build_fleet_table(
    records: list[DroneRecord],
    offset: int = 0,
    max_rows: int | None = None,
) -> Table:
    t = Table(title="Fleet Status")
    t.add_column("Drone", style="cyan")
    t.add_column("State")
    t.add_column("Diagnostic")
    t.add_column("Breach")
    t.add_column("Preflight Pass")
    t.add_column("Arming State")
    t.add_column("Disarming Reason")
    t.add_column("Nav State")
    t.add_column("Failure Det.")
    t.add_column("Safety Off")
    t.add_column("Position (mission)")
    t.add_column("Heading")
    t.add_column("GPS Sats")
    t.add_column("NED Vel")
    t.add_column("Battery")
    t.add_column("Last Seen")
    t.add_column("Link")

    sorted_records = sorted(records, key=lambda x: x.namespace)
    total = len(sorted_records)
    visible = sorted_records[offset:offset + max_rows] if max_rows is not None else sorted_records

    for r in visible:
        age = time.monotonic() - r.last_seen
        link = "[red]STALE[/]" if r.stale else "[green]OK[/]"
        breach = "[red]YES[/]" if r.breach else ("--" if r.breach is None else "no")

        preflight = "[green]OK[/]" if r.preflight_pass else "[red]FAIL[/]"
        arming_state = f"{r.arming_state}"
        disarming_reason = f"{r.disarming_reason}"
        nav_state = f"{r.nav_state}"
        failure_detector_status = f"{r.failure_detector_status}"
        safety_off = "[green]OFF[/]" if r.safety_off else "[red]ON[/]"

        mission_pos = (
            "(" + ",".join(f"{v:.2f}" for v in r.position) + ")"
            if r.position is not None else "—"
        )
        local_pos = (
            f"{r.local_pos_x:.2f},{r.local_pos_y:.2f}"
            if r.local_pos_x is not None else "—"
        )
        heading = f"{math.degrees(r.local_pos_heading):.1f}° ({r.local_pos_heading:.3f}r)" if r.local_pos_heading is not None else "—"
        alt = f"{r.local_pos_z:.2f}m" if r.local_pos_z is not None else "—"
        gps = (
            f"{r.gps_lat:.5f},{r.gps_lon:.5f}"
            if r.gps_lat is not None else "—"
        )
        gps_sats = str(r.gps_sats) if r.gps_sats is not None else "—"
        ned_vel = (
            f"{r.ned_vel_x:.2f},{r.ned_vel_y:.2f},{r.ned_vel_z:.2f}"
            if r.ned_vel_x is not None else "—"
        )

        robot_name = f"[green]{r.namespace}[/]" if r.gps_sats is not None and int(r.gps_sats) > 10 else f"[red]{r.namespace}[/]"

        if r.batt_pct is not None:
            _batt_colors = {
                "OK": "green", "Low": "yellow", "Critical": "red", "EMPTY": "bold red",
            }
            color = _batt_colors.get(r.batt_status or "", "dim")
            batt = f"[{color}]{r.batt_pct}%[/]"
        elif r.batt_status is not None:
            batt = f"[dim]{r.batt_status}[/]"
        else:
            batt = "[dim]—[/]"

        t.add_row(
            robot_name,
            r.state or "--",
            r.diagnostic or "--",
            breach,
            preflight,
            arming_state,
            disarming_reason,
            nav_state,
            failure_detector_status,
            safety_off,
            mission_pos,
            heading,
            gps,
            gps_sats,
            ned_vel,
            batt,
            f"{age:.1f}s ago",
            link,
        )

    if max_rows is not None and total > max_rows:
        end = min(offset + max_rows, total)
        scroll_hint = f"rows {offset + 1}–{end} of {total}  ↑↓ scroll  PgUp/PgDn page  |  "
    else:
        scroll_hint = ""
    t.caption = f"[dim]{scroll_hint}State/breach/GPS/vel columns update on RobotStatus message only[/]"
    return t


# Rows consumed by the top section (map panel + mission tables + fleet table chrome)
_TOP_ROWS_WITH_MAP = MAP_H + 8   # map border/legend/caption + padding
_TOP_ROWS_NO_MAP = 14            # mission tables
_TABLE_CHROME = 5                # fleet table title + header + separator + caption


def ui_main(store: DroneStateStore, mission_store: MissionStore, show_map: bool = True):
    global _scroll_offset
    console = Console()
    _start_keyboard_thread()

    with Live(console=console, refresh_per_second=4) as live:
        while True:
            mission_rec = mission_store.snapshot()
            records = store.snapshot()

            top_rows = _TOP_ROWS_WITH_MAP if show_map else _TOP_ROWS_NO_MAP
            max_fleet_rows = max(1, console.height - top_rows - _TABLE_CHROME)

            with _scroll_lock:
                total = len(records)
                _scroll_total[0] = total
                _scroll_vis[0] = max_fleet_rows
                _scroll_offset = max(0, min(_scroll_offset, max(0, total - max_fleet_rows)))
                offset = _scroll_offset

            top = (
                Columns(
                    [build_mission_table(mission_rec), build_map_panel(records, mission_rec)],
                    padding=(0, 2),
                )
                if show_map else build_mission_table(mission_rec)
            )
            live.update(Group(top, build_fleet_table(records, offset=offset, max_rows=max_fleet_rows)))
            time.sleep(0.25)
