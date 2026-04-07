import math

from rich.live import Live
from rich.table import Table
from rich.columns import Columns
from rich.console import Group
import time

from mission_status.drone_record import DroneRecord, DroneStateStore, MissionRecord, MissionStore


def _flag(val: bool | None) -> str:
    if val is None:
        return "[dim]—[/]"
    return "[green]ON[/]" if val else "[dim]off[/]"


def build_mission_table(rec: MissionRecord) -> Table:
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

    return Columns([origin, ctrl], padding=(0, 4))


def build_fleet_table(records: list[DroneRecord]) -> Table:
    t = Table(title="Fleet Status")
    t.add_column("Drone", style="cyan")
    t.add_column("State")
    t.add_column("Breach")
    t.add_column("Position (local)")
    t.add_column("Heading")
    t.add_column("Alt")
    t.add_column("GPS (lat,lon)")
    t.add_column("GPS Sats")
    t.add_column("NED Vel (x,y,z)")
    t.add_column("Last Seen")
    t.add_column("Link")

    for r in sorted(records, key=lambda x: x.namespace):
        age = time.monotonic() - r.last_seen
        link = "[red]STALE[/]" if r.stale else "[green]OK[/]"
        breach = "[red]YES[/]" if r.breach else ("—" if r.breach is None else "no")

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

        t.add_row(
            r.namespace,
            r.state or "—",
            breach,
            local_pos,
            heading,
            alt,
            gps,
            gps_sats,
            ned_vel,
            f"{age:.1f}s ago",
            link,
        )

    t.caption = "[dim]State/breach/GPS/vel columns update on RobotStatus message only (state-change driven)[/]"
    return t


def ui_main(store: DroneStateStore, mission_store: MissionStore):
    with Live(refresh_per_second=4) as live:
        while True:
            panel = Group(
                build_mission_table(mission_store.snapshot()),
                build_fleet_table(store.snapshot()),
            )
            live.update(panel)
            time.sleep(0.25)
