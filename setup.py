from setuptools import find_packages, setup

package_name = 'mission_status'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fred',
    maintainer_email='fred.vatnsdal@gmail.com',
    description='Live fleet status dashboard for PAC GCS',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_status = mission_status.main:main',
        ],
    },
)
