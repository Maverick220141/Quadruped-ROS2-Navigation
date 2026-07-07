from glob import glob
from setuptools import setup

package_name = "lidar_scan_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="navigation owner",
    maintainer_email="navigation@example.com",
    description="Bridge FAST-LIO body-frame point clouds to Nav2 LaserScan input.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fix_scan_stamp = lidar_scan_bridge.fix_scan_stamp:main",
            "nav_diag_logger = lidar_scan_bridge.nav_diag_logger:main",
        ],
    },
)
