from glob import glob
from setuptools import setup

package_name = "quadruped_navigation_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config/maps", glob("config/maps/*")),
        (f"share/{package_name}/config/pcd", glob("config/pcd/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="navigation owner",
    maintainer_email="user@example.com",
    description="Navigation-only bringup for quadruped mapping, localization, Nav2, obstacle avoidance, and motion service.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wait_for_nav_ready = quadruped_navigation_bringup.wait_for_nav_ready:main",
            "run_when_ready = quadruped_navigation_bringup.run_when_ready:main",
        ]
    },
)
