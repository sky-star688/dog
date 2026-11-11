from glob import glob
from setuptools import setup

package_name = "fleet_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fleet Team",
    maintainer_email="user@example.com",
    description="Fleet-level coordinator for dual-Lite3 collaborative missions.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_manager = fleet_manager.fleet_manager:main",
        ],
    },
)
