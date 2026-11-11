from glob import glob
from setuptools import setup

package_name = "lite3_02_picking"

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
    maintainer="Dog 2 Team",
    maintainer_email="user@example.com",
    description="OpenMV precision picking + MyArm-2 arm dog.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "picking_manager = lite3_02_picking.picking_manager:main",
            "openmv_positioner = lite3_02_picking.openmv_positioner:main",
            "map_subscriber = lite3_02_picking.map_subscriber:main",
            "dog2_localization = lite3_02_picking.dog2_localization:main",
            "dog2_arm_bridge = lite3_02_picking.arm_bridge:main",
            "vlm_detector = lite3_02_picking.vlm_detector:main",
        ],
    },
)
