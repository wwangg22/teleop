from setuptools import setup
import os
from glob import glob

package_name = "demo_station"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "web"),
         glob("demo_station/web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Teleop demo collection station: recorder + web GUI",
    license="MIT",
    entry_points={
        "console_scripts": [
            "demo_recorder = demo_station.recorder:main",
            "demo_gui = demo_station.gui:main",
        ],
    },
)
