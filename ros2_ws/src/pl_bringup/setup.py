import os
from glob import glob
from setuptools import setup

package_name = "pl_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="madakie",
    maintainer_email="madakie@todo.todo",
    description="Bringup launch files for the precision landing stack",
    license="MIT",
    entry_points={"console_scripts": []},
)
