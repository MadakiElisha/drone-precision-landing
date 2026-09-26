from setuptools import setup

package_name = "pl_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="madakie",
    maintainer_email="madakie@todo.todo",
    description="Closed-loop precision landing control",
    license="MIT",
    entry_points={
        "console_scripts": [
            "landing_controller = pl_control.landing_controller:main",
        ],
    },
)
