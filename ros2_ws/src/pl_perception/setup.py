from setuptools import setup

package_name = "pl_perception"

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
    description="Perception for precision landing",
    license="MIT",
    entry_points={
        "console_scripts": [
            "aruco_detector = pl_perception.aruco_detector:main",
            "yolo_detector = pl_perception.yolo_detector:main",
            "tag_estimator = pl_perception.tag_estimator:main",
        ],
    },
)
