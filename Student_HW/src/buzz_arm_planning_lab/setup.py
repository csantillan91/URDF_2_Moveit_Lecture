from setuptools import find_packages, setup

package_name = "buzz_arm_planning_lab"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/scenes.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Chris Santillan",
    maintainer_email="csb1@iastate.edu",
    description="Part 2 lab tools: benchmark OMPL planners through move_group and load obstacle scenes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "plan_benchmark = buzz_arm_planning_lab.plan_benchmark:main",
            "scene_objects = buzz_arm_planning_lab.scene_objects:main",
            "plot_results = buzz_arm_planning_lab.plot_results:main",
        ],
    },
)
