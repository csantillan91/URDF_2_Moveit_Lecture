from setuptools import find_packages, setup

package_name = "buzz_arm_hw"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Chris Santillan",
    maintainer_email="csb1@iastate.edu",
    description="Homework tools: pick-and-place sequencer, simulation preflight, block reset, Cartesian path check.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pick_place = buzz_arm_hw.pick_place:main",
            "check_sim = buzz_arm_hw.check_sim:main",
            "reset_block = buzz_arm_hw.reset_block:main",
            "cartesian_check = buzz_arm_hw.cartesian_check:main",
            "ik_check = buzz_arm_hw.ik_check:main",
        ],
    },
)
