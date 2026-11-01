from setuptools import setup

package_name = "x650_offboard"

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
    maintainer="Polarbear6260",
    maintainer_email="Polarbear6260@users.noreply.github.com",
    description="Offboard control for the X650 PX4 SITL",
    license="MIT",
    entry_points={
        "console_scripts": [
            "offboard_hover = x650_offboard.offboard_hover:main",
            "landing_demo = x650_offboard.landing_demo:main",
            "moving_landing = x650_offboard.moving_landing:main",
        ],
    },
)
