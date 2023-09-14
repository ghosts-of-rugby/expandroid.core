from setuptools import find_packages, setup

package_name = "expandroid_commander"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hisaki",
    maintainer_email="yhisaki31@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "expandroid_commander = expandroid_commander.expandroid_commander:main",
            "expandroid_parameter_setup = expandroid_commander.expandroid_parameter_setup:main",
            "joystick_and_app = expandroid_commander.joystick_and_app:main",
        ],
    },
)
