from setuptools import find_packages, setup

package_name = "rl_action"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="you@example.com",
    description="ROS 2 action client shell",
    license="MIT",
    entry_points={
        "console_scripts": [
            "command = rl_action.command:main",
        ],
    },
)

