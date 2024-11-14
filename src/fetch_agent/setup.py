from setuptools import find_packages, setup

package_name = "fetch_agent"

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
    maintainer="tobinio",
    maintainer_email="tobinio@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["agent = fetch_agent.agent:main", "ros_agent = fetch_agent.rosAgent:main"],
    },
)
