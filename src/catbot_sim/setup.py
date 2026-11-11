from setuptools import setup

setup(
    name="catbot_sim",
    version="0.1.0",
    packages=["catbot_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/catbot_sim"]),
        ("share/catbot_sim", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    license="MIT",
    maintainer="Catbot",
    maintainer_email="hiroto125takeuchi@gmail.com",
    description="ROS adapter for an external Catbot MuJoCo worker",
    entry_points={
        "console_scripts": [
            "bridge = catbot_sim.bridge:main",
            "remote = catbot_sim.remote:main",
        ]
    },
)
