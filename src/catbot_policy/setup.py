"""Install the exact policy source pinned by external/Catbot, without hardware."""

from setuptools import setup

setup(
    name="catbot_policy",
    version="0.1.0",
    packages=["catbot_policy"],
    package_data={
        "catbot_policy": [
            "default_mapping.json",
            "inference/*.toml",
            "inference/*.lock",
            "inference/*.py",
        ]
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/catbot_policy"]),
        ("share/catbot_policy", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Catbot",
    maintainer_email="hiroto125takeuchi@gmail.com",
    description="Portable policy from the pinned Catbot source",
    license="MIT",
    entry_points={"console_scripts": ["policy = catbot_policy.node:main"]},
)
