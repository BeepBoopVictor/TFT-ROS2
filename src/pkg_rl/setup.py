from setuptools import setup, find_packages
from glob import glob
import os

package_name = "pkg_rl"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Victor",
    maintainer_email="victor@example.com",
    description=(
        "Reinforcement learning package for Panda pick and place using "
        "ROS 2 Humble, Gymnasium, Tianshou, SAC, HER and curriculum learning."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "train_sac = pkg_rl.training.train_sac:main",
            "train_sac_her = pkg_rl.training.train_sac_her:main",
            "eval_policy = pkg_rl.inference.run_policy:main",
        ],
    },
)