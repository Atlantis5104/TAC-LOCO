# Copyright (c) 2026, Tactile Array Sensor contributors.
# SPDX-License-Identifier: BSD-3-Clause

"""Packaging for the standalone Isaac Lab tactile-array extension."""

from setuptools import find_packages, setup

setup(
    name="isaaclab-tactile-array",
    version="0.1.0",
    description="A standalone tactile-array sensor extension for Isaac Lab 2.3.2.",
    author="Tactile Array Sensor contributors",
    license="BSD-3-Clause",
    license_files=("LICENSE",),
    python_requires=">=3.10",
    packages=find_packages(),
    include_package_data=True,
    package_data={"isaaclab_tactile_array": ["assets/*", "assets/**/*"]},
    install_requires=["rtree", "trimesh"],
    extras_require={"demo": []},
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    zip_safe=False,
)
