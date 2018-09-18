#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_pose_control'],
    package_dir={
        'udom_pose_control': 'src/udom_pose_control',
    }
)

setup(**d)
