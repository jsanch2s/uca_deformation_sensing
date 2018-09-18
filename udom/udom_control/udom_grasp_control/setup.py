#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_grasp_control'],
    package_dir={
        'udom_grasp_control': 'src/udom_grasp_control',
    }
)

setup(**d)
