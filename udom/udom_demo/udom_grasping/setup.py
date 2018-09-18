#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_grasping'],
    package_dir={
        'udom_grasping': 'src/udom_grasping',
    }
)

setup(**d)
