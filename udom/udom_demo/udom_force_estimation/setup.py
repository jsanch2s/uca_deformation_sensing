#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_force_estimation'],
    package_dir={
        'udom_force_estimation': 'src/udom_force_estimation',
    }
)

setup(**d)
