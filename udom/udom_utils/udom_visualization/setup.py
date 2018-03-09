#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_visualization'],
    package_dir={
        'udom_visualization': 'src/udom_visualization',
    }
)

setup(**d)
