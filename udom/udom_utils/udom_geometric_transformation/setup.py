#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_geometric_transformation'],
    package_dir={
        'udom_geometric_transformation': 'src/udom_geometric_transformation',
    }
)

setup(**d)
