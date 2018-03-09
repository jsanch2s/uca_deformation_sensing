#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_deformation_sensing'],
    package_dir={
        'udom_deformation_sensing': 'src/udom_deformation_sensing',
    }
)

setup(**d)
