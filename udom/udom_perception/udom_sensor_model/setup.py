#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_sensor_model', 'sensor_model'],
    package_dir={
        'udom_sensor_model': 'src/udom_sensor_model',
        'sensor_model': 'sensor_model',
    }
)

setup(**d)
