#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_contact_model'],
    package_dir={
        'udom_contact_model': 'src/udom_contact_model',
    }
)

setup(**d)
