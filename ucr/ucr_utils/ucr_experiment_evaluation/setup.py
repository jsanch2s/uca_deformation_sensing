#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ucr_experiment_evaluation'],
    package_dir={
        'ucr_experiment_evaluation': 'src/ucr_experiment_evaluation',
    }
)

setup(**d)
