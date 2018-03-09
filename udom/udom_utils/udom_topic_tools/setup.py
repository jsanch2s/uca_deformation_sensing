#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['udom_topic_tools'],
    package_dir={
        'udom_topic_tools': 'src/udom_topic_tools',
    }
)

setup(**d)
