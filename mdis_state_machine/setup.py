#!/usr/bin/python3.8

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mdis_state_machine'],
    package_dir={'': 'src'}
)

setup(**d)
