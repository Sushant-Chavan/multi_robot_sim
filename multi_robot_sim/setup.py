#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['multi_robot_sim'],
   package_dir={'multi_robot_sim': 'ros/src/multi_robot_sim'}
)

setup(**d)
