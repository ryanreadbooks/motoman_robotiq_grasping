#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['inference'],
    package_dir={'': 'src'})

#这就是把上马那个setup_args里的东西塞到这个函数里，也可以直接在setup()里写
setup(**setup_args)