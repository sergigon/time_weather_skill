#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


d = generate_distutils_setup(
    # don't do this unless you want a globally visible script
    # scripts=['bin/myscript'],
    packages=['time_weather_skill'],
    package_dir={'': 'include'}  # ,
    #     package_xml_path=pkg_path
)

setup(**d)
