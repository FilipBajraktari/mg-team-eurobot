from setuptools import setup
from setuptools import Extension

setup(
    name='rtrrt-lib',
    version='1',
    ext_modules=[Extension('_rtrrt',['main.cpp'])]
)