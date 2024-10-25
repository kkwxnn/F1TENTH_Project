from setuptools import find_packages
from setuptools import setup

setup(
    name='example_description',
    version='0.0.0',
    packages=find_packages(
        include=('example_description', 'example_description.*')),
)
