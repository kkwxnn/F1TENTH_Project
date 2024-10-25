from setuptools import find_packages
from setuptools import setup

setup(
    name='robot_bridge',
    version='0.0.0',
    packages=find_packages(
        include=('robot_bridge', 'robot_bridge.*')),
)
