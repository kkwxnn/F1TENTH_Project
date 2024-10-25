from setuptools import find_packages
from setuptools import setup

setup(
    name='ackerman_odometry',
    version='0.0.0',
    packages=find_packages(
        include=('ackerman_odometry', 'ackerman_odometry.*')),
)
