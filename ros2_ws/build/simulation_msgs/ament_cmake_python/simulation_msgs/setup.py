from setuptools import find_packages
from setuptools import setup

setup(
    name='simulation_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('simulation_msgs', 'simulation_msgs.*')),
)
