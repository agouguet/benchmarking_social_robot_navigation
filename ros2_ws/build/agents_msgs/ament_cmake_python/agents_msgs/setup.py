from setuptools import find_packages
from setuptools import setup

setup(
    name='agents_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('agents_msgs', 'agents_msgs.*')),
)
