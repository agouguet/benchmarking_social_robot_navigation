from setuptools import find_packages
from setuptools import setup

setup(
    name='metric_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('metric_msgs', 'metric_msgs.*')),
)
