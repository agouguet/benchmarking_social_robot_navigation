from setuptools import find_packages
from setuptools import setup

setup(
    name='social_metrics',
    version='0.0.0',
    packages=find_packages(
        include=('social_metrics', 'social_metrics.*')),
)
