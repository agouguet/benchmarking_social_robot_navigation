from setuptools import find_packages
from setuptools import setup

setup(
    name='agent_trajectory_prediction',
    version='0.0.0',
    packages=find_packages(
        include=('agent_trajectory_prediction', 'agent_trajectory_prediction.*')),
)
