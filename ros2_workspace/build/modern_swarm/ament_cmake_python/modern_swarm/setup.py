from setuptools import find_packages
from setuptools import setup

setup(
    name='modern_swarm',
    version='1.0.0',
    packages=find_packages(
        include=('modern_swarm', 'modern_swarm.*')),
)
