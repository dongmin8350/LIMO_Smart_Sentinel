from setuptools import find_packages
from setuptools import setup

setup(
    name='instant_messaging_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('instant_messaging_interfaces', 'instant_messaging_interfaces.*')),
)
