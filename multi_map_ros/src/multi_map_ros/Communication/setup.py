from setuptools import find_packages, setup

setup(
    name='coms',
    packages=find_packages(include=['coms']),
    version='0.0.1',
    description='Provides encoding and decoding of occupancy grids for transport over networks.',
    author='Connor Mclaughlin, Tyler Ferrara, Peter Nikopoulos',
    license='MIT',
    install_requires=['numpy==1.21.4'],
    setup_requires=['pytest-runner'],
    tests_require=['pytest==4.4.1'],
    test_suite='tests',
)
