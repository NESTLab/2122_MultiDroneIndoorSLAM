from setuptools import find_packages, setup

setup(
    name='mapmerge',
    packages=find_packages(include=['mapmerge']),
    version='0.0.1',
    description='Toolkit for performing keypoint detection and occupancy grid merges.',
    author='Connor Mclaughlin, Tyler Ferrara, Peter Nikopoulos',
    license='MIT',
    install_requires=['numpy==1.21.4', 'matplotlib==3.4.3', 'scipy==1.7.2', 'scikit-image==0.18.3', 'tqdm==4.62.3', 'opencv-python==4.5.4.58'],
    setup_requires=['pytest-runner'],
    tests_require=['pytest==4.4.1'],
    test_suite='tests',
    include_package_data=True,
)
