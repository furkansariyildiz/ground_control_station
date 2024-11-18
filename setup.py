import os
from setuptools import setup
from glob import glob

package_name = 'ground_control_station'
qt_package_name = 'qt'
vehicle_package_name = 'vehicles'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    packages=[package_name, qt_package_name, vehicle_package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Furkan',
    author_email='furkansariyildiz@windowslive.com',
    maintainer='Furkan Sariyildiz',
    maintainer_email='furkansariyildiz@windowslive.com',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Ground Control Station for autonomous vehicles via ROS2 and PyQt5',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_control_station = ground_control_station.ground_control_station:main',
        ],
    }
)