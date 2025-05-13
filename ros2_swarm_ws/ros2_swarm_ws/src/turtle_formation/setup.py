#!/usr/bin/env python3

import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'turtle_formation'

# Data files to install:
data_files = [
    # Ament resource index marker
    ('share/ament_index/resource_index/packages',
     [os.path.join('resource', package_name)]),
    # package.xml
    (f'share/{package_name}', ['package.xml']),
]

# Install all launch/*.launch.py files under share/<pkg>/launch
launch_files = glob(os.path.join('launch', '*.launch.py'))
if launch_files:
    data_files.append((f'share/{package_name}/launch', launch_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Decentralized predator–prey swarm in turtlesim',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'predator_swarm = turtle_formation.predator_swarm:main',
            'prey_node       = turtle_formation.prey_node:main',
            'spawn_turtles  = turtle_formation.spawn_turtles:main',
        ],
    },
)

