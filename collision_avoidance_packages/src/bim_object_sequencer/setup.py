from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bim_object_sequencer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required index for ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Resource folder (installs all files in resource/)
        # This should include resource/file.ifc
        ('share/' + package_name + '/resource', glob('resource/*')),
        # Optional: launch folder if you add launch files later
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bidinger',
    maintainer_email='bidinger@todo.todo',
    description=(
        'BIM Object Sequencer package: '
        'Provides sequencer and spawner nodes that use IfcOpenShell to import '
        'IFC entities and manage their placement in ROS2/MoveIt.'
    ),
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sequencer = bim_object_sequencer.sequencer:main',
            'spawner   = bim_object_sequencer.spawner:main',
            'attached_base = bim_object_sequencer.attached_base:main',
        ],
    },
)

