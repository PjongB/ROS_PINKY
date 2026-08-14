import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'pinky_project_1'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + glob('launch/*.launch.xml'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'map'),
            glob('map/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PjongB',
    maintainer_email='PjongB@users.noreply.github.com',
    description='Autonomous patrol and precision docking project for Pinky.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_manager = pinky_project_1.patrol_manager:main',
            'docking_controller = pinky_project_1.docking_controller:main',
        ],
    },
)
