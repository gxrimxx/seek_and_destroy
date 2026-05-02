from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'seek_and_destroy_brain'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Upgraded to the robust, OS-agnostic pathing:
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'state_machine = seek_and_destroy_brain.state_machine:main',
            'color_detector = seek_and_destroy_brain.color_detector:main',
            'navigator = seek_and_destroy_brain.navigator:main',
            'mission_gui = seek_and_destroy_brain.mission_gui:main',
        ],
    },
)