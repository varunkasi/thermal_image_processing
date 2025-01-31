
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'thermal_image_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        # https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='varunkasi',
    maintainer_email='vkasired@andrew.cmu.edu',
    description='A repository for thermal image processing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processor = thermal_image_processing.processor:main',
        ],
    },
)
