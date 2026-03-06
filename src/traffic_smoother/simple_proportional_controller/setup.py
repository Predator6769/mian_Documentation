from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simple_proportional_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple Proportional Controller for Traffic Smoothing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_proportional_controller = simple_proportional_controller.simple_proportional_controller:main',
            'live_acc_plotter = simple_proportional_controller.plots:main',
        ],
    },
)