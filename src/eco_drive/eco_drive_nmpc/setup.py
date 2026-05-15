from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'eco_drive_nmpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/eco_drive_nmpc.yaml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prashanth',
    maintainer_email='sankarap@purdue.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eco_drive_nmpc = eco_drive_nmpc.eco_drive_nmpc:main'
        ],
    },
)
