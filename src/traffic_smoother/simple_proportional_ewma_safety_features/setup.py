from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simple_proportional_ewma_safety_features'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='prashanth',
    maintainer_email='sankarap@purdue.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_proportional_ewma_safety_features = simple_proportional_ewma_safety_features.simple_proportional_ewma_safety_features:main'
        ],
    },
)
