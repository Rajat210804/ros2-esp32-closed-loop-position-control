from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'single_axis'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rajat Goyal',
    maintainer_email='goyrajat82@gmail.com',
    description='Closed-loop single-axis driver and repeatability test for ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'axis_driver = single_axis.axis_driver_node:main',
            'repeatability_test = single_axis.repeatability_test:main',
        ],
    },
)
