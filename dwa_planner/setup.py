"""
Custom DWA Local Planner Setup

Author: Parth Singh
Institution: Carnegie Mellon University
Program: MS Robotics Systems Development
"""

from setuptools import find_packages, setup

package_name = 'dwa_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Ensure resource folder exists
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'transforms3d'],
    zip_safe=True,
    maintainer='Parth Singh',
    maintainer_email='parthsin@cs.cmu.edu',
    description='Dynamic Window Approach (DWA) planner for ROS2 TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dwa_planner = dwa_planner.dwa_planner:main',
        ],
    },
)
