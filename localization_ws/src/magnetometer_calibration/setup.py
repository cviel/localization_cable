import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'magnetometer_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gillarlo',
    maintainer_email='louis.gillard@ensta-bretagne.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'magnetometer_publisher = magnetometer_calibration.magnetometer_publisher:main',
            'magnetometer_visualizer = magnetometer_calibration.magnetometer_visualizer:main',
        ],
    },
)
