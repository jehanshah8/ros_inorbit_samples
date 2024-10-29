from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inorbit_republisher_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
        'ament_index_python',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Jehan Shah',
    maintainer_email='jehan@inorbit.ai',
    description='ROS2 version of InOrbit republisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inorbit_republisher = inorbit_republisher_ros2.main:main',
        ],
    },
)
