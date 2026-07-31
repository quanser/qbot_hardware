import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'qbot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rt_models'), glob('rt_models/*.rt-linux_qbot_platform')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zinan',
    maintainer_email='zinan.cen@quanser.com',
    description='Scan match-based localization nodes for QBot Platform in ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
entry_points={
    'console_scripts': [
        'ekf_localization_node = qbot_localization.ekf_localization_node:main',
        'scan_match_node = qbot_localization.scan_match_node:main',
    ],
    },
)
