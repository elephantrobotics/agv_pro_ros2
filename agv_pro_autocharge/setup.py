from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'agv_pro_autocharge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='AGV automatic charging system for ROS2 Humble',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'combined_auto_recharger = agv_pro_autocharge.combined_auto_recharger:main',
        ],
    },
)
