from setuptools import find_packages, setup

package_name = 'agv_pro_calibration'

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
    maintainer='elephant',
    maintainer_email='elephant@todo.todo',
    description='AGV Pro calibration tools for odom, IMU, and TF health check',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_linear_calib = agv_pro_calibration.odom_linear_calib:main',
            'odom_yaw_calib = agv_pro_calibration.odom_yaw_calib:main',
            'final_pose_refiner = agv_pro_calibration.final_pose_refiner:main',
            'navigate_to_pose_refiner_proxy = agv_pro_calibration.navigate_to_pose_refiner_proxy:main',
        ],
    },
)
