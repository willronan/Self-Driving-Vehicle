from setuptools import setup

package_name = 'mren_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/qcar_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='William Ronan',
    maintainer_email='willronan4@gmail.com',
    description='Self-driving QCar system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    'sign_detector = mren_ros.sign_detection:main',
                    'lane_detector = mren_ros.lane_detection:main',
                    'driver = mren_ros.motor:main',
                    'lidar = mren_ros.lidar:main',
            ],
    },
)
