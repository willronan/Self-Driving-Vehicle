from setuptools import setup

package_name = 'lab6_groupX'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar = lab6_groupX.lab6_lidar:main',
            'motor = lab6_groupX.lab6_motor:main',
            'steering = lab6_groupX.lab6_lane_detection:main',
            'sign_detection = lab6_groupX.lab6_sign_detection:main',
        ],
    },
)
