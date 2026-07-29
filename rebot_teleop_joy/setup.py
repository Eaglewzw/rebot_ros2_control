from setuptools import find_packages, setup

package_name = 'rebot_teleop_joy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joy_teleop.launch.py']),
        ('share/' + package_name + '/config', ['config/teleop_joy.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eaglewzw',
    maintainer_email='1460853569@qq.com',
    description='Single Joy-Con teleoperation for the reBot Arm B601-DM.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = rebot_teleop_joy.teleop_node:main',
            'servo_minimal = rebot_teleop_joy.servo_minimal:main',
        ],
    },
)
