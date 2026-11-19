from setuptools import setup

package_name = 'robot_arm_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Engineer',
    maintainer_email='engineer@example.com',
    description='Interactive Cartesian and gripper control for the 4-DOF robotic arm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_motion_controller = robot_arm_control.arm_motion_controller:main',
            'keyboard_control = robot_arm_control.keyboard_control:main',
            'robot_arm_gui = robot_arm_control.robot_arm_gui:main',
            'hand_control = robot_arm_control.hand_control:main',
        ],
    },
)
