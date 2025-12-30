from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotboy',
    maintainer_email='kalhapure.swapnil@gmail.com',
    description='Configurable swerve drive controller package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = robot_controller.controller_node:main',
            'visualizer_node = robot_controller.visualizer_node:main',
            'steering_monitor = robot_controller.steering_monitor:main',
            'gazebo_monitor = robot_controller.gazebo_monitor:main',
            'diff_swerve_node = robot_controller.diff_swerve_node:main',
            'simple_swerve_controller = robot_controller.simple_swerve_node:main',
            'swerve_visualizer = robot_controller.swerve_visualizer:main',
        ],
    },
)
