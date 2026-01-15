import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'real_hand_mujoco_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='realhand',
    maintainer_email='realhand@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'real_hand_mujoco_ros2_node=real_hand_mujoco_ros2.real_hand_mujoco_ros2:main'
        ],
    },
)
