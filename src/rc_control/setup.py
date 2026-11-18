from glob import glob

from setuptools import find_packages, setup

package_name = 'rc_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Björn Bergström',
    maintainer_email='bjorn.bergstr@outlook.com',
    description='Drive controllers for the RC racing stack, and the mux that picks one',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = rc_control.pure_pursuit_node:main',
            'follow_node = rc_control.follow_node:main',
            'drive_mux_node = rc_control.drive_mux_node:main',
            'keyboard_teleop_node = rc_control.keyboard_teleop_node:main',
        ],
    },
)
