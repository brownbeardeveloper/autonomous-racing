from glob import glob

from setuptools import find_packages, setup

package_name = 'rc_localization'

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
    description='Localization for the RC racing stack: a particle filter on a saved track map, '
                'and slam_toolbox to build the map',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_odom_broadcaster_node = rc_localization.map_odom_broadcaster_node:main',
            'particle_filter_node = rc_localization.particle_filter_node:main',
        ],
    },
)
