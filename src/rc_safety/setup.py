from glob import glob

from setuptools import find_packages, setup

package_name = 'rc_safety'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Björn Bergström',
    maintainer_email='bjorn.bergstr@outlook.com',
    description='Emergency braking and the remote e-stop for the RC racing stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = rc_safety.safety_node:main',
            'estop_bridge_node = rc_safety.estop_bridge_node:main',
        ],
    },
)
