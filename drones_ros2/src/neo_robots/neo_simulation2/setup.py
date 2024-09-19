from setuptools import setup

from glob import glob
import os

package_name = 'neo_simulation2'

setup(
    name=package_name,
    version='1.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pradheep padmanabhan',
    maintainer_email='padhupradheep@gmail.com',
    description='ROS-2 Simulation packages for neobotix robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_robot = neo_simulation2.control_robot:main',
        ],
    },
)