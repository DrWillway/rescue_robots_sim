from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolobot_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, "launch"), glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='daryna.datsenko@vortex-colab.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_recognition_yolo = yolobot_recognition.ros_recognition_yolo:main'
        ],
    },
)