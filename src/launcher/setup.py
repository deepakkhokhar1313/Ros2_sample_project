from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launcher'), 
          glob(os.path.join('launcher', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deepak',
    maintainer_email='deepakkhokhar1313@gmail.com',
    description='Ros2 Package to launch the Nodes.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'master_launcher = launcher.master_launcher:generate_launch_description',
        ],
    },
)