from setuptools import find_packages, setup
import os
from glob import glob
# from setuptools import setup

package_name = 'rov'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niran',
    maintainer_email='nirans2002@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rov = rov.rov:main',
            'hull_conditions = rov.hull:main',
            'bldc = rov.bldc:main',
            'imu = rov.imu:main',
            'joy_2_cmd = rov.joy_2_cmd:main',

        ],
    },
)
