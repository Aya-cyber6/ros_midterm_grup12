from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'midterm_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xya',
    maintainer_email='aya17khalifa@gmail.com',
    description='Midterm Project - Mobile Robot Simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
           'obstacle_avoider = midterm_robot.obstacle_avoider:main',
        ],
    },
)
