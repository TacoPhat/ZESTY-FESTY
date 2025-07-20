import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tutorial_ardusub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niche',
    maintainer_email='nichelle.thinagar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber = tutorial_ardusub.bluerov2_sensors:main',
            'arm = tutorial_ardusub.bluerov2_arm:main',
            'move = tutorial_ardusub.bluerov2_move:main',
            'heading = tutorial_ardusub.heading:main'
            'depth = tutorial_ardusub.depth:main'

        ],
    },
)