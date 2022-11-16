import os
from glob import glob
from setuptools import setup

package_name = 'webremote'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'template'), glob('webremote/template/*')),
    #         (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ns',
    maintainer_email='noushad.sojib@unh.edu',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webremote= webremote.webremote:main',
            'odom_listener= webremote.odom_listener:main',
            'helper=webremote.helper:main'
        ],
    },
)

 