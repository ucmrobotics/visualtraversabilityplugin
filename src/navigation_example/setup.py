import os
from glob import glob
from setuptools import setup

package_name = 'navigation_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'),
            glob(os.path.join('params','*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ettore Sani',
    maintainer_email='ettoresani0@gmail.com',
    description='This package provides an example of the usage of the grass plugin.',
    license='Apache Licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
