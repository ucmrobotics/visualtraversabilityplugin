import os
from glob import glob
from setuptools import setup

package_name = 'yolov8'

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
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ettore Sani',
    maintainer_email='ettoresani0@gmail.com',
    description='yolov8 traversable terrain segmentation pipeline',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_segment_node = yolov8.yolov8_segment_node:main',
        ],
    },
)
