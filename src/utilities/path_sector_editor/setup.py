from setuptools import setup
import os
from glob import glob

package_name = 'path_sector_editor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dawgs_nx',
    maintainer_email='dallma300@gmail.com',
    description='Interactive sector-based path tuning tool with matplotlib visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_sector_editor_node = path_sector_editor.path_sector_editor_node:main',
        ],
    },
)
