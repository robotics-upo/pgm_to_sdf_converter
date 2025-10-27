from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pgm_to_sdf_converter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'examples'), glob('examples/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miguel Escudero',
    maintainer_email='mescjim@upo.es',
    description='ROS2 package to convert occupancy maps (PGM + YAML) to Gazebo SDF world files',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pgm_to_sdf = pgm_to_sdf_converter.converter_node:main',
        ],
    },
)
