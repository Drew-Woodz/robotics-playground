from setuptools import setup
from glob import glob
import os

package_name = 'sim_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models', 'box'), ['models/box/model.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Lockwood',
    maintainer_email='andrew@example.com',
    description='Gazebo sim demo: launch + spawn a model',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
