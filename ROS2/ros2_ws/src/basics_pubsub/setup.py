from setuptools import setup

package_name = 'basics_pubsub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Lockwood',
    maintainer_email='andrew@example.com',
    description='Simple ROS2 pub/sub demo package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_pub = basics_pubsub.simple_pub:main',
            'simple_sub = basics_pubsub.simple_sub:main',
        ],
    },
)
