from setuptools import find_packages, setup

package_name = 'waypoints'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'rclpy_action', 'gps_interfaces'],
    zip_safe=True,
    maintainer='hager',
    maintainer_email='hajermajed22@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_node = waypoints.waypoint:main',
        ],
    },
)
