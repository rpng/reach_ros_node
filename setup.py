from setuptools import setup

package_name = 'reach_ros_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Geneva',
    maintainer_email='pgneva@udel.edu',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmea_tcp_driver = reach_ros_node.nmea_tcp_driver:main',
        ],
    },
)
