from setuptools import setup

package_name = 'follow_the_gap_v0_ride'

setup(
    name=package_name,
    version='0.2.4',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name,
            ['launch/start.launch.py']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaroslav Klapálek',
    maintainer_email='klapajar@fel.cvut.cz',
    description='Handles integration of follow_the_gap_v0 with drive_api.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ride_node = follow_the_gap_v0_ride.ride_node:main',
        ],
    },
)
