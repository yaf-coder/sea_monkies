from setuptools import find_packages, setup

package_name = 'sea_monkies'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yafeekhan',
    maintainer_email='yafee22.khan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement = sea_monkies.movement:main',
            'arming = sea_monkies.arming:main',
            'pressure = sea_monkies.pressure:main',
            'pid_controller = sea_monkies.pid_controller:main',
            'angle_controller = sea_monkies.angle_controller:main',
            'cam = sea_monkies.camera_sub:main',
            'live_april_tags = sea_monkies.live_april_tags:main'
        ],
    },
)
