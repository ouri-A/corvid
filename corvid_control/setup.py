from setuptools import find_packages, setup

package_name = 'corvid_control'

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
    maintainer='Anshul Kumar Paliwal',
    maintainer_email='ouri200425@gmail.com',
    description='Holds a PX4 drone over an ArUco marker with Offboard velocity setpoints',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'station_keeper = corvid_control.station_keeper:main'
        ],
    },
)
