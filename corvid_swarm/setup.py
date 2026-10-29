from setuptools import find_packages, setup

package_name = 'corvid_swarm'

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
    description='Coordinator and keyboard pilot for camera-guided ground robots',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coordinator = corvid_swarm.coordinator:main',
            'pilot = corvid_swarm.pilot:main',
        ],
    },
)
