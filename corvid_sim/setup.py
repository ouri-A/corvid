from setuptools import find_packages, setup

package_name = 'corvid_sim'

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
    description='Gazebo marker mover for tracking tests',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'marker_mover = corvid_sim.marker_mover:main'
        ],
    },
)
