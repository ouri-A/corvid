from setuptools import find_packages, setup

package_name = 'corvid_perception'

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
    description='ArUco marker detection and metric pose from a downward camera',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'marker_detector = corvid_perception.marker_detector:main'
        ],
    },
)
