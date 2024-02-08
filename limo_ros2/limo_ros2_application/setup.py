from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'limo_ros2_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wego',
    maintainer_email='changmin@wego-robotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_limo = limo_ros2_application.move_limo:main',
            'limo_e_stop = limo_ros2_application.limo_e_stop:main',
            'detect_line = limo_ros2_application.detect_line:main',
            'limo_control = limo_ros2_application.limo_control:main',
        ],
    },
)
