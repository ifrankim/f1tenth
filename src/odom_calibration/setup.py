from setuptools import setup
from glob import glob
import os

package_name = 'odom_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rilbasestation02',
    maintainer_email='rilbasestation02@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_calibration = odom_calibration.odom_calibration:main',
            'square_movement = odom_calibration.square_movement:main',
        ],
    },
)
