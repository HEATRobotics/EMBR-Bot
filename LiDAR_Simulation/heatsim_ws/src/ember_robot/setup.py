from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'ember_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # MARK: Adding Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
        (os.path.join('share', package_name, '3D_LiDAR'), glob('3D_LiDAR/*')),
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maison',
    maintainer_email='maison.personal03@gmail.com',
    description='TODO: Package Description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    }
)