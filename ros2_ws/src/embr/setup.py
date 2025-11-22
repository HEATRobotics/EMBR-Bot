from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'embr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'getCube = embr.getCube:main',
            'getTemp = embr.getTemp:main',
            'sendRf = embr.sendRf:main',
            'getCube_v2 = embr.getCube_v2:main',
            'getTemp_v2 = embr.getTemp_v2:main',
            'sendRf_v2 = embr.sendRf_v2:main',
        ],
    },
)
