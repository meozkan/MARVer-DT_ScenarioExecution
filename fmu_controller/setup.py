import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fmu_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # FMU dosyasını share/fmu_controller/ dizinine kopyalar:
        (os.path.join('share', package_name), glob('fmu_controller/*.fmu')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='baki',
    maintainer_email='todo@todo.com',
    description='FMU Controller Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fmu_node = fmu_controller.fmu_node:main',
            'fmu_action_server = fmu_controller.fmu_controller_action_server:main',
        ],
    },
)
