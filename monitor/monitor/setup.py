from setuptools import setup

package_name = 'monitor'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/monitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROSMonitoring',
    maintainer_email='maintainer@example.com',
    description='Generated ROSMonitoring monitors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'monitor_0 = monitor.monitor_0:main',
        ],
    },
)
