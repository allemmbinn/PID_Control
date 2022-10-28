from setuptools import setup

package_name = 'pid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='allemmbinn',
    maintainer_email='allemmbinn@gmail.com',
    description='PID Control for Turtlebot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = pid_control.turtlebot3_control:main',
        ],
    },
)
