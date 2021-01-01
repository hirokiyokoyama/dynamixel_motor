#!/usr/bin/env python

from setuptools import setup

package_name = 'dynamixel_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'dynamixel_driver.dynamixel_io',
        'dynamixel_driver.dynamixel_const',
        'dynamixel_driver.dynamixel_serial_proxy',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='user',
    author_email="user@todo.todo",
    maintainer='user',
    maintainer_email="user@todo.todo",
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO: Package description.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_serial_proxy= dynamixel_driver.dynamixel_serial_proxy:main',
            'info_dump= scripts.info_dump:main',
            'change_id= scripts.change_id:main',
            'set_servo_config= scripts.set_servo_config:main',
            'set_torque= scripts.set_torque:main',
        ],
    },
)

