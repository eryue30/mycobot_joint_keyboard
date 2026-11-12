# -*- coding: utf-8 -*-
"""ament_python 安装入口：安装节点、参数文件和中文使用说明。"""
from glob import glob
from setuptools import find_packages, setup

PACKAGE = 'mycobot_joint_keyboard'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md', 'README_中文.md']),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE + '/tools', glob('tools/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='myCobot workspace maintainer',
    maintainer_email='maintainer@example.com',
    description='myCobot 280 M5 joint keyboard service client',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_keyboard = mycobot_joint_keyboard.keyboard_node:main',
        ],
    },
)
