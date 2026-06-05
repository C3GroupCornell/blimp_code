from setuptools import find_packages, setup
from glob import glob

package_name = 'blimp_linux'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='c3',
    maintainer_email='c3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_node = blimp_linux.serial_node:main',
            'setup_gui_node = blimp_linux.setup_gui_node:main',
            'teleop_receiver = blimp_linux.teleop_receiver:main',
            'optitrack_node = blimp_linux.optitrack_node:main',
            'agent_manager = blimp_linux.agent_manager:main',
            'low_level_controller = blimp_linux.low_level_controller:main',
        ],
    },
)
