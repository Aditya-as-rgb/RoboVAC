from glob import glob

from setuptools import find_packages, setup

package_name = 'vacuum_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.json')),
        ('share/' + package_name + '/assets', glob('assets/*')),
        ('share/applications', ['vacuum_gui.desktop']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adi',
    maintainer_email='adi@example.com',
    description='PySide6 control panel for the RoboVAC ROS 2 simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vacuum_gui = vacuum_gui.app:main',
            'vacuum_gui_install_desktop = vacuum_gui.install_desktop:main',
        ],
    },
)
