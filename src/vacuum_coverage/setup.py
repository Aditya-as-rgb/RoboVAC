from glob import glob

from setuptools import find_packages, setup

package_name = 'vacuum_coverage'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adi',
    maintainer_email='adi@example.com',
    description='Wall-pass and lane-sweep coverage mission for the vacuum robot, with a coverage meter.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_mission = vacuum_coverage.mission_node:main',
            'coverage_meter = vacuum_coverage.meter_node:main',
            'coverage_preview = vacuum_coverage.preview:main',
        ],
    },
)
