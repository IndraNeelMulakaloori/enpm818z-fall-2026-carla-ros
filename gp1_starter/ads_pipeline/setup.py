from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ads_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GP1 Team X',                      # TODO: your team
    maintainer_email='team@umd.edu',              # TODO: your team
    description='ENPM818Z cumulative ADS pipeline (GP1 to GP4).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # TODO (Task 1): one line per node you create.
            'sensor_manager = ads_pipeline.sensor_manager:main',
        ],
    },
)
