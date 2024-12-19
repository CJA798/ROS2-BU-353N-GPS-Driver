
import os
from setuptools import find_packages, setup
from glob import glob
from setuptools import setup

package_name = 'gps_driver'
submodules = f'{package_name}/submodels'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CJA798',
    maintainer_email='',
    description='GNSS Driver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'talker = gps_driver.driver:main',
        ],
    },
)
