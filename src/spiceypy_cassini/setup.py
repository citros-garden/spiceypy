from setuptools import setup
from glob import glob
import os

package_name = 'spiceypy_cassini'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'resource'), glob('resource/cassMetaK.txt')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='gleb@lulav.space',
    description='Simple Spiceypy example showing the position of the Cassini spacecraft relative to the barycenter of Saturn',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spiceypy_cassini = spiceypy_cassini.spiceypy_cassini:main'
        ],
    },
)
