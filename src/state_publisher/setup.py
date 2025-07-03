from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
package_name = 'state_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
  (os.path.join('share', package_name, 'launch'), glob('launch/*')),
  (os.path.join('share', package_name), glob('urdf/*')),
       (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
       # in your data_files list:
(os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deeric',
    maintainer_email='onganjuk@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
'state_publisher = state_publisher.state_publisher:main'
        ],
    },
)
