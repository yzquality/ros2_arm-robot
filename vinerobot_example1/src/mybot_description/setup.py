from setuptools import find_packages, setup
from glob import glob #这里
import os #这里

package_name = 'mybot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
       ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')), #这里
	(os.path.join('share', package_name, 'meshes'), glob('meshes/**')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yzquality',
    maintainer_email='yzquality@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
