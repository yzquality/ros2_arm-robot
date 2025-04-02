from setuptools import find_packages, setup

package_name = 'depth_obstacle_detector'

setup(
    name=package_name,
    version='0.0.0',
packages=find_packages(include=['depth_obstacle_detector', 'depth_obstacle_detector.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'depth_detector = depth_obstacle_detector.depth_detector:main',
    ],
    },
)
