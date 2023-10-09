from setuptools import find_packages, setup

package_name = 'depth_optimization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco De Simone',
    maintainer_email='marco1998.mds@gmail.com',
    description='This package includes alghoritms for 6D-pose post processing',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service = depth_optimization.depth_optimizer:main',
        ],
    },
)