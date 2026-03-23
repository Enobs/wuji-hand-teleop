from setuptools import setup, find_packages

package_name = 'collision_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/collision_config.yaml',
        ]),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Preh Robotics',
    maintainer_email='dev@preh.com',
    description='Real-time collision detection and safe projection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'collision_manager = collision_manager.collision_manager_node:main',
        ],
    },
)
