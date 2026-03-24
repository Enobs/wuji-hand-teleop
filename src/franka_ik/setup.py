from setuptools import setup

package_name = 'franka_ik'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/franka_pico_viz.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/franka_pico.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'franka_ik_node = franka_ik.franka_ik_node:main',
            'franka_pico_input_node = franka_ik.franka_pico_input_node:main',
        ],
    },
)
