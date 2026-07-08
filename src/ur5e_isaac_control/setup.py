from setuptools import find_packages, setup

package_name = 'ur5e_isaac_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aravindsuresh16',
    maintainer_email='aravind162000@gmail.com',
    description='Joint control scripts for a UR5e arm simulated in Isaac Sim over ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_command_publisher = ur5e_isaac_control.joint_command_publisher:main',
        ],
    },
)
