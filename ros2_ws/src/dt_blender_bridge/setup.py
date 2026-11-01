from setuptools import find_packages, setup

package_name = 'dt_blender_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shashank Sharma',
    maintainer_email='shashank.sharma.280201@gmail.com',
    description='ROS 2 -> Blender live bridge for the digital-twin sim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['dt_blender_bridge = dt_blender_bridge.node:main']},
)
