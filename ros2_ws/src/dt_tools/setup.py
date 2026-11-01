from setuptools import find_packages, setup

package_name = 'dt_tools'

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
    description='Digital-twin capture tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dt_monitor = dt_tools.monitor:main',
            'dt_session = dt_tools.session:main',
            'dt_glim_config = dt_tools.glim_config:main',
            'dt_apply_camera_calib = dt_tools.apply_camera_calib:main',
            'dt_overlay_check = dt_tools.overlay_check:main',
        ],
    },
)
