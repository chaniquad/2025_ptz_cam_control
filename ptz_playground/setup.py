from setuptools import find_packages, setup

package_name = 'ptz_playground'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chani',
    maintainer_email='chani@todo.todo',
    description='Pan–tilt control node for PTZ Playground',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pan_tilt_controller = ptz_playground.pan_tilt_controller:main',
            '2d_to_pan_tilt_controller = ptz_playground.2d_to_pan_tilt_controller:main',
        ],
    },
)

