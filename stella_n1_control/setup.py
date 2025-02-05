from setuptools import find_packages, setup

package_name = 'stella_n1_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools>=70.0.0'],
    zip_safe=True,
    maintainer='dongwan',
    maintainer_email='dongwan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        	'road_detection = stella_n1_control.road_detection:main',
        	'velocity_control = stella_n1_control.velocity_control:main',
        ],
    },
)
