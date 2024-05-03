from setuptools import find_packages, setup

package_name = 'agpv_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/agpv_navigation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enrique Kortright',
    maintainer_email='ekortr1@lsu.edu',
    description='AGPV Package',
    license='LSU',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alert_handler = agpv_navigation.alert_handler:main',
            'visual_detector= agpv_navigation.visual_detector:main',
            'patrol_vehicle= agpv_navigation.patrol_vehicle:main',
        ],
    },
)
