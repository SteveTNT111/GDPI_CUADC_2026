from setuptools import setup, find_packages

package_name = 'd435i_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/d435i_detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cuadc',
    maintainer_email='cuadc@todo.todo',
    description='D435i depth camera driver + yellow circle detector for ROS2 Foxy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd435i_publisher  = d435i_detector.d435i_publisher:main',
            'yellow_detector  = d435i_detector.yellow_detector:main',
        ],
    },
)
