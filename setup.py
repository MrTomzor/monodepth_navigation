from setuptools import setup, find_packages

package_name = 'monodepth_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='tom@todo.todo',
    description='ROS2 port of monodepth',
    license='MIT',
    entry_points={
        'console_scripts': [
            'monocular_depth_estimator = monodepth.monocular_depth_estimator:main',
            'navigation_controller = monodepth.navigation_controller:main',
        ],
    },
)
