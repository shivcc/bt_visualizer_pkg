from setuptools import find_packages, setup

package_name = 'bt_visualizer_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@todo.todo',
    description='A simple graphical tool to visualize Behavior Trees from XML files.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualizer = bt_visualizer_pkg.visualizer_node:main',
        ],
    },
)