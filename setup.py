from setuptools import setup

package_name = 'my_wheelchair'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_node = my_wheelchair.mapping_node:main',
            'sensors_node = my_wheelchair.sensors_node:main',
            'navigation_node = my_wheelchair.navigation_node:main',
        ],
    },
)
