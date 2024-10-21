from setuptools import setup
import os
from glob import glob

package_name = 'my_wheelchair'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dynage-technologies',
    maintainer_email='hello@dynage.technology',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_node = my_wheelchair.mapping_node:main',
            'sensors_node = my_wheelchair.sensors_node:main',
            'navigation_node = my_wheelchair.navigation_node:main',
            'voice_recognition_node = my_wheelchair.voice_recognition_node:main',
            'navigation_feedback_node = my_wheelchair.scripts.navigation_command_node:main',
            'voice_feedback_node = my_wheelchair.scripts.voice_recognition_node:main',
            'headless_image_viewer_node = my_wheelchair.image_viewer_node:main',
        ],
    },
)
