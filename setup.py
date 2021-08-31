import os
from glob import glob
from setuptools import setup

package_name = 'rqt_tf_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.ui')),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
        # (os.path.join(os.path.join('share', package_name), "config"), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='george',
    maintainer_email='george@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_listener = rqt_tf_tools.tf_listener:main'
        ],
    },
)
