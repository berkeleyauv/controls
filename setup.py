import os
from setuptools import setup
from glob import glob

package_name = 'controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashwin',
    maintainer_email='ashwinvangipuram@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init = controls.sub_init:processInput',
            'velocity_transformer = controls.velocity_transformer:main',
            'orientation_control = controls.orientation_control:main'
        ],
    },
)
