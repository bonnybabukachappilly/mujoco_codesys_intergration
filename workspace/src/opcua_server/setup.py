import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'opcua_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'asyncua', 'pyyaml'],
    zip_safe=True,
    maintainer='bonnybk',
    maintainer_email='bonnybabukachappilly@gmail.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['opcua_server = opcua_server.main:main'],
    },
)
