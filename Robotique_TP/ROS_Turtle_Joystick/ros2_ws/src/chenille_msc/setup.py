from setuptools import setup
import os
from glob import glob

package_name = 'chenille_msc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ensea',
    maintainer_email='you@example.com',
    description='Chenille joystick control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chenille_node = chenille_msc.chenille_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ajoute le dossier launch pour qu'il soit installé
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)
