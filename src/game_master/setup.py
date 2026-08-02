from setuptools import setup
from glob import glob

package_name = 'game_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carrossel Caipira',
    maintainer_email='fernando.murusaki@unesp.br',
    description='Arbitro da partida e telas da feira',
    license='MIT',
    entry_points={
        'console_scripts': [
            'master_node = game_master.master_node:main',
        ],
    },
)
