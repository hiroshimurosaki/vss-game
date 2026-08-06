from setuptools import setup
from glob import glob

package_name = 'vision_game'

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
    description='Visão do campo: câmera para /game_data',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = vision_game.vision_node:main',
        ],
    },
)
