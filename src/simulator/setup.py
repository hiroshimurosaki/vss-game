from setuptools import setup
from glob import glob

package_name = 'simulator'

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
    description='Simulador do campo VSS com GUI no navegador',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim_node = simulator.sim_node:main',
        ],
    },
)
