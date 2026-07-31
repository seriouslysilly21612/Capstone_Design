from setuptools import find_packages, setup

package_name = 'raon_operator_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kww642@mju.ac.kr',
    description='Desktop PyQt5 operator console for the KV260 pick&place pipeline (Phase 1: observe only)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'operator_console = raon_operator_gui.app:main',
        ],
    },
)
