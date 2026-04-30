from setuptools import find_packages, setup

package_name = 'my_first_node'

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
    maintainer='liuning',
    maintainer_email='star05w1@126.com',
    description='my first node,learn hard~!',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dual_motor_controller = my_first_node.dual_motor_controller:main',
            'keyboard_control = my_first_node.keyboard_control:main',
        ],
    },
)
