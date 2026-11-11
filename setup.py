from setuptools import setup

package_name = 'armdoc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darshmenon',
    maintainer_email='darshmenon02@gmail.com',
    description='Attach to a running ros2_control / MoveIt 2 arm and say why it is not moving.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['armdoc = armdoc.doctor:main']},
)
