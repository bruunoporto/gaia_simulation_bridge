from setuptools import setup
import os
from glob import glob

package_name = 'gaia_simulation_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bruno',
    maintainer_email='bruno@example.com',
    description='Bridge between gaia_controller and cylidrone simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gaia_bridge_node = gaia_simulation_bridge.gaia_bridge_node:main',
            'robot_controller_example = gaia_simulation_bridge.robot_controller_example:main',
        ],
    },
)
