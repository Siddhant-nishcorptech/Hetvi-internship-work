from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wheelchair_isaac'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install every file in urdf/ individually (exclude subdirectories)
        *[
            (os.path.join('share', package_name, 'urdf'), [f])
            for f in glob('urdf/*') if os.path.isfile(f)
        ],
        # Install every file in meshes/ individually  
        *[
            (os.path.join('share', package_name, 'meshes'), [f])
            for f in glob('meshes/*')
        ],
        *[
            (os.path.join('share', package_name, 'launch'), [f])
            for f in glob('launch/*')
        ],
        *[
            (os.path.join('share', package_name, 'scripts'), [f])
            for f in glob('scripts/*')
        ],
        # Install config (PID params, etc.)
        *[
            (os.path.join('share', package_name, 'config'), [f])
            for f in glob('config/*')
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hetvi_311',
    maintainer_email='hetvi@todo.todo',
    description='Isaac Sim integration for wheelchair balancing robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balance_controller = wheelchair_isaac.balance_controller_node:main',
            'teleop_keyboard = wheelchair_isaac.teleop_keyboard_node:main',
        ],
    },
)
