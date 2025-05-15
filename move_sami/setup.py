from setuptools import find_packages, setup

package_name = 'move_sami'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # register the package with ament
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # install package manifest
        ('share/' + package_name, ['package.xml']),
        # install your config JSONs (paths are relative to this setup.py)
        (f'share/{package_name}/config', [
            'config/Joint_config.json',
            'config/Emote.json',
        ]),
    ],

    install_requires=[
        'setuptools',
        'playsound',    # for AudioManager
        'pyserial',     # for serial communication
    ],
    zip_safe=True,
    maintainer='Nathan M.',
    maintainer_email='martnat8@oregonstate.edu',
    description='ROS2 node for driving SAMI joints via serial commands',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = move_sami.move_sami:main',
        ],
    },
)
