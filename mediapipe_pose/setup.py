from setuptools import find_packages, setup

package_name = 'mediapipe_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','mediapipe'],
    options={
        'build_scripts': {
            # use /usr/bin/env python3 for the shebang in console_scripts wrappers
            'executable': '/usr/bin/env python3',
        },
    },
    zip_safe=True,
    maintainer='Nathan Marin',
    maintainer_email='martnat8@oregonstate.edu',
    description='MediaPose node for ROS2',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_estimator = mediapipe_pose.pose_estimator:main',
        ],
    },
)
