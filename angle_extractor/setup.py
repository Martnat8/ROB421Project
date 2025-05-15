from setuptools import find_packages, setup

package_name = 'angle_extractor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['mediapipe', 'numpy'],
    zip_safe=True,
    maintainer='Teft',
    maintainer_email='martnat8@oregonstate.edu',
    description='Package that subscribes to mediapipe landmark node and extracts angles for SAMI',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'angle_publisher = angle_extractor.anglepublisher:main',
            'angle_corrector = angle_extractor.correct_angles:main'
        ],
    },
)
