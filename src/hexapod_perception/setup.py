from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'hexapod_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='OpenCV DNN person detection and following',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'person_detector = hexapod_perception.person_detector:main',
            'person_follower = hexapod_perception.person_follower:main',
            'review_recorder = hexapod_perception.review_recorder:main',
            'pi_camera = hexapod_perception.pi_camera:main',
        ],
    },
)
