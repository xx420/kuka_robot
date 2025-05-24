from setuptools import find_packages, setup

package_name = 'marker_pose_follow'

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
    maintainer='rosuser',
    maintainer_email='imtiajsayem5@gmail.com',
    description='ROS2 package to move iiwa14 end-effector to tracked marker pose',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_follower = marker_pose_follow.pose_follower:main',
        ],
    },
)

