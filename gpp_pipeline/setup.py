import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gpp_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "launch/utils"), glob("launch/utils/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "behavior_trees"), glob("behavior_trees/*.xml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henrik Lurz',
    maintainer_email='lurz@match.uni-hannover.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "send_new_goal_node = gpp_pipeline.send_new_goal_node:main",
            "create_position_list_node = gpp_pipeline.create_position_list_node:main",
            "static_frame_publisher_node = gpp_pipeline.static_frame_publisher_node:main",
            "path_listener_node = gpp_pipeline.path_listener_node:main",
            "test_node = gpp_pipeline.test_node:main"
        ],
    },
)
