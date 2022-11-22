from setuptools import setup
import os
from glob import glob

package_name = 'motion_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayushgaggar',
    maintainer_email='agaggar@u.northwestern.edu',
    description='will allow the user to input final end effector position \
    and orientation, then plan the trajectory; the user can choose to inspect \
    the scene after trajectory planning, and then execute the trajectory. \
    Additionally, user can dynamically add box object to the scene at user-defined position.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={

        'console_scripts': ['intercept_node = motion_planning.intercept_node:main',
                            # 'plan_node = motion_planning.plan_node:main',
                            # 'part1_plan_node = motion_planning.planning_for_position:main',
                            # 'plan_orientation = motion_planning.plan_orientation:main',
                            # 'add_box = motion_planning.add_box:main',
                            # 'wait_before_exec_node = motion_planning.wait_before_execute:main',
                            # 'multiple_points_node = motion_planning.plan_multiple_points:main',
                            'simple_move = motion_planning.simple_move:main',
                            'arena_node = motion_planning.balloon_drop:main',
                            'balloon_move = motion_planning.balloon_move:main'
                            ],

    },
)
