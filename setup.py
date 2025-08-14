import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'grasplan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Recursively include all files from launch/
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f]) 
           for f in glob('launch/**/*', recursive=True) if os.path.isfile(f)],
        # Recursively include all files from config/
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f])
           for f in glob('config/**/*', recursive=True) if os.path.isfile(f)],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oscar.Lima',
    maintainer_email='oscar.lima@dfki.de',
    description='Simple grasp planning for robots',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    scripts=[
            'scripts/insert_obj_test_action_client',
            'scripts/object_recognition_mockup.py',
            'scripts/pick_obj_test_action_client',
            'scripts/place_obj_test_action_client',
            'scripts/publish_tf_world_to_robot',
            'scripts/rqt_grasplan',
            'scripts/rqt_planning_scene',
            'scripts/set_rviz_logger_level.py',
            'scripts/visualize_planning_scene_node',
        ],
    entry_points={
        'console_scripts': [
            'pick = grasplan.pick:main',
	        'place = grasplan.place:main',
            'insert = grasplan.insert:main',
        ],
    },
)