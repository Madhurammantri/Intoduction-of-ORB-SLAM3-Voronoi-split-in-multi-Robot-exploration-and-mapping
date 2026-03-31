entry_points={
    'console_scripts': [
        'pose_logger = orbslam_analysis_tools.pose_logger:main',
    ],
},
from setuptools import setup

package_name = 'orbslam_analysis_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='madhuram',
    maintainer_email='madhuram@todo.todo',
    description='Tools to log and analyze ORB-SLAM vs Nav2 poses',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_logger = orbslam_analysis_tools.pose_logger:main',
	    'send_fixed_goal = orbslam_analysis_tools.send_fixed_goal:main',
	    'auto_explore = orbslam_analysis_tools.auto_explore:main',
	    'map_latcher = orbslam_analysis_tools.map_latcher:main',
	    'auto_explore_merged = orbslam_analysis_tools.auto_explore_merged:main',
	    'auto_explore_merged_idea_A = orbslam_analysis_tools.auto_explore_merged_idea_A:main',
	    'auto_explore_merged_idea_B = orbslam_analysis_tools.auto_explore_merged_idea_B:main',
	    'auto_explore_merged_my_idea = orbslam_analysis_tools.auto_explore_merged_my_idea:main',
        ],
    },
)

