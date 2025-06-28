from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'modern_swarm'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Configuration files
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml') + glob('config/*.rviz')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*.urdf.xacro')),
        # World files
        (os.path.join('share', package_name, 'worlds'), 
         glob('worlds/*.world')),
        # Service definitions
        (os.path.join('share', package_name, 'srv'), 
         glob('srv/*.srv')),
        # Python scripts
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'sensor_msgs',
        'std_msgs',
        'nav_msgs',
        'tf2_ros',
        'cv_bridge',
        'numpy',
        'opencv-python',
        'matplotlib',
    ],
    zip_safe=True,
    maintainer='Modern Swarm Team',
    maintainer_email='user@example.com',
    description='Modern ROS2-based swarm robotics with AI-enhanced leader-follower navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_swarm_ros2 = unified_swarm_ros2:main',
            'test_ros2_integration = test_ros2_integration:main',
            'follower_controller_node = follower_controller_node:main',
            'vision_node = vision_node:main',
        ],
    },
) 