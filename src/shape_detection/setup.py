from setuptools import find_packages, setup

package_name = 'shape_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detect_shapes.launch.py']),
        ('share/' + package_name + '/detect_resources', ['detect_resources/dynamic_video.mp4']),
        ('share/' + package_name + '/detect_resources', ['detect_resources/normal_video.mp4']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='jonkach',
    maintainer_email='jonkach@seas.upenn.edu',
    description='Video Stream + Detect Shapes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_publisher = shape_detection.video_publisher:main',
            'shape_detector = shape_detection.shape_detector_node:main',
        ],
    },
)
