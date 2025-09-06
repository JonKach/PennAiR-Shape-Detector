# PennAiR-Shape-Detector
My solution to the 2025 Penn AiR Technical Challenge @ The University of Pennsylvania

EVALUATION INSTRUCTIONS (Just Image Algorithm)
Find all the source code for the shape detection algorithm in src/shape_detection. The two main files are IMG_SOLUTION.py and VIDEO_SOLUTION.py. The pre-recorded videos and reports for evaluation are in the "Results" folder in the src/shape_detection.

EVALUATION INSTRUCTIONS (ROS2):
The root folder of this project is the ROS2 Workspace. You can find all the associated ROS2 files (nodes, launch files, etc.) in the src/shape_detection folder. From the ROS2 directory, run:
- colcon build
- source the correct ROS version for your shell (ex. source install/local_setup.zsh)
- ros2 launch shape_detection detect_shapes.launch.py
You can then view the output (detection JSONs) by running "ros2 topic echo \contours_json" from your ros2 environment.
