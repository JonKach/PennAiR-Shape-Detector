# PennAiR-Shape-Detector
My solution to the 2025 Penn AiR Technical Challenge @ The University of Pennsylvania

Normal Video:

[![Watch the video](https://drive.google.com/thumbnail?id=1Wq8kXJFXGGS9ta9b8pQ3arzIlol385sa)](https://drive.google.com/file/d/1Wq8kXJFXGGS9ta9b8pQ3arzIlol385sa/view?usp=drive_link)

Background-Agnostic Video:

[![Watch the video](https://drive.google.com/thumbnail?id=1ZOV96D86TyChuhMY4ZsTb-QSBo6OWIHw)](https://drive.google.com/file/d/1ZOV96D86TyChuhMY4ZsTb-QSBo6OWIHw/view?usp=drive_link)

Evaluations Instructions (Just Image Algorithm):

Find all the source code for the shape detection algorithm in src/shape_detection. The two main files are IMG_SOLUTION.py and VIDEO_SOLUTION.py. The pre-recorded videos and reports for evaluation are also in the "Results" folder in src/shape_detection.

Evaluation Instructions (ROS2):

The root folder of this project is the ROS2 Workspace (colcon build should be run from the root). You can find all the associated ROS2 files (nodes, launch files, etc.) in the src/shape_detection folder (nodes are in src/shape_detection/shape_detection). After building the ROS2 workspace, run the project with the following command:
- ros2 launch shape_detection detect_shapes.launch.py
You can then view the output (detection JSONs) by running "ros2 topic echo /contours_json" from your ros2 environment.
