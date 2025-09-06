import json
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Header

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose, ObjectHypothesis
from geometry_msgs.msg import Pose2D

class ShapeDetector(Node):
    def __init__(self):
        super().__init__('shape_detector')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, 'image_raw', self.on_image, qos)
        # self.det_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.debug_pub = self.create_publisher(Image, 'debug_image', 10)
        self.json_pub = self.create_publisher(String, 'contours_json', 10)

        self.get_logger().info('Subscribed to /image_raw; publishing /detections, /debug_image, /contours_json')

    def on_image(self, msg: Image):
        # Convert ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Build messages
        header = Header()
        header.stamp = msg.header.stamp  # keep original timestamp
        header.frame_id = msg.header.frame_id if msg.header.frame_id else 'Frame'

        json_out = {
            "stamp": {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec},
            "frame_id": header.frame_id,
            "contours": []
        }

        # ---------- DETECTION LOGIC (replace with your detector if you have one) ----------
        img = frame
        copy = img.copy()
        copy = cv2.cvtColor(copy, cv2.COLOR_RGB2GRAY, None)

        med = np.median(copy)
        sigma = 0.33
        lower = int(max(0, (1.0-sigma)*med))
        upper = int(min(255, (1.0+sigma)*med))

        edges = cv2.Canny(copy, lower, upper)

        sobelx = cv2.Sobel(edges, cv2.CV_64F, 1, 0, ksize=3)  # Horizontal edges
        sobely = cv2.Sobel(edges, cv2.CV_64F, 0, 1, ksize=3)  # Vertical edges\\

        
        # Compute gradient magnitude
        gradient_magnitude = cv2.magnitude(sobelx, sobely)
        
        # Convert to uint8
        gradient_magnitude = cv2.convertScaleAbs(gradient_magnitude)

        blur = cv2.GaussianBlur(gradient_magnitude, (17,17), 0)

        dilate = cv2.dilate(blur, (3,3 ), iterations=1)

        #thresh once to eliminate much of the grass texture
        retval, threshed = cv2.threshold(dilate, 10, 255, cv2.THRESH_BINARY_INV)

        # Create a kernel for morphological ops
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # # First close small black lines/gaps
        # closed = cv2.morphologyEx(threshed, cv2.MORPH_CLOSE, kernel)

        contours, hierarchy = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print(len(contours))

        selected_contours = []
        for contour in contours:
            if(cv2.contourArea(contour) > 5000):
                selected_contours.append(contour)

        #we've found the individual contours, now time to merge

        #create a black image
        black_img = np.zeros_like(img)
        black_img = cv2.cvtColor(black_img, cv2.COLOR_BGR2GRAY)

        #draw contours onto black image
        cv2.drawContours(black_img, selected_contours, -1, (255, 255, 255), cv2.FILLED)

        blur_2 = cv2.GaussianBlur(black_img, (31, 31), 40)

        ret, binary = cv2.threshold(blur_2, 40, 255, cv2.THRESH_BINARY)

        joined_contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #get rid of boundaries
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # closed = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        cv2.drawContours(img, joined_contours, -1, (57.12,255,19.89), 2)

        camera_matrix = [[2564.3186869,0,0],
                    [0,2569.70273111,0],
                    [0, 0, 1]]
        CIRCLE_RADIUS_IN_PX = 92
        CIRCLE_RADIUS_CM = 10

        id = 0
        for contour in joined_contours:
            moments = cv2.moments(contour) #returns a dictionary of moments
            if moments['m00'] != 0:
                Cx = int(moments['m10'] / moments['m00'])
                Cy = int(moments['m01'] / moments['m00'])
                cv2.circle(img, (Cx, Cy), 4, (255, 255, 255), -1)
                cv2.putText(img, "Center", (Cx-20, Cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

                depth = camera_matrix[0][0] * CIRCLE_RADIUS_CM / CIRCLE_RADIUS_IN_PX
                x = (Cx - camera_matrix[0][2]) * depth / camera_matrix[0][0]
                y = (Cy - camera_matrix[1][2]) * depth / camera_matrix[1][1]

                cv2.putText(img, f"Coords: [{int(x)}, {int(y)}, {int(depth)}]", (Cx-20, Cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

                contour_points = contour.squeeze(1).tolist()
                json_out["contours"].append({
                    "id": id,
                    "center_pos": [int(x), int(y), int(depth)],
                    "contour_points": contour_points
                })
        # ----------------------------------------------------------------------------------

        # Publish
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))
        self.json_pub.publish(String(data=json.dumps(json_out)))
        self.get_logger().info("Published a contour!") 


def main():
    rclpy.init()
    node = ShapeDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
