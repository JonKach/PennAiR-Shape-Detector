import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.declare_parameter('source', 'detect_resources/dynamic.mp4')          
        self.declare_parameter('frame_rate', 5.0)       
        self.declare_parameter('loop', True)            

        source_param = self.get_parameter('source').get_parameter_value().string_value
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.loop = bool(self.get_parameter('loop').value)
        
        if not os.path.exists(source_param):
            raise FileNotFoundError(f'Video file not found: {source_param}')
        self.cap = cv2.VideoCapture(source_param)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'image_raw', 10)
        period = 1.0 / self.frame_rate
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(f'Publishing /image_raw from source={source_param} at {self.frame_rate:.1f} Hz')

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
                if not ok:
                    self.get_logger().warning('Looping failed to read frame')
                    return
            else:
                self.get_logger().warning('No frame; stopping timer')
                self.timer.cancel()
                return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'Frame'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
