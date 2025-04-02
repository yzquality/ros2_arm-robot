import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthObstacleDetector(Node):
    def __init__(self):
        super().__init__('depth_obstacle_detector')

        # 订阅 Kinect 深度图像话题
        self.subscription = self.create_subscription(
            Image,
            '/kinect_v2/depth/kinect_v2_depth_sensor/image_raw',
            self.depth_callback,
            10)
        
        self.bridge = CvBridge()
        self.get_logger().info("DepthObstacleDetector node started!")

    def depth_callback(self, msg):
        # 将 ROS 图像转换为 OpenCV 格式
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # 处理 NaN 数据
        depth_image = np.nan_to_num(depth_image, nan=np.inf)

        # 计算最近障碍物的距离
        min_distance = np.min(depth_image)

        # 判断是否有障碍物
        if min_distance < 5.0:
            self.get_logger().info(f"🚨 障碍物检测：最近物体 {min_distance:.2f} 米（小于 5 米）")
        else:
            self.get_logger().info("✅ 视野内没有障碍物")

def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
