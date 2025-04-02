import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class DepthColorDetector(Node):
    def __init__(self):
        super().__init__('depth_color_detector')
        # ✅ 确保 `self.arm_client` 在这里正确初始化
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/my_group_controller/follow_joint_trajectory')


        self.depth_subscription = self.create_subscription(
            Image, '/kinect_v2/depth/kinect_v2_depth_sensor/depth/image_raw', self.depth_callback, 10)
        
        self.rgb_subscription = self.create_subscription(
            Image, '/kinect_v2/rgb/kinect_v2_rgb_camera/image_raw', self.rgb_callback, 10)
        
        self.depth_camera_info_subscription = self.create_subscription(
            CameraInfo, '/kinect_v2/depth/kinect_v2_depth_sensor/camera_info', self.depth_camera_info_callback, 10)

        self.rgb_camera_info_subscription = self.create_subscription(
            CameraInfo, '/kinect_v2/rgb/kinect_v2_rgb_camera/camera_info', self.rgb_camera_info_callback, 10)

        self.bridge = CvBridge()
        self.depth_image = None
        self.rgb_image = None
        self.K_depth = None  
        self.K_rgb = None  

        self.get_logger().info("✅ DepthColorDetector 节点已启动!")

    def depth_camera_info_callback(self, msg):
        """ 读取深度相机内参矩阵 """
        self.K_depth = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f"📷 读取 Depth Camera Info: \n{self.K_depth}")

    def rgb_camera_info_callback(self, msg):
        """ 读取 RGB 相机内参矩阵 """
        self.K_rgb = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(f"📷 读取 RGB Camera Info: \n{self.K_rgb}")

    def depth_callback(self, msg):
        """ 处理深度图像数据 """
        try:
            depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            depth_image = np.nan_to_num(depth_image, nan=np.inf)
            depth_image[depth_image < 0.1] = np.inf  
            depth_image[depth_image > 10] = np.inf   
        except Exception as e:
            self.get_logger().error(f"❌ 深度图像处理失败: {e}")
            return

        self.depth_image = depth_image
        min_distance = np.min(depth_image)

        if min_distance == np.inf:
            self.get_logger().info("✅ 视野内没有物体，程序结束")
            rclpy.shutdown()
            return

        if self.rgb_image is None:
            self.get_logger().warning("⚠️ RGB 图像未准备好，跳过颜色检测")
            return  

        if 0.1 <= min_distance <= 1.65:
            self.get_logger().info(f"🚨 obstacle detection: nearest obj is in {min_distance:.2f} m")
            self.project_depth_to_rgb(min_distance)
        else:
            self.get_logger().info("✅ No abstacle")
            self.run_robot_arm()

    def rgb_callback(self, msg):
        """ 处理RGB彩色图像数据 """
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ RGB图像处理失败: {e}")

    def project_depth_to_rgb(self, object_distance):
        """ 用 3D 投影映射 `depth mask` 到 `RGB` """
        if self.rgb_image is None or self.K_depth is None or self.K_rgb is None:
            self.get_logger().warning("⚠️ 必要数据未就绪")
            return

        depth_h, depth_w = self.depth_image.shape
        rgb_h, rgb_w, _ = self.rgb_image.shape

        min_dist = object_distance
        max_dist = object_distance + 0.3
        mask = (self.depth_image >= min_dist) & (self.depth_image <= max_dist)

        if np.sum(mask) == 0:
            self.get_logger().warning("⚠️ 没有找到对应的RGB像素，跳过颜色检测")
            return

        fx, fy = self.K_depth[0, 0], self.K_depth[1, 1]
        cx, cy = self.K_depth[0, 2], self.K_depth[1, 2]

        x, y = np.meshgrid(np.arange(depth_w), np.arange(depth_h))
        x3d = (x - cx) * self.depth_image / fx
        y3d = (y - cy) * self.depth_image / fy
        z3d = self.depth_image.copy()

        rgb_fx, rgb_fy = self.K_rgb[0, 0], self.K_rgb[1, 1]
        rgb_cx, rgb_cy = self.K_rgb[0, 2], self.K_rgb[1, 2]
        x_rgb = (x3d * rgb_fx / z3d) + rgb_cx
        y_rgb = (y3d * rgb_fy / z3d) + rgb_cy

        x_rgb = np.clip(np.round(x_rgb).astype(int), 0, rgb_w - 1)
        y_rgb = np.clip(np.round(y_rgb).astype(int), 0, rgb_h - 1)

        rgb_mask = np.zeros((rgb_h, rgb_w), dtype=np.uint8)
        rgb_mask[y_rgb[mask], x_rgb[mask]] = 255

        selected_pixels = self.rgb_image[rgb_mask > 0]  # 选取 BGR 值

        if len(selected_pixels) > 0: 
            mean_bgr = np.mean(selected_pixels, axis=0).astype(np.uint8)
            mean_rgb = mean_bgr[::-1]  # BGR 转换为 RGB
            
            color_name = "灰色"  # 默认灰色
            if np.all(np.abs(mean_rgb - [122, 4, 3]) <= 20):  
                color_name = "红色"
            elif np.all(np.abs(mean_rgb - [97, 82, 65]) <= 20):  
                color_name = "棕色"

            self.get_logger().info(f"🎨 物体 RGB 颜色 (均值): {mean_rgb} -> 识别为 {color_name}")
        else:
            self.get_logger().warning("⚠️ 选中的区域中没有有效的像素")

    def run_robot_arm(self):
        """ 运行机械臂 """
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ 机械臂 Action 服务器未找到!")
            return

        target_positions_list = [
            [0.5236, -0.7854, 0],
            [-0.5236, -0.7854, 0],
            [0, 0, 0],
            [0, -0.7856, 0.1],
            [0, -0.7856, 0.5],
            [0.0, 0.0, 0.0],
        ]
        max_accelerations = [1.204, 2.374, 0.2]  # 关节最大加速度（rad/s²）
        max_speeds = [1.0, 1.0, 0.2]         # 关节最大速度（rad/s）

        current_positions = [0.0, 0.0, 0.0]  # 假设初始位置
        total_time = 0.0
        trajectory_points = []

        for i, target_positions in enumerate(target_positions_list):
            duration = self.calculate_trajectory_time_with_acceleration(
                current_positions, target_positions, max_accelerations, max_speeds
            )
            total_time += duration
            trajectory_points.append(
                JointTrajectoryPoint(
                    positions=target_positions,
                    time_from_start=rclpy.time.Duration(seconds=total_time).to_msg()
                )
            )
            self.get_logger().info(
                f"🛠 机械臂移动到 Point {i+1}: {target_positions}, 预计时间 {duration:.2f}s"
            )
            current_positions = target_positions  

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3']
        goal_msg.trajectory.points = trajectory_points

        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result().accepted:
            self.get_logger().info("✅ 机械臂运行成功!")
        else:
            self.get_logger().error("❌ 机械臂运行失败!")

    def calculate_trajectory_time_with_acceleration(self, current_positions, target_positions, max_accelerations, max_speeds):
        """ 计算从当前点到目标点的所需时间，考虑最大加速度和最大速度 """
        times = []
        for current, target, max_acc, max_speed in zip(current_positions, target_positions, max_accelerations, max_speeds):
            distance = abs(target - current)
            critical_distance = max_speed**2 / (2 * max_acc)

            if distance > 2 * critical_distance:
                t_accel = max_speed / max_acc
                t_cruise = (distance - 2 * critical_distance) / max_speed
                t_total = 2 * t_accel + t_cruise
            else:
                t_total = (2 * distance / max_acc) ** 0.5

            times.append(t_total)

        return max(times)

def main():
    rclpy.init()
    node = DepthColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

