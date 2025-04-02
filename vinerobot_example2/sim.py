import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def calculate_trajectory_time_with_acceleration(current_positions, target_positions, max_accelerations, max_speeds):
    """计算从当前点到目标点的所需时间，考虑最大加速度和最大速度"""
    times = []
    for current, target, max_acc, max_speed in zip(current_positions, target_positions, max_accelerations, max_speeds):
        distance = abs(target - current)
        # 匀加速运动所需的最大距离
        critical_distance = max_speed**2 / (2 * max_acc)

        if distance > 2 * critical_distance:
            # 有匀速阶段
            t_accel = max_speed / max_acc  # 加速时间
            t_cruise = (distance - 2 * critical_distance) / max_speed  # 匀速时间
            t_total = 2 * t_accel + t_cruise
        else:
            # 无匀速阶段，全程加速和减速
            t_total = (2 * distance / max_acc)**0.5

        times.append(t_total)
    # 返回最长时间作为同步时间
    return max(times)

def main():
    rclpy.init()
    node = rclpy.create_node('send_trajectory_goal')

    # 创建 Action 客户端
    action_client = ActionClient(node, FollowJointTrajectory, '/my_group_controller/follow_joint_trajectory')

    # 等待服务器
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Action server not available!")
        return

    # 定义五个目标点和关节的最大加速度与速度
    target_positions_list = [
        [0.5236, -0.7854, 0],
        [-0.5236, -0.7854, 0],
        [0, 0, 0],
        [0, -0.7856, 0.1],
        [0, -0.7856, 0.5],
        [0.0, 0.0, 0.0],
    ]
    max_accelerations = [1.204, 2.374, 0.2]  # 每个关节的最大加速度（rad/s²）
    max_speeds = [1.0, 1.0, 0.2]         # 每个关节的最大速度（rad/s）

    # 初始化变量
    current_positions = [0.0, 0.0, 0.0]  # 假设初始位置
    total_time = 0.0  # 总运行时间
    trajectory_points = []

    # 计算每个点的运行时间并构造轨迹点
    for i, target_positions in enumerate(target_positions_list):
        duration = calculate_trajectory_time_with_acceleration(
            current_positions, target_positions, max_accelerations, max_speeds
        )
        total_time += duration
        trajectory_points.append(
            JointTrajectoryPoint(
                positions=target_positions,
                time_from_start=rclpy.time.Duration(seconds=total_time).to_msg()
            )
        )
        node.get_logger().info(
            f"Point {i+1}: Target={target_positions}, Duration={duration:.2f}s, Total Time={total_time:.2f}s"
        )
        current_positions = target_positions  # 更新当前位置

    # 创建目标消息
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3']
    goal_msg.trajectory.points = trajectory_points

    # 发送目标
    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    # 检查结果
    if future.result().accepted:
        node.get_logger().info("Goal accepted!")
    else:
        node.get_logger().error("Goal rejected!")

    rclpy.shutdown()

if __name__ == '__main__':
    main()

