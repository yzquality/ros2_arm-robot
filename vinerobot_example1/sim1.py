import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def calculate_trajectory_time(current_positions, target_positions, max_speeds):
    """计算从当前点到目标点的每个关节所需时间，返回最大时间。"""
    times = [
        abs(target - current) / max_speed
        for target, current, max_speed in zip(target_positions, current_positions, max_speeds)
    ]
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

    # 定义五个目标点和关节速度
    target_positions_list = [
        [0.5236, -0.7854, 0],
        [-0.5236, -0.7854, 0],
        [0, 0, 0],
        [0, -0.7856, 0.5],
        [0.0, 0.0, 0.0],
    ]
    max_speeds = [0.2, 0.2, 0.2]  # 每个关节的最大速度（rad/s）

    # 初始化变量
    current_positions = [0.0, 0.0, 0.0]  # 假设初始位置
    total_time = 0.0  # 总运行时间
    trajectory_points = []

    # 计算每个点的运行时间并构造轨迹点
    for i, target_positions in enumerate(target_positions_list):
        duration = calculate_trajectory_time(current_positions, target_positions, max_speeds)
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

