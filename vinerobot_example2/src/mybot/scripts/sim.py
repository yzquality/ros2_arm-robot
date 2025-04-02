#!/usr/bin/env python3
import rclpy
import time
from moveit_commander import MoveGroupCommander

def cartesian_to_joint_positions(target_position):
    """Convert Cartesian coordinates to joint positions using MoveIt."""
    move_group = MoveGroupCommander("RRP_Robot")  # Replace with your MoveIt group name
    move_group.set_position_target(target_position)  # Set the Cartesian target point

    plan = move_group.plan()  # Plan the motion
    if plan:
        joint_positions = plan.joint_trajectory.points[-1].positions  # Get joint values
        return joint_positions
    else:
        raise ValueError("Inverse kinematics solution not found")

def send_trajectory(goal_points):
    """Send joint trajectory to the robot."""
    import rclpy.action
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    # Initialize node
    rclpy.init()
    node = rclpy.create_node("rrp_motion_controller")

    # Create action client
    action_client = rclpy.action.ActionClient(node, FollowJointTrajectory, "/my_group_controller/follow_joint_trajectory")
    action_client.wait_for_server()

    # Create trajectory message
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = ["joint1", "joint2", "joint3"]  # Replace with your joint names

    # Add points to the trajectory
    for i, point in enumerate(goal_points):
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = point
        trajectory_point.time_from_start = Duration(sec=i * 5, nanosec=0)
        goal_msg.trajectory.points.append(trajectory_point)

    # Send goal and wait for result
    start_time = time.time()
    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    end_time = time.time()
    print(f"Total time taken: {end_time - start_time:.2f} seconds")
    rclpy.shutdown()

def main():
    """Main function to execute the trajectory."""
    target_cartesian_points = [
        [0.5, 0.5, 1.7],  # Example Cartesian positions
        [0.5, -0.5, 1.7],
        [0.8, 0, 1.7],
    ]

    # Convert Cartesian points to joint positions
    joint_positions = []
    for point in target_cartesian_points:
        positions = cartesian_to_joint_positions(point)
        joint_positions.append(positions)

    # Send the trajectory
    send_trajectory(joint_positions)

if __name__ == "__main__":
    main()

