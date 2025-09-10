#!/usr/bin/python3
from __future__ import print_function
import rospy
import sys
import copy
from math import pi
import numpy as np
from geometry_msgs.msg import PoseStamped
import moveit_commander
from geometry_msgs.msg import Pose
from termcolor import cprint
from moveit_commander import (
    RobotCommander,
    PlanningSceneInterface,
    MoveGroupCommander,
    roscpp_initialize,
    roscpp_shutdown
)
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time
from tf.transformations import (
    quaternion_from_euler,
    quaternion_multiply,
    quaternion_matrix,
    quaternion_from_matrix
)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

# 初始化
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("shortest_path_planner", anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()
move_group = MoveGroupCommander("arm_main")
move_group.set_max_velocity_scaling_factor(0.1)
move_group.set_max_acceleration_scaling_factor(0.1)
move_group.set_planning_pipeline_id("ompl")
move_group.set_planner_id("RRTConnectkConfigDefault")
move_group.set_num_planning_attempts(10)
move_group.set_planning_time(10.0)
move_group.allow_replanning(True)

# 添加障碍物示例
def publish_colored_mesh_marker(name, pose_stamped, mesh_path, scale=(1,1,1), rgba=(1,0,0,1)):
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    rospy.sleep(0.5)
    marker = Marker()
    marker.header = pose_stamped.header
    marker.ns = name
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.mesh_resource = "file://" + mesh_path
    marker.pose = pose_stamped.pose
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
    marker.lifetime = rospy.Duration(0)
    marker_pub.publish(marker)

# 打印关节状态
def print_joint_states(label):
    names = move_group.get_active_joints()
    vals = move_group.get_current_joint_values()
    print(f"==== {label} Joint States:")
    for n, v in zip(names, vals): print(f"{n}: {v:.4f}")

# 计算轨迹长度
def compute_joint_path_length(traj):
    length = 0.0
    pts = traj.points
    for i in range(1, len(pts)):
        p1 = np.array(pts[i-1].positions)
        p2 = np.array(pts[i].positions)
        length += np.linalg.norm(p2 - p1)
    return length

# 关节空间最短路径（只考虑位置）
def joint_space_shortest_path(target_pose):
    # 清除所有旧目标
    move_group.clear_pose_targets()
    move_group.clear_path_constraints()
    # 仅设置末端位置目标
    move_group.set_position_target(
        [
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        ]
    )
    best_plan = None
    best_len = float('inf')
    for i in range(5):
        ok, plan = move_group.plan()
        if ok:
            length = compute_joint_path_length(plan.joint_trajectory)
            rospy.loginfo(f"[Joint] Trial {i} len={length:.4f}")
            if length < best_len:
                best_len, best_plan = length, plan
    move_group.clear_pose_targets()
    return best_plan, best_len

# 主程序
if __name__ == '__main__':
    try:
        print_joint_states("Before")
        # 构造目标位置PoseStamped（只用位置）
        goal = Pose()
        goal.position.x = -0.05
        goal.position.y = 0.2
        goal.position.z = 0.63726937

        # 添加可视化Frame
        broadcaster = StaticTransformBroadcaster()
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = robot.get_planning_frame()
        tf.child_frame_id = "pose_goal_frame"
        tf.transform.translation.x = goal.position.x
        tf.transform.translation.y = goal.position.y
        tf.transform.translation.z = goal.position.z
        tf.transform.rotation.w = 1.0
        broadcaster.sendTransform(tf)

        # 规划并执行
        plan, length = joint_space_shortest_path(goal)
        if plan:
            cprint(f"Executing path, len={length:.4f}", "green")
            move_group.execute(plan, wait=True)
            move_group.stop()
            rospy.sleep(plan.joint_trajectory.points[-1].time_from_start.to_sec())
        else:
            cprint("No valid plan found.", "red")
        print_joint_states("After")
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
