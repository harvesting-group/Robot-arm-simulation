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
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf2_ros
from geometry_msgs.msg import TransformStamped
import time
from tf.transformations import quaternion_from_euler, quaternion_multiply
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from scipy.spatial.transform import Rotation as R
import numpy as np
from visualization_msgs.msg import Marker


def rotation_matrix(axis: str, angle_deg: float) -> np.ndarray:
    """
    返回绕指定轴旋转 angle_deg 度的 3×3 旋转矩阵。

    参数
    ----
    axis : str
        旋转轴，'x' 或 'y' 或 'z'（大小写不限）。
    angle_deg : float
        旋转角度，单位为度。

    返回
    ----
    R : (3,3) ndarray
        对应的旋转矩阵。
    """
    θ = np.deg2rad(angle_deg)
    c, s = np.cos(θ), np.sin(θ)
    ax = axis.lower()
    if ax == 'x':
        R = np.array([
            [1, 0,  0],
            [0, c, -s],
            [0, s,  c]
        ])
    elif ax == 'y':
        R = np.array([
            [ c, 0, s],
            [ 0, 1, 0],
            [-s, 0, c]
        ])
    elif ax == 'z':
        R = np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1]
        ])
    else:
        raise ValueError("axis must be 'x', 'y', or 'z'")
    return R

# ========================================================================== #


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("shortest_path_planner", anonymous=True)

tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)


robot = RobotCommander()
scene = PlanningSceneInterface()
move_group = MoveGroupCommander("arm_main")
move_group.set_max_velocity_scaling_factor(0.1)
move_group.set_max_acceleration_scaling_factor(0.1)


# 强制使用 OMPL 随机采样
move_group.set_planning_pipeline_id("ompl")
move_group.set_planner_id("RRTConnectkConfigDefault")

# 给够时间和尝试次数
move_group.set_num_planning_attempts(10)
move_group.set_planning_time(10.0)   # 最长等 10s
move_group.allow_replanning(True)
move_group.set_start_state_to_current_state()


# 定义角度（单位：度）
rot_y = 180
rot_x = 20

# 创建复合旋转：先绕y转180，再绕新的x转20
r = R.from_euler('yx', [rot_y, rot_x], degrees=True)

# 转为四元数 (x, y, z, w)
quat_leaf_xyzw = r.as_quat()

leaf = PoseStamped()
leaf.header.frame_id = robot.get_planning_frame()   
leaf.pose.position.x = 0.2
leaf.pose.position.y = 0.16
leaf.pose.position.z = 0.85
leaf.pose.orientation.x = quat_leaf_xyzw[0]
leaf.pose.orientation.y = quat_leaf_xyzw[1]
leaf.pose.orientation.z = quat_leaf_xyzw[2]
leaf.pose.orientation.w = quat_leaf_xyzw[3]
leaf_mesh = "/home/jiziheng/Music/xianyu/gabriele13_moveit/moveit_ws/src/arm12/meshes/leaf.stl"
# box_size = (0.05, 0.002, 0.1)
scene.add_mesh("leaf", leaf, leaf_mesh, size=(0.2,0.2,0.2))
time.sleep(1.0)







try:
    trans = tf_buffer.lookup_transform("rotated_base", "straw3", rospy.Time(0), rospy.Duration(1.0))
except Exception as e:
    rospy.logerr("Failed to get transform: %s", str(e))



# strawberry = PoseStamped()
# strawberry.header.frame_id = robot.get_planning_frame()
# strawberry.header.stamp    = rospy.Time.now()

# strawberry.pose.position.x = -0
# strawberry.pose.position.y = 0.2
# strawberry.pose.position.z = 0.63726937053768507
# strawberry.pose.orientation.w = 0.7071067811865476
# strawberry.pose.orientation.x = -0.7071067811865476

# mesh_res = "/home/jiziheng/Music/xianyu/gabriele13_moveit/moveit_ws/src/arm12/meshes/st.stl"
# scene.add_mesh("strawberry", strawberry, mesh_res, size=(0.02,0.02,0.02))
# rospy.sleep(1.0)

strawberry = PoseStamped()
strawberry.header.frame_id = "base_link"
strawberry.pose.position = trans.transform.translation
strawberry.pose.orientation = trans.transform.rotation


q_orig = [
    strawberry.pose.orientation.x,
    strawberry.pose.orientation.y,
    strawberry.pose.orientation.z,
    strawberry.pose.orientation.w
]
roll_corr  = np.deg2rad(-90)   # 绕 X 轴
pitch_corr = 0   # 绕 Y 轴
yaw_corr   = 0                 # 绕 Z 轴
q_corr = quaternion_from_euler(roll_corr, pitch_corr, yaw_corr)
q_new = quaternion_multiply(q_corr, q_orig)

# 4) 写回 pose
strawberry.pose.orientation.x = q_new[0]
strawberry.pose.orientation.y = q_new[1]
strawberry.pose.orientation.z = q_new[2]
strawberry.pose.orientation.w = q_new[3]




def print_joint_states(label):
    joint_names = move_group.get_active_joints()
    joint_values = move_group.get_current_joint_values()
    print(f"============ {label} Joint Values:")
    for name, val in zip(joint_names, joint_values):
        print(f"{name}: {val:.4f} rad")

def extend_trajectory_with_mirrored_joints(plan):
    traj = plan.joint_trajectory
    if 'j1' in traj.joint_names or 'j11' in traj.joint_names:
        return plan

    name_idx = {name: i for i, name in enumerate(traj.joint_names)}

    # 如果 j4 已经存在，直接复制 j3 的值
    if 'j3' in name_idx and 'j4' in name_idx:
        for pt in traj.points:
            j3_val = pt.positions[name_idx['j3']]
            pt.positions = list(pt.positions)
            pt.positions[name_idx['j4']] = j3_val
            pt.positions = tuple(pt.positions)



    new_joint_names = traj.joint_names + ['j1', 'j11']
    new_points = []

    for i,pt in enumerate(traj.points):
        j2 = pt.positions[name_idx['j2']] if 'j2' in name_idx else 0.0
        j3 = pt.positions[name_idx['j3']] if 'j3' in name_idx else 0.0
        new_pt = copy.deepcopy(pt)
        if i == 0:
            j1 = 0
        else:
            j1 = 0.6119 - 0.85 * np.cos(0.83798 + (-j3)) + j2 - 0.07
            # j1 = 0.6119 - 0.85 * np.cos(0.83798 + (-j3)) + j2 - 0.05
        new_pt.positions = tuple(list(pt.positions) + [j1, -j3])
        new_points.append(new_pt)

    traj.joint_names = new_joint_names
    traj.points = new_points

    return plan

def compute_joint_path_length(traj):
    length = 0.0
    for i in range(1, len(traj.points)):
        p1 = np.array(traj.points[i-1].positions)
        p2 = np.array(traj.points[i].positions)
        length += np.linalg.norm(p2 - p1)
    return length

def joint_space_shortest_path(pose_goal, res):

    move_group.set_pose_target(pose_goal)


    best_plan = None
    best_length = float("inf")

    for i in range(5):  # 多次采样
        plan_result = move_group.plan()
        if plan_result and plan_result[0]:
            plan = plan_result[1]
            length = compute_joint_path_length(plan.joint_trajectory)
            rospy.loginfo(f"[Joint-Space] Trial {i}: Length = {length:.4f}")
            if length < best_length:
                best_length = length
                best_plan = plan

    move_group.clear_pose_targets()
    return best_plan, best_length

def cartesian_path(pose_goal):
    waypoints = []
    waypoints.append(pose_goal)

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, eef_step=0.01, avoid_collisions=True
    )

    length = compute_joint_path_length(plan.joint_trajectory)
    rospy.loginfo(f"[Cartesian] Success fraction = {fraction:.3f}, Length = {length:.4f}")

    return plan, fraction, length

from tf.transformations import quaternion_from_euler


def main(mode="cartesian"):  # mode: "joint" or "cartesian"
    print_joint_states("Before Motion")

    pose_goal = Pose()
  
    # 固定目标
    # # pose_goal.orientation.x = qx
    # # pose_goal.orientation.y = qy
    # # pose_goal.orientation.z = qz
    # # pose_goal.orientation.w = qw
    # pose_goal.orientation.w = 0.0004874439225823042
    # pose_goal.orientation.x = 0.0005562231614472014
    # pose_goal.orientation.y = 0.7070982459279406
    # pose_goal.orientation.z = -0.7071149295693359
    # pose_goal.position.x = 0.2873599318214804
    # pose_goal.position.y = 0.22
    # # pose_goal.position.y = 0.0
    # pose_goal.position.z = 0.53726937053768507


    # 草莓作为目标
    T_orig = quaternion_matrix([
        strawberry.pose.orientation.x,
        strawberry.pose.orientation.y,
        strawberry.pose.orientation.z,
        strawberry.pose.orientation.w
    ])
    T_orig[:3,3] = [strawberry.pose.position.x,
                strawberry.pose.position.y,
                strawberry.pose.position.z]
    T_rot = np.eye(4)
    T_rot[:3,:3] = np.array([[-1,0,0],[0,0,-1],[0,-1,0]])
    
    T_rot[:3,3]=np.array([-0.01,-0.08,0.05])
    # T_rot[:3,3]=np.array([-0.1,-0.1,-0])


    # T_rot[:3,:3] = np.array([[-1,0,0],[0,0,1],[0,1,0]])
    # T_rot[:3,3]=np.array([0,0,0.02])
    T_new = T_orig.dot(T_rot)

    T_2 = np.eye(4)
    # T_2[:3,:3] = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
    T_2[:3,:3] = rotation_matrix("y",-45)
    T_2[:3,3] = np.array([0,0,0.0])
    T_new = T_new.dot(T_2)

    new_q = quaternion_from_matrix(T_new)   # [qx, qy, qz, qw]
    new_t = T_new[0:3, 3]                   # [x, y, z]
    # print(T_orig)
    pose_goal.position.x    = new_t[0]
    pose_goal.position.y    = new_t[1]
    pose_goal.position.z    = new_t[2]
    pose_goal.orientation.x = new_q[0]
    pose_goal.orientation.y = new_q[1]
    pose_goal.orientation.z = new_q[2]
    pose_goal.orientation.w = new_q[3]

    static_broadcaster = StaticTransformBroadcaster()

    static_tf = TransformStamped()
    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id    = robot.get_planning_frame()   # e.g. "base_link"
    static_tf.child_frame_id     = "pose_goal_frame"
    static_tf.transform.translation.x = pose_goal.position.x
    static_tf.transform.translation.y = pose_goal.position.y
    static_tf.transform.translation.z = pose_goal.position.z
    static_tf.transform.rotation      = pose_goal.orientation
    static_broadcaster.sendTransform(static_tf)
    print(f"pose_goal:{pose_goal}")



    # leaf_tf_broadcaster = StaticTransformBroadcaster()
    # leaf_tf = TransformStamped()
    # leaf_tf.header.stamp = rospy.Time.now()
    # leaf_tf.header.frame_id    = robot.get_planning_frame()   # e.g. "base_link"
    # leaf_tf.child_frame_id     = "leaf_tf"
    # leaf_tf.transform.translation.x = leaf.pose.position.x
    # leaf_tf.transform.translation.y = leaf.pose.position.y
    # leaf_tf.transform.translation.z = leaf.pose.position.z
    # leaf_tf.transform.rotation.x = leaf.pose.orientation.x
    # leaf_tf.transform.rotation.y = leaf.pose.orientation.y
    # leaf_tf.transform.rotation.z = leaf.pose.orientation.z
    # leaf_tf.transform.rotation.w = leaf.pose.orientation.w
    # leaf_tf_broadcaster.sendTransform(leaf_tf)







    from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
    rospy.wait_for_service('/compute_ik')
    ps = PoseStamped()
    ps.header.frame_id = move_group.get_planning_frame()  # e.g. "base_link"
    ps.header.stamp    = rospy.Time.now()
    ps.pose = pose_goal   # 如果 pose_goal 已经是 PoseStamped，就直接用 pose_goal

    # 2) 调用 IK 服务
    rospy.wait_for_service('/compute_ik')
    ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    req    = GetPositionIKRequest()
    req.ik_request.group_name    = move_group.get_name()
    req.ik_request.pose_stamped  = ps
    req.ik_request.robot_state   = robot.get_current_state()
    req.ik_request.timeout       = rospy.Duration(0.5)

    res = ik_srv(req)
    if res.error_code.val != res.error_code.SUCCESS:
        rospy.logerr("IK 无解: %d", res.error_code.val)
    else:
        rospy.loginfo("IK 解算成功: %s", res.solution.joint_state.position)



    if mode == "joint":
        plan, best_len = joint_space_shortest_path(pose_goal, res)
        if plan:
            cprint(f"[Joint-Space] Best length: {best_len:.4f}. Executing...", "green")
            plan = extend_trajectory_with_mirrored_joints(plan)
            duration = plan.joint_trajectory.points[-1].time_from_start.to_sec()
            move_group.execute(plan, wait=True)
            rospy.sleep(duration)
    elif mode == "cartesian":
        plan, fraction, length = cartesian_path(pose_goal)
        if fraction > 0.2:
            cprint(f"[Cartesian] Planning success. Length: {length:.4f}. Executing...", "green")
            plan = extend_trajectory_with_mirrored_joints(plan)
            duration = plan.joint_trajectory.points[-1].time_from_start.to_sec()
            move_group.execute(plan, wait=True)
            rospy.sleep(duration)
        else:
            cprint("[Cartesian] Planning failed (low fraction).", "red")
    else:
        raise ValueError(f"Unknown planning mode: {mode}")

    move_group.stop()
    print_joint_states("After Motion")
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     broadcast_strawberry_frame(strawberry)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        main("joint")       # 使用关节空间最短路径
        # main("cartesian")     # 使用笛卡尔空间最短路径
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
