#!/usr/bin/python3


from __future__ import print_function
from six.moves import input
import rospy
from sensor_msgs.msg import JointState
import time
from termcolor import cprint
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from termcolor import cprint
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
from geometry_msgs.msg import Pose
import threading
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_commander import (
    RobotCommander,
    PlanningSceneInterface,
    MoveGroupCommander,
    roscpp_initialize,
    roscpp_shutdown
)
moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node("moveit_arm12", anonymous=True)
rospy.init_node("fake_joint_publisher",anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

robot = RobotCommander()
scene = PlanningSceneInterface()

# 主右臂控制组（负责规划）
move_group = MoveGroupCommander("arm_main")
move_group.set_max_velocity_scaling_factor(0.1)
move_group.set_max_acceleration_scaling_factor(0.1)

# 左臂控制组（镜像控制 j1, j11）
left_group = MoveGroupCommander("left")
left_group.set_max_velocity_scaling_factor(0.1)
left_group.set_max_acceleration_scaling_factor(0.1)
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)




def print_joint_states(label):
    joint_names_main = move_group.get_active_joints()
    joint_values_main = move_group.get_current_joint_values()
    joint_names_left = left_group.get_active_joints()
    joint_values_left = left_group.get_current_joint_values()

    print(f"============ {label} Joint Values:")
    for name, val in zip(joint_names_main, joint_values_main):
        print(f"{name}: {val:.4f} rad")
    print(f"============ {label} Left Joint Values:")
    for name, val in zip(joint_names_left, joint_values_left):
        print(f"{name}: {val:.4f} rad")



def sync_left_arm_from_traj(main_traj):
    # 从 arm_main 的 joint trajectory 中提取 j2, j3 → j1, j11
    left_traj = moveit_msgs.msg.RobotTrajectory()
    left_traj.joint_trajectory.joint_names = ['j1', 'j11']

    for point in main_traj.joint_trajectory.points:
        name_idx = {name: i for i, name in enumerate(main_traj.joint_trajectory.joint_names)}
        p = point.positions

        j2_val = p[name_idx['j2']] if 'j2' in name_idx else 0.0
        j3_val = p[name_idx['j3']] if 'j3' in name_idx else 0.0

        new_point = copy.deepcopy(point)
        new_point.positions = [j2_val, -j3_val]
        left_traj.joint_trajectory.points.append(new_point)

    return left_traj




# def main():
#     print_joint_states("Before Motion")

#     # 设定目标位姿
#     pose_goal = Pose()
#     pose_goal.orientation.w = 0.0004874439225823042
#     pose_goal.orientation.x = 0.0005562231614472014
#     pose_goal.orientation.y = 0.7070982459279406
#     pose_goal.orientation.z = -0.7071149295693359
#     pose_goal.position.x = -0.2873599318214804
#     pose_goal.position.y = 0.075887694941611713
#     pose_goal.position.z = 0.45726937053768507
#     move_group.set_pose_target(pose_goal)

#     # 规划
#     plan = move_group.plan()
#     if plan and plan[0]:
#         cprint("Planning succeeded. Executing arm_main trajectory...", color="green")
#         traj = plan[1].joint_trajectory
#         duration = traj.points[-1].time_from_start.to_sec()

#         # # 启动同步线程
#         # stop_event = threading.Event()
#         # sync_thread = threading.Thread(target=sync_left_arm, args=(stop_event,))
#         # sync_thread.start()

#         # # 执行主臂轨迹
#         # move_group.execute(traj, wait=False)
#         # 生成 left 轨迹：j1 = j2, j11 = -j3
#         name_idx = {name: i for i, name in enumerate(traj.joint_names)}
#         left_traj = moveit_msgs.msg.RobotTrajectory()
#         left_traj.joint_trajectory.joint_names = ['j1', 'j11']
#         for pt in traj.points:
#             j2 = pt.positions[name_idx['j2']] if 'j2' in name_idx else 0.0
#             j3 = pt.positions[name_idx['j3']] if 'j3' in name_idx else 0.0
#             new_point = copy.deepcopy(pt)
#             new_point.positions = [j2, -j3]
#             left_traj.joint_trajectory.points.append(new_point)

#         # 执行两条轨迹
#         move_group.execute(plan[1], wait=False)
#         left_group.execute(left_traj, wait=False)

#         rospy.sleep(duration)

#         # 等待轨迹执行完成
#         rospy.sleep(duration)

#         # # 停止同步
#         # stop_event.set()
#         # sync_thread.join()

#         # 清理
#         move_group.stop()
#         move_group.clear_pose_targets()
#         left_group.stop()
#         left_group.clear_pose_targets()

#         print_joint_states("After Motion")
#     else:
#         cprint("Planning failed.", color="red")



def main():
    print_joint_states("Before Motion")

    # 设定目标位姿
    pose_goal = Pose()
    pose_goal.orientation.w = 0.0004874439225823042
    pose_goal.orientation.x = 0.0005562231614472014
    pose_goal.orientation.y = 0.7070982459279406
    pose_goal.orientation.z = -0.7071149295693359
    pose_goal.position.x = -0.2873599318214804
    pose_goal.position.y = 0.075887694941611713
    pose_goal.position.z = 0.45726937053768507
    move_group.set_pose_target(pose_goal)

    plan = move_group.plan()
    if plan and plan[0]:
        cprint("Planning succeeded. Executing arm_main trajectory...", color="green")
        traj = plan[1].joint_trajectory
        duration = traj.points[-1].time_from_start.to_sec()

        # 生成左臂轨迹：j1 = j2, j11 = -j3
        name_idx = {name: i for i, name in enumerate(traj.joint_names)}
        left_traj = moveit_msgs.msg.RobotTrajectory()
        left_traj.joint_trajectory.joint_names = ['j1', 'j11']
        for pt in traj.points:
            j2 = pt.positions[name_idx['j2']] if 'j2' in name_idx else 0.0
            j3 = pt.positions[name_idx['j3']] if 'j3' in name_idx else 0.0
            new_point = copy.deepcopy(pt)
            new_point.positions = [j2, -j3]
            left_traj.joint_trajectory.points.append(new_point)

        # 同步执行
        execute_both_arms(plan[1], left_traj)

        # 清理
        move_group.stop()
        move_group.clear_pose_targets()
        left_group.stop()
        left_group.clear_pose_targets()

        print_joint_states("After Motion")
    else:
        cprint("Planning failed.", color="red")


def execute_both_arms(main_traj, left_traj):
    done_event = threading.Event()

    def run_main():
        move_group.execute(main_traj, wait=True)
        done_event.set()

    def run_left():
        left_group.execute(left_traj, wait=True)
        done_event.set()

    # 启动两个线程
    t1 = threading.Thread(target=run_main)
    t2 = threading.Thread(target=run_left)
    t1.start()
    t2.start()

    # 等待两个执行完
    t1.join()
    t2.join()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()