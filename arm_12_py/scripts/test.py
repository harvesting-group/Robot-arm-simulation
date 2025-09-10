#!/usr/bin/python3

from __future__ import print_function
import rospy
import sys
import copy
from math import pi
import threading

import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from termcolor import cprint
from moveit_commander import (
    RobotCommander,
    PlanningSceneInterface,
    MoveGroupCommander,
    roscpp_initialize,
    roscpp_shutdown
)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("fake_joint_publisher", anonymous=True)

robot = RobotCommander()
scene = PlanningSceneInterface()
move_group = MoveGroupCommander("arm_main")
move_group.set_max_velocity_scaling_factor(0.1)
move_group.set_max_acceleration_scaling_factor(0.1)

def print_joint_states(label):
    joint_names = move_group.get_active_joints()
    joint_values = move_group.get_current_joint_values()
    print(f"============ {label} Joint Values:")
    for name, val in zip(joint_names, joint_values):
        print(f"{name}: {val:.4f} rad")

def extend_trajectory_with_mirrored_joints(plan):
    """在 plan 的基础上添加 j1 = j2, j11 = -j3"""
    traj = plan.joint_trajectory
    if 'j1' in traj.joint_names or 'j11' in traj.joint_names:
        return plan  # 已经有了就不要重复添加

    name_idx = {name: i for i, name in enumerate(traj.joint_names)}
    new_joint_names = traj.joint_names + ['j1', 'j11']
    new_points = []

    for pt in traj.points:
        j2 = pt.positions[name_idx['j2']] if 'j2' in name_idx else 0.0
        j3 = pt.positions[name_idx['j3']] if 'j3' in name_idx else 0.0
        new_pt = copy.deepcopy(pt)
        new_pt.positions = tuple(list(pt.positions) + [j2, -j3])
        new_points.append(new_pt)

    # 修改原始轨迹（注意：是 plan.joint_trajectory）
    traj.joint_names = new_joint_names
    traj.points = new_points

    return plan

def main():
    print_joint_states("Before Motion")

    pose_goal = Pose()
    pose_goal.orientation.w = 0.0004874439225823042
    pose_goal.orientation.x = 0.0005562231614472014
    pose_goal.orientation.y = 0.7070982459279406
    pose_goal.orientation.z = -0.7071149295693359
    pose_goal.position.x = -0.2873599318214804
    pose_goal.position.y = 0.075887694941611713
    pose_goal.position.z = 0.45726937053768507
    move_group.set_pose_target(pose_goal)

    plan_result = move_group.plan()
    if plan_result and plan_result[0]:
        cprint("Planning succeeded. Executing trajectory...", color="green")
        plan = plan_result[1]

        # 扩展轨迹，添加 j1 = j2, j11 = -j3
        plan = extend_trajectory_with_mirrored_joints(plan)

        duration = plan.joint_trajectory.points[-1].time_from_start.to_sec()
        move_group.execute(plan, wait=True)
        rospy.sleep(duration)

        move_group.stop()
        move_group.clear_pose_targets()
        print_joint_states("After Motion")
    else:
        cprint("Planning failed.", color="red")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
