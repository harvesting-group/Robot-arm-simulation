#!/usr/bin/env python3

import rospy
import PyKDL
import kdl_parser_py.urdf as kdl_parser
from urdf_parser_py.urdf import URDF
from tf_conversions import posemath

def main():
    rospy.init_node("read_joint_pose")
    
    robot = URDF.from_parameter_server()
    ok, tree = kdl_parser.treeFromUrdfModel(robot)
    if not ok:
        print("Failed to parse URDF into KDL tree")
        return

    base_link = "base_link"
    joint_states = {
        "j2": 0.0,
        "j3": 0.0,
        "j4": 0.0
    }

    for joint_name in ["j2", "j3", "j4"]:
        # 每个 joint 的 child link
        joint_obj = robot.joint_map[joint_name]
        link_name = joint_obj.child

        chain = tree.getChain(base_link, link_name)
        fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

        joint_array = PyKDL.JntArray(chain.getNrOfJoints())
        # 填入 joint_states（注意：可能只用 j2/j3 的值影响）
        idx = 0
        for seg in chain.segments:
            jnt = seg.getJoint()
            if jnt.getType() != PyKDL.Joint.NoneJoint and jnt.getName() in joint_states:
                joint_array[idx] = joint_states[jnt.getName()]
                idx += 1

        end_frame = PyKDL.Frame()
        fk_solver.JntToCart(joint_array, end_frame)

        pos = end_frame.p
        rot = end_frame.M.GetRPY()
        print(f"[{joint_name}] pose of child link `{link_name}`:")
        print(f"  Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        print(f"  Rotation (RPY): ({rot[0]:.3f}, {rot[1]:.3f}, {rot[2]:.3f})")

if __name__ == "__main__":
    main()
