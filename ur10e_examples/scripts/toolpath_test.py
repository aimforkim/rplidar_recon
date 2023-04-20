#!/usr/bin/env python3
from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_commander.conversions import list_to_pose

from move_group_utils.move_group_utils import (MoveGroupUtils,
                                               poses_list_from_yaml,
                                               publish_trajectory_markers)
from pilz_robot_program.pilz_robot_program import Lin, Ptp, Sequence

# define endeffector
# tcp pose should match static tf declared in launch file
ee_name = 'D405'
tcp_pose = Pose(position=Point(0.0, 0.0, 0.045),
                orientation=Quaternion(0, 0, 0, 1))
size = [0.042, 0.042, 0.023]

vel = 0.1
acc = 0.1


def robot_program():

    mgi = MoveGroupUtils()
    rospy.sleep(1.0)

    mgi.add_ground_cube()

    # attach camera and set new tcp
    mgi.attach_camera(ee_name, tcp_pose, size)
    mgi.move_group.set_end_effector_link(f'{ee_name}/tcp')
    rospy.loginfo(
        f'{mgi.name}: end effector link set to {mgi.move_group.get_end_effector_link()}'
    )

    sequence = Sequence()

    home = (0.0, -pi/2.0, pi/2.0, 0.0, pi/2.0, -pi/2)
    sequence.append(Ptp(goal=home, vel_scale=0.2, acc_scale=0.2))
    # create pose mgs list from yaml
    poses_list = poses_list_from_yaml(
        '/dev_ws/src/ur10e_examples/toolpaths/rplidar_scan.yaml')

    poses = [list_to_pose(pose) for pose in poses_list]

    # publish the poses to rviz for preview
    mgi.publish_pose_array(poses)

    for p in poses:
        sequence.append(Lin(goal=p, vel_scale=vel, acc_scale=acc))

    # publish the poses to rviz for preview
    mgi.publish_pose_array(poses)

    rospy.sleep(2.0)
    success, plan = mgi.sequencer.plan(sequence)[:2]

    # for i in range(10):
    # pose1_jointstate = mgi.get_ik(pose1)
    # print(pose1_jointstate)
    # mgi.display_trajectory(plan)
    # publish_trajectory_markers(plan[0])

    if not success:
        return rospy.logerr(f'{mgi.name}: Failed to plan to sequence')
    mgi.sequencer.execute()

    return rospy.loginfo('Robot program completed')


if __name__ == '__main__':

    robot_program()
