#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

from laser_assembler.srv import AssembleScans2


def main():
    rospy.init_node("laser2pc")
    rospy.wait_for_service("assemble_scans2")
    r = rospy.Rate(5)

    assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
    pub = rospy.Publisher("/laser_pointcloud", PointCloud2, queue_size=1)

    start_time = rospy.get_rostime()

    while (True):
        try:
            resp = assemble_scans(start_time, rospy.get_rostime())
            print("Got cloud with %u points" % len(resp.cloud.data))
            pub.publish(resp.cloud)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            break
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo('%s stopped' % rospy.get_name())
