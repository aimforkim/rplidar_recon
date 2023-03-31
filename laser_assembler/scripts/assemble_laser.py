#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud

from laser_assembler.srv import *


def main():
    rospy.init_node("assemble_laser", anonymous=True)
    rospy.wait_for_service("assemble_scans")

    r = rospy.Rate(1)
    assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
    pub = rospy.Publisher("/assembled_cloud", PointCloud, queue_size=1)

    while True:
        try:
            resp = assemble_scans(rospy.Time(5, 0), rospy.get_rostime())
            print("Got cloud with %u points" % len(resp.cloud.points))
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
