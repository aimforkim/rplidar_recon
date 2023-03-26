#!/usr/bin/env python
import roslib
import rospy

from laser_assembler.srv import *

roslib.load_manifest('laser_assembler')

rospy.init_node("test_client")
rospy.wait_for_service("assemble_scans")
try:
    assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)

    resp = assemble_scans(rospy.Time(0, 0), rospy.get_rostime())
    print("Got cloud with %u points" % len(resp.cloud.points))

except rospy.ServiceException as e:
    print("Service call failed: %s" % e)
