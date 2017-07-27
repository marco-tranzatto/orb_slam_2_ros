#!/usr/bin/env python

#
#  Title:        generate_cam_params_file.py
#  Description:  ROS module to generate camera parameters file from a camera_info (OF THE RIGHT HAND SIDE CAMERA!)
#                topic published by a rectification node. The generated parameters 
#                can be copied and pasted in the default yaml file read by orb_slam2. 
#

import rospy

import os
import sys
import time

from sensor_msgs.msg import CameraInfo

class GenerateCamParamsFile:
    def __init__(self):
        rospy.loginfo(rospy.get_name() + " start")

        rospy.Subscriber("right_camera/camera_info", CameraInfo,
                         self.camera_info_callback)

        rospy.spin()

    def camera_info_callback(self, msg):

        fx = msg.K[0]
        fy = msg.K[4]
        cx = msg.K[2]
        cy = msg.K[5]

        k1 = msg.D[0]
        k2 = msg.D[1]
        p1 = msg.D[2]
        p2 = msg.D[3]

        width = msg.width
        height = msg.height

        # msg.P[3] = -fx * baseline, ORB_SLAM_2 requires stereo baseline times fx
        bf = -msg.P[3]

        # Write params waypoints to file
        script_path = os.path.dirname(os.path.realpath(sys.argv[0]))
        desired_path = "%s/../../config/generated_cam_params.yaml" % script_path
        file_obj = open(desired_path, 'w')

        file_obj.write("# File generated on " + time.strftime("%Y-%m-%d-%H-%M-%S") + "\n")
        file_obj.write("#\n")
        file_obj.write("# ----- Copy and paste the following values in the yaml file loaded by orb_slam_2_ros node. -----\n")
        file_obj.write("#\n\n")
        file_obj.write("# Camera Parameters. \n\n")

        file_obj.write("Camera.fx: %.3f \n" % fx)
        file_obj.write("Camera.fy: %.3f \n" % fy)
        file_obj.write("Camera.cx: %.3f \n" % cx)
        file_obj.write("Camera.cy: %.3f \n\n" % cy)

        file_obj.write("Camera.k1: %.3f \n" % k1)
        file_obj.write("Camera.k2: %.3f \n" % k2)
        file_obj.write("Camera.p1: %.3f \n" % p1)
        file_obj.write("Camera.p2: %.3f \n\n" % p2)

        file_obj.write("Camera.width: %d \n" % width)
        file_obj.write("Camera.height: %d \n\n" % height)

        file_obj.write("Camera.bf: %.13f \n\n" % bf)

        file_obj.close()

        rospy.loginfo(rospy.get_name() + 
            " parameters written in %s" % desired_path)
        rospy.signal_shutdown("")


if __name__ == '__main__':

    rospy.init_node('generate_cam_params_file')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        generate_cam_params_file = GenerateCamParamsFile()
    except rospy.ROSInterruptException:
        pass