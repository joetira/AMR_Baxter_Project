# based off marlins pick and place->iktest1->iktest2->iktest3->iktest4->current
##doesn't use pertubation method anymore, strictly IK from dist 2 obj using camera image ratio to find location of
# object relative to center of camera
## can pick up object but its offset, this is currently what working on
import argparse
import sys
import rospy
import rospkg
import cv2
import os
import subprocess
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, Range
# from vicon.vicon import vicon
from transformations import quaternion_from_euler, euler_from_quaternion
from tf.transformations import *
import baxter_interface
import imutils
from imutils import perspective
from imutils import contours
from scipy.spatial import distance as dist
from PickAndPlace import PickAndPlace

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

# VICON: need to call roslaunch from terminal to get running, but also need to do some stuff on vicon machines end as well


#Offset Camera->Gripper
x_offset = 0
y_offset = 0
z_offset = 0


def tuck(s):
    os.system("python tuck_arms.py -" + s)


# Centers the camera onto the yellow ball such that the grippers will be in place to pick up the ball
def CenterOnBall(pnp, idx, hd, initial_poses, z_theta):
    # P_B = [xp, current quat, new quat, P_baseFrame, P_baseFrame_offset, P_calibrate offset]
    P_B = pnp.get_pose(z_theta)

    course_Adjust = P_B[3]
    real_Adjust = P_B[4]
    calibrate_Adjust = P_B[5]

    newX = initial_poses[idx].position.x + course_Adjust[0]
    newY = initial_poses[idx].position.y + course_Adjust[1]

    initial_poses[idx].position.x = newX
    initial_poses[idx].position.y = newY
    pnp._approach(initial_poses[idx], hd)

    realX = initial_poses[idx].position.x + real_Adjust[0]
    realY = initial_poses[idx].position.y + real_Adjust[1]

    calX = initial_poses[idx].position.x + calibrate_Adjust[0]
    calY = initial_poses[idx].position.y + calibrate_Adjust[1]
    #reticle loc: 320, 200
    rospy.sleep(1.5)

    xCenter = (pnp.xbar >= 315 and pnp.xbar <= 325)
    yCenter = (pnp.ybar >= 195 and pnp.ybar <= 205)
    return xCenter, yCenter, realX, realY, calX, calY


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", required=False, help="Print additional information")
    args = vars(ap.parse_args())


    rospy.init_node("master")
    limb = 'right'  # this chooses arm, change to calibrate appropriate gripper and move arm
    hd = -0.15  # meters, don't get too close or range value gives vale of object, causing either z2pick to over or undershoot
    z_theta = -1.50  #for total [-3.059 3.059]  this will be variable based on object type - later
    # call/define class? -- see comment below
    pnp = PickAndPlace(limb)  # creates a new PickAndPlace object called pnp
    limb_choice = pnp._limb  # from class pnp, get limb passed to it
    gripper = pnp._gripper  # also get gripper
    ######lets try to get vicon data here for target pose etc##################
    # ViconData = vicon()
    # print ViconData
    ######################################################################
    # move to start position
    tuck('u')

    rospy.Subscriber('/cameras/right_hand_camera/image', Image, pnp.image_callback)
    rospy.Subscriber('/robot/range/right_hand_range/state', Range, pnp.get_distance)

    #rospy.spin()
    # actual Starting Joint angles for arm
    starting_joint_angles = limb_choice.joint_angles()

    # gripper calibrate
    gripper.calibrate()
    if args["verbose"]:
        print "Calibrate Grippers"
    if gripper.close():
        gripper.open()
        rospy.sleep(0.5)

    # An orientation for gripper fingers to be overhead and parallel to the obj
    #####lets change this, make it more variable for different applications, object maybe be at angle
    #current_pose = limb_choice.endpoint_pose()

    new_pose=pnp.get_pose(z_theta)
    overhead_orientation=new_pose[2]

    initial_poses = list()

    # fine for now, later have this as starting pose then adjust using new loop below
    # or maybe move to "general area" as start then loop/adjust
    initial_poses.append(Pose(
        position=Point(x=0.65, y=-0.45, z=00.129),
        orientation=overhead_orientation))
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    while not rospy.is_shutdown():
        continue
        return 0



if __name__ == '__main__':
    main()
