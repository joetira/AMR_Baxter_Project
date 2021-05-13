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
from PickAndPlace2 import PickAndPlace
from PickAndPlace2 import get_dice_from_blobs
from PickAndPlace2 import get_blobs
import random
#from towerpnp import PickAndPlace

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

    #loose tolerane, can probably use for smaller die
    xCenter = (pnp.xbar >= 312 and pnp.xbar <= 328)  #original values 315 and 325
    yCenter = (pnp.ybar >= 192 and pnp.ybar <= 208) #original values 195 and 205
    #tight tolerance, used for now so it doesn't crush die
    #xCenter = (pnp.xbar >= 315 and pnp.xbar <= 325)  #original values 315 and 325
    #yCenter = (pnp.ybar >= 195 and pnp.ybar <= 205) #original values 195 and 205
    xCenter_rot = (pnp.xbar >= 305 and pnp.xbar <= 335)
    yCenter_rot = (pnp.ybar >= 185 and pnp.ybar <= 215)

    return xCenter, yCenter, realX, realY, calX, calY, xCenter_rot, yCenter_rot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", required=False, help="Print additional information")
    args = vars(ap.parse_args())


    rospy.init_node("master")
    limb = 'right'  # this chooses arm, change to calibrate appropriate gripper and move arm
    left_Limb = 'left'

    hd = -0.15  # meters, don't get too close or range value gives vale of object, causing either z2pick to over or undershoot
    z_theta_i = 0 #for total [-3.059 3.059]  this will be variable based on object type - later was-1.5

    # call/define class? -- see comment below
    pnp = PickAndPlace(limb)  # creates a new PickAndPlace object called pnp
    pnp_L = PickAndPlace(left_Limb)

    limb_choice = pnp._limb  # from class pnp, get limb passed to it
    left_limb_choice = pnp_L._limb

    gripper = pnp._gripper  # also get gripper
    gripper_L = pnp_L._gripper
    ######lets try to get vicon data here for target pose etc##################
    # ViconData = vicon()
    # print ViconData
    ######################################################################
    # move to start position
    tuck('u')
    #move left arm out of way so less shadows
    starting_joint_angles_left = {'left_w0': 0.6699952259595108, 'left_w1': 1.030009435085784, 'left_w2': -0.4999997247485215,
                                  'left_e0': -1.189968899785275, 'left_e1': 1.9400238130755056, 'left_s0': 1.5015681889618952,
                                    'left_s1': -0.9999781166910306}

    rospy.Subscriber('/cameras/right_hand_camera/image', Image, pnp.image_callback)
    rospy.Subscriber('/robot/range/right_hand_range/state', Range, pnp.get_distance)

    #rospy.spin()
    # actual Starting Joint angles for arm
    starting_joint_angles = limb_choice.joint_angles()

    # gripper calibrate
    gripper.calibrate()
    gripper_L.calibrate()
    if args["verbose"]:
        print "Calibrate Grippers"
    if gripper.close():
        gripper.open()
        rospy.sleep(0.5)

    new_pose=pnp.get_pose(z_theta_i)
    overhead_orientation=new_pose[2]

    initial_poses = list()

    initial_poses.append(Pose(
        position=Point(x=0.60, y=-0.25, z=00.129),
        orientation=overhead_orientation))
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    pnp_L.move_to_start(starting_joint_angles_left)
    pnp_L.move_to_start(starting_joint_angles_left)

    idx = 0

    pnp._approach(initial_poses[idx], hd)

    print "==================="
    user_bet = int(input("Enter bet (1-6) for dice roll, : "))
    print "\n"
    willbeinputlater = 1  #input from user for die side
    baxter_bet = random.randint(1, 6)
    while baxter_bet == user_bet:
        baxter_bet = random.randint(1, 6)
    print "++++++++++++++++++++++"
    print "Mr. Baxter bets on ", baxter_bet
    print "User bets on ", user_bet
    print "++++++++++++++++++++++"
    print "\n"

    while not rospy.is_shutdown():
        print ("centering")

        center_cond = CenterOnBall(pnp, idx, hd, initial_poses, z_theta_i)

        if center_cond[0]==True and center_cond[1]==True:

            ik_gripper_pose = CenterOnBall(pnp, idx, hd, initial_poses, z_theta_i)

            initial_poses.append(initial_poses[idx])
            idx = idx + 1

            ik_X = ik_gripper_pose[2]
            ik_Y = ik_gripper_pose[3]

            ik_calX = ik_gripper_pose[4]
            ik_calY = ik_gripper_pose[5]

            adj_p = pnp.get_pose(z_theta_i)
            overhead_orientation = adj_p[6]
            initial_poses[idx].orientation = overhead_orientation
            print("Changing orientation")
            pnp._approach(initial_poses[idx], hd)


            initial_poses[idx].position.x = ik_calX
            initial_poses[idx].position.y = ik_calY

            if args["verbose"]:
                print("Moving to distance calibration position")
            pnp._approach(initial_poses[idx], hd)

            rospy.sleep(1.0)
            surface_range = pnp.rangeD
            if args["verbose"]:
                print("Surface Range Estimate", pnp.rangeD)

            initial_poses[idx].position.x = ik_X
            initial_poses[idx].position.y = ik_Y

            pnp._approach(initial_poses[idx], hd)
            gripper_Zoffset = 0.12  #10 cm from camera with 2 cm safety factor, this can change later
            z2pick = gripper_Zoffset-surface_range
            if args["verbose"]:
                print("Moving to pick position", z2pick * 100, "[cm]")

            initial_poses[idx].position.z = initial_poses[idx].position.z+z2pick    #z2pick is negative value so add to z pose below

            more_force = [[0, 0],[100, 100]]
            #pnp.gripper_control([20, 0.01])
            #rospy.sleep(.1)
            if args["verbose"]:
                print("Pick Function init")

            #for changing drop orientation, not really needed with new printed die, it bounces well enough, keep for now just incase
            #drop_orientation = pnp.get_pose(0.223599)
            #dice_roll_pose = Pose(
                #position=initial_poses[idx].position,
                #orientation=drop_orientation[7])



            #pnp.pick(initial_poses[idx], hd, more_force)
            pnp.roll_die(initial_poses[idx], hd, more_force)
            print "rolling"
            pnp.gripper_open()


            initial_poses[idx].position.z = 0.129
            pnp._approach(initial_poses[idx], hd)

            center_cond = CenterOnBall(pnp, idx, hd, initial_poses, z_theta_i)
            center_cond[0]==False
            center_cond[1]==False


            if pnp.die is not None:
                print(pnp.die.value)
                if pnp.die.value == user_bet or pnp.die.value == baxter_bet:
                    print "++++++++++++++++++++++"
                    print "Dice roll of", pnp.die.value, " successfully found"
                    if pnp.die.value == user_bet:
                        print "user wins"
                        print "++++++++++++++++++++++"
                    if pnp.die.value == baxter_bet:
                        print "Mr. Baxter wins"
                        print "++++++++++++++++++++++"
                    return 0

            if pnp.die is not None:
                print("value of",pnp.die.value)
            print "++++++++++++++++++++++"
            print "rerolling"
            print "++++++++++++++++++++++"
            #return 0


if __name__ == '__main__':
    main()
