# based off marlins pick and place->iktest1->iktest2->iktest3_fail->current
import argparse
import struct
import sys
import copy
import rospy
import rospkg
import cv2
import os
import subprocess
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from vicon.vicon import vicon

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
import baxter_interface

# VICON: need to call roslaunch from terminal to get running, but also need to do some stuff on vicon machines end as well
bridge = cv_bridge.CvBridge()

dropPt_x = 0
dropPt_y = 0
dropPt_r = 0

class PickAndPlace(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0] * 7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}

        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
            }.get(resp_seeds[0], 'None')

            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                    (seed_str)))

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                # print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

    ################## CAMERA FUNCTIONS WITHIN #######################################
    def closeCameras(s):
        if (s is "head"):
            os.system("python camera_control.py -c head_camera")
        elif (s is "left"):
            os.system("python camera_control.py -c left_hand_camera")
        elif (s is "right"):
            os.system("python camera_control.py -c right_hand_camera")

    def initializeCameras(s):
        if (s is "head"):
            os.system("python camera_control.py -o head_camera -r 1280x800")
        elif (s is "left"):
            os.system("python camera_control.py -o left_hand_camera -r 1280x800")
        elif (s is "right"):
            os.system("python camera_control.py -o right_hand_camera -r 1280x800")

    def view_cameras(s):
        if (s is "head"):
            os.system("rosrun image_view image_view image:=/cameras/head_camera/image")
        elif (s is "left"):
            os.system("rosrun image_view image_view image:=/cameras/left_hand_camera/image")
        elif (s is "right"):
            os.system("rosrun image_view image_view image:=/cameras/right_hand_camera/image")

    def image_callback(self, ros_img):
        # Convert received image message to OpenCv image
        src_img = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
        output = src_img.copy()
        gray_img = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
        hsv_img = cv2.cvtColor(src_img, cv2.COLOR_BGR2HSV)

        # red range here doesn't work well, needs to wrap around 255 0 or something like that, fix is to combine mask
        # lower = np.array([0, 100, 100])
        # upper = np.array([20, 255, 255])

        # red range here works better,but not using fix proposed from online forums
        # lower = np.array([0, 50, 20])
        # upper = np.array([5, 255, 255])

        circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 1.2, 1)
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (255, 0, 255), 4)


        # yellow
        lower = np.array([22, 93, 0], dtype="uint8")
        upper = np.array([45, 255, 255], dtype="uint8")

        mask = cv2.inRange(hsv_img, lower, upper)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]


        # print len(cnts), " contours"
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(src_img, (x, y), (x + w, y + h), (12, 255, 12), 2)
            centroid_x, centroid_y = (x + w) / 2, (y + h) / 2
            cv2.rectangle(src_img, (316, 205), (316+5, 205+5), (255, 0, 0), 1)
            # prevents robot from crawling
            if (True):
                if centroid_x > 316 + 5:
                    adjust_high_X = True
                    adjust_low_X = False

                if centroid_x < 316 - 5:
                    adjust_high_X = False
                    adjust_low_X = True

                if centroid_y > 205 + 5:
                    adjust_high_Y = True
                    adjust_low_Y = False

                if centroid_y < 205 - 5:
                    adjust_high_Y = False
                    adjust_low_Y = True

        global adjust_high_X, adjust_low_X, adjust_high_Y, adjust_low_Y
        cv2.imshow("output", src_img)
        # cv2.imshow('Image')  # display image
        cv2.waitKey(1)

    def tuck(self, s):
        os.system("python tuck_arms.py -" + s)


###################### END CAMERA FUNCTIONS ##################################33

def main():
    rospy.init_node("ik_pick_and_place_demo")
    limb = 'right'  # this chooses arm, change to calibrate appropriate gripper and move arm
    # hover distance needs to be variable, it causes IK to fail sometimes, i think
    # if IK not found maybe change hover distance, get from IK function
    hover_distance = 0.15  # meters

    # call/define class? -- see comment below
    pnp = PickAndPlace(limb, hover_distance) # creates a new PickAndPlace object called pnp
    limb_choice = pnp._limb  # from class pnp, get limb passed to it
    gripper = pnp._gripper  # also get gripper
    ######lets try to get vicon data here for target pose etc##################
    ViconData = vicon()
    print
    ViconData
    ######################################################################
    # move to start position
    pnp.tuck('u')
    # when center over object, then call "pick" function to pick it up because hand orientation should already
    # be oriented along z-axis.  Pick function probably needs to be updated to slow down pick to accurately pick it up
    rospy.Subscriber('/cameras/right_hand_camera/image', Image, pnp.image_callback)

    # actual Starting Joint angles for arm
    starting_joint_angles = limb_choice.joint_angles()
    """""
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    """""
    # commented out for now, b/c annoying troubleshooting
    # print "starting joint angles are:"
    # print limb_choice.joint_angles()

    # gripper calibrate
    gripper.calibrate()
    print
    "Calibrate"
    if gripper.close():
        gripper.open()
        rospy.sleep(0.5)
        print("gripper closed, opening gripper")

    # An orientation for gripper fingers to be overhead and parallel to the obj
    #####lets change this, make it more variable for different applications, object maybe be at angle
    current_pose = limb_choice.endpoint_pose()
    x = current_pose['orientation'].x
    y = current_pose['orientation'].y
    z = current_pose['orientation'].z
    w = current_pose['orientation'].w

    overhead_orientation = Quaternion(x, y, z, w)
    block_poses = list()

    # fine for now, later have this as starting pose then adjust using new loop below
    # or maybe move to "general area" as start then loop/adjust
    block_poses.append(Pose(
        position=Point(x=0.65, y=-0.45, z=00.129),
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(x=0.75, y=0.0, z=-0.129),
        orientation=overhead_orientation))
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    ss = .02  # just for now, later have inner loop (better searching) with smaller stepsize to get accurate position
    # lets make a fine tuner loop or something to get it to stop bouncing around the target
    # careful about hover distance, it casues IK to fail sometimes
    while not rospy.is_shutdown():
        adapter_X = block_poses[idx].position.x
        adapter_Y = block_poses[idx].position.y
        if adjust_high_X == True:
            print "Adjust X for high\n(old,new)"
            adapter_X += -ss
            print(block_poses[idx].position.x, adapter_X)
        if adjust_low_X == True:
            print "Adjust X for low\n(old,new)"
            adapter_X += ss
            print(block_poses[idx].position.x, adapter_X)
        if adjust_high_Y == True:
            print "Adjust Y for high\n(old,new)"
            adapter_Y += -ss
            print(block_poses[idx].position.y, adapter_Y)
        if adjust_low_Y == True:
            print "Adjust Y for low\n(old,new)"
            adapter_Y += ss
            print(block_poses[idx].position.y, adapter_Y)

        # adjust x and y values to center of centroid, not working yet, problem in imagecallback
        block_poses[idx].position.x = adapter_X
        block_poses[idx].position.y = adapter_Y
        # call to approach function, which calls IK function with Z distance and orientation constant
        pnp._approach(block_poses[idx])
        rospy.sleep(1.0)

    return 0


if __name__ == '__main__':
    main()
