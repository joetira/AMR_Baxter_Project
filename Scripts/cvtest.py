# # based off marlins pick and place->iktest1->iktest2->iktest3->iktest4->current
# ##doesn't use pertubation method anymore, strictly IK from dist 2 obj using camera image ratio to find location of
# # object relative to center of camera
# ## can pick up object but its offset, this is currently what working on
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
from sensor_msgs.msg import Image, Range
# from vicon.vicon import vicon
from transformations import quaternion_from_euler, euler_from_quaternion
from tf.transformations import *
import math
from shapedetector import ShapeDetector
import imutils


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


#Offset Camera->Gripper
x_offset = 0
y_offset = 0
z_offset = 0

xbar = 0
ybar = 0

class PickAndPlace(object):
    def __init__(self, limb, verbose=True):
        self._limb_name = limb  # string
        #self._hover_distance = hover_distance  # in meters
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
        # print("Moving the {0} arm to start pose...".format(self._limb_name))
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
                # print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                # (seed_str)))

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

    def _approach(self, pose,hd):
        # print("approach fun")
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + hd
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self,hd):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + hd
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        # print("servo 2 pose fun")
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose, hd):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose,hd)
        # close gripper
        self.gripper_close()
        # servo to pose
        self._servo_to_pose(pose)
        # retract to clear object
        self._retract(hd)

    def place(self, pose):
        # servo above pose
        self._approach(pose,hd)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract(hd)

    def isInsideCircle(self, cir_x, cir_y, cir_r, centroid_x, centroid_y):
        return math.dist((cir_x, cir_y), (centroid_x, centroid_y)) <= cir_r

    def image_callback(self, ros_img):
        # Convert received image message to OpenCv image
        cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
        circle_image = cv_image.copy()
        shape_image = cv_image.copy()
        final = cv_image.copy()

        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.blur(gray_img, (3, 3))
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # red range here doesn't work well, needs to wrap around 255 0 or something like that, fix is to combine mask
        # lower = np.array([0, 100, 100])
        # upper = np.array([20, 255, 255])

        # red range here works better,but not using fix proposed from online forums
        # lower = np.array([0, 50, 20])
        # upper = np.array([5, 255, 255])

        # yellow
        lower = np.array([22, 93, 0], dtype="uint8")
        upper = np.array([45, 255, 255], dtype="uint8")

        mask = cv2.inRange(hsv_image, lower, upper)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = cnts[0]

        _, thresh  = cv2.threshold(gray_img, 240, 255,cv2.THRESH_BINARY)
        contours,_ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for c in contours:
            x,y,w, h = cv2.cv2.boundingRect(c)
            cv2.rectangle(shape_image, (x,y), (x+w,y+h), (12, 255, 12), 2)


        # find centroid location
        M = cv2.moments(cnt)
        if M['m00'] > 0:
            xbar = int(M['m10'] / M['m00'])
            ybar = int(M['m01'] / M['m00'])
            # global xbar, ybar

        # temp=self.convert_image_pixel_to_cm()

        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cv2.rectangle(cv_image, (318, 198), (322, 202), (0, 0, 255), 1)
        # print len(cnts), " contours"
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            if (w > 20 and h > 20):
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (12, 255, 12), 2)
            reticle = [320, 200]  # reticle centered location
            # prevents robot from crawling
            if xbar > reticle[0]:
                adjust_high_X = True
                adjust_low_X = False

            if xbar < reticle[0]:
                adjust_high_X = False
                adjust_low_X = True

            # positve y is in conventional negative direction
            if ybar > reticle[1]:
                adjust_high_Y = True
                adjust_low_Y = False

            if ybar < reticle[1]:
                adjust_high_Y = False
                adjust_low_Y = True

        global adjust_high_X, adjust_low_X, adjust_high_Y, adjust_low_Y


        circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 1, 20, param1 = 50,
               param2 = 30, minRadius = 40, maxRadius = 80) # (1.7. 5)
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(circle_image, (x, y), r, (255, 0, 255), 4)
                if self.isInsideCircle(x, y, r, xbar, ybar):
                    cv2.circle(final, (x,y), r, (0, 255, 0), 5)

        cv2.imshow("output", np.hstack([cv_image, circle_image, final, shape_image]))
        cv2.waitKey(1)


    def get_distance(self, irdist):
        rangeD = irdist.range
        # is there better way than using global to return function value, I cant get return working right in main due to being called from ros_subscriber
        global rangeD

    def get_pose(self):  # rangeD is global, get rid of this input
        current_pose = self._limb.endpoint_pose()
        x = current_pose['position'].x
        y = current_pose['position'].y
        z = current_pose['position'].z
        qx = current_pose['orientation'].x
        qy = current_pose['orientation'].y
        qz = current_pose['orientation'].z
        qw = current_pose['orientation'].w

        # need to verfiy all this, it might not give what i think it does
        # also incoporate the rangeD so can get z distance
        xp = [x, y, z]
        quat = [qx, qy, qz, qw]
        T_BG = quaternion_matrix(quat)  # gives back rotational homogenous transform matrix from quaternion
        P_BG = [x, y, z, 1]
        T_BG[:, 3] = P_BG  # with translation of the gripper from the base

        R_BG = T_BG[:3, :3]

        # find vector 2 target object
        temp = self.convert_image_pixel_to_cm()
        reticle = [320, 200]
        dist2obj = np.subtract([xbar, ybar], reticle)
        convert_dist2obj = np.array(temp) * np.array(dist2obj) / 100

        b = np.array([0, 1])  # prob no needed, keep for now incase
        b2 = np.array([0])
        camera_vec = np.concatenate((convert_dist2obj, b))  # prob no needed, keep for now incase
        camera_vec2 = np.concatenate((convert_dist2obj, b2))

        # camera to gripper frame
        T_GO = euler_matrix(0, 0, -math.pi / 2, 'sxyz')
        R_GO = T_GO[:3,
                    :3]  # extract rotational componenet, find way to just get rot matrix, make function yourself if needed
        P_gripperFrame = R_GO.dot(camera_vec2)  # in gripper coordinates
        # print P_gripperFrame
        P_baseFrame = R_BG.dot(P_gripperFrame)
        # print P_baseFrame
        T_GO[:, 3] = camera_vec
        # print T_GO

        # T_BO=np.matmul(T_BG, T_GO)

        return xp, quat, T_BG, P_baseFrame

    def convert_image_pixel_to_cm(self):
        # linear interpolated function based on range data, converts pixels to [cm] width and height
        # need averages for width b/c camera is skewed top and bottom,  top dimension is slightly larger than bottom
        # found this using MATLAB, can do in python but don't have time atm
        W = 151.522 * rangeD + 3.4151
        H = 88.3429 * rangeD + 2.8822
        CW = 640
        CH = 400
        cm_pixel_ratio = [W / CW, H / CH]
        return cm_pixel_ratio

    def tuck(self, s):
        os.system("python tuck_arms.py -" + s)


###################### END CAMERA FUNCTIONS ##################################33

# Centers the camera onto the yellow ball such that the grippers will be in place to pick up the ball
def CenterOnBall(pnp, idx, hd, block_poses):
    # P_B = [xp, quat, T_BG, P_baseFrame]
    P_B = pnp.get_pose()
    course_Adjust = P_B[3]
    newX = block_poses[idx].position.x + course_Adjust[0]
    newY = block_poses[idx].position.y + course_Adjust[1]
    # print("pb3", P_B[3])
    block_poses[idx].position.x = newX
    block_poses[idx].position.y = newY
    pnp._approach(block_poses[idx],hd)
    # 320, 200
    print (xbar, ybar)

    xCenter = (xbar > 315 and xbar < 325)
    yCenter = (ybar > 195 and ybar < 205)
    return (xCenter and yCenter)


def main():
    rospy.init_node("master")
    limb = 'right'  # this chooses arm, change to calibrate appropriate gripper and move arm
    # hover distance needs to be variable, it causes IK to fail sometimes, i think
    # if IK not found maybe change hover distance, get from IK function
    hd = -0.21  # meters

    # call/define class? -- see comment below
    pnp = PickAndPlace(limb)  # creates a new PickAndPlace object called pnp
    limb_choice = pnp._limb  # from class pnp, get limb passed to it
    gripper = pnp._gripper  # also get gripper
    ######lets try to get vicon data here for target pose etc##################
    # ViconData = vicon()
    # print ViconData
    ######################################################################
    # move to start position
    pnp.tuck('u')
    # when center over object, then call "pick" function to pick it up because hand orientation should already
    # be oriented along z-axis.  Pick function probably needs to be updated to slow down pick to accurately pick it up
    rospy.Subscriber('/cameras/right_hand_camera/image', Image, pnp.image_callback)
    rospy.Subscriber('/robot/range/right_hand_range/state', Range, pnp.get_distance)

    # actual Starting Joint angles for arm
    starting_joint_angles = limb_choice.joint_angles()

    # commented out for now, b/c annoying troubleshooting
    # print "starting joint angles are:"
    # print limb_choice.joint_angles()

    # gripper calibrate
    gripper.calibrate()
    print "Calibrate Grippers"
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
        position=Point(x=0.85, y=-0.50, z=00.129),
        orientation=overhead_orientation))
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    ss = .02  # just for now, later have inner loop (better searching) with smaller stepsize to get accurate position


    pnp._approach(block_poses[idx], hd)

    while not rospy.is_shutdown():
        # input = raw_input("Enter Comomand:\n")
        # if input == "pick":
        #     pnp.pick(pnp.get_pose(), hd)
        pass
    return 0


if __name__ == '__main__':
    main()

# Attempted ZED Camera access - missing Python API
# import pyzed.sl as sl
#
# # Create an RGBA sl.Mat object
# image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
# # Retrieve data in a numpy array with get_data()
# image_ocv = image_zed.get_data()
#
# if zed.grab() == sl.ERROR_CODE.SUCCESS :
#     # Retrieve the left image in sl.Mat
#     zed.retrieve_image(image_zed, sl.VIEW.LEFT)
#     # Use get_data() to get the numpy array
#     image_ocv = image_zed.get_data()
#     # Display the left image from the numpy array
#     cv2.imshow("Image", image_ocv)