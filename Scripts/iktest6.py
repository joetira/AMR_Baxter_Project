# based off marlins pick and place->iktest1->iktest2->iktest3->iktest4->current
##doesn't use pertubation method anymore, strictly IK from dist 2 obj using camera image ratio to find location of
# object relative to center of camera
## can pick up object but its offset, this is currently what working on
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
from math import pi
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
        #print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        #print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        # print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0] * 7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Start Pose. Ctrl-c to quit")

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
        ik_pose.position.z = current_pose['position'].z - hd  #changed to negative so it goes up, after picking
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

    def pick(self, pose, hd, grip_control_param):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose, hd)
        # close gripper
        self.gripper_control(grip_control_param[0])
        self.gripper_close()
        # servo to pose
        self._servo_to_pose(pose)
        print("Updating Gripper Forces")
        self.gripper_control(grip_control_param[1])
        self.gripper_close()
        # crush ball, testing, this can move, need to sleep to wait for update
        # retract to clear object
        self._retract(hd/4)

    def place(self, pose, hd):
        # servo above pose
        self._approach(pose, hd)
        # servo to pose
        #self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract(hd)

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
        cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
        output = cv_image.copy()
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) #not sure why duplicate, can probably just assign 2nd to 1st var
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # yellow
        #lower = np.array([22, 93, 0], dtype="uint8")
        #upper = np.array([45, 255, 255], dtype="uint8")

        #blue doesn't work well b/c camera is trash
        #lower = np.array([100,150,0],np.uint8)
        #upper = np.array([120,255,255],np.uint8)

        # red range here works better,but not using fix proposed from online forums
        lower = np.array([0, 50, 20])
        upper = np.array([5, 255, 255])
        """  circle detection may just delete
        #morhology blur is just one type, many types to do this. experiment what works for our case, probably changes based on shadows
        kernel = np.ones((10,10),np.uint8)
        closing = cv2.morphologyEx(gray_img, cv2.MORPH_CLOSE, kernel)

        # for circle detection
        circles = cv2.HoughCircles(closing, cv2.HOUGH_GRADIENT, 1.1, 20, param1 = 50, param2 = 40)
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (255, 0, 255), 4)
        """
        mask = cv2.inRange(hsv_image, lower, upper)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cnt = cnts[0]
        # find centroid location of color detection
        #M = cv2.moments(cnt)
        #if M['m00'] > 0:
            #xbar = int(M['m10'] / M['m00'])
            #ybar = int(M['m01'] / M['m00'])
            #print("color centroid", xbar,ybar) seems to match well with shape detection centroid so far
            #global xbar, ybar


        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # for color detection
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (12, 255, 12), 2)
            cv2.rectangle(cv_image, (318, 198), (322, 202), (0, 0, 255), 1)
            reticle = [320, 200]  # reticle centered location
            # prevents robot from crawling
            """ for color based search
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
            """

        ##################-test shapes from tutorial-################################
        #######https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/

        #COMMENTED OUT IS THE FOR ORIGINAL IMAGE SIZE, but i think this works better with smaller image size as per tutorial
        resized = imutils.resize(hsv_image, width=300)
        ratio = hsv_image.shape[0] / float(resized.shape[0])
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        #gray = cv2.cvtColor(hsv_image, cv2.COLOR_BGR2GRAY) #for full size

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        cnts2 = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = imutils.grab_contours(cnts2)
        cnt2 = cnts2[0]
        M2 = cv2.moments(cnt2)

        #det_inst = self.detect(cnt2)
        #shape = det_inst[0]
        #vertices = det_inst[1]

        """#inside or outside function?
        if M2["m00"] > 0:
            xbar = int((M2["m10"] / M2["m00"]) * ratio)
            ybar = int((M2["m01"] / M2["m00"]) * ratio)
            # print("shape centroid", cX,cY)
            # cX = int((M2["m10"] / M2["m00"]))  #for full size
            # cY = int((M2["m01"] / M2["m00"]))  #for full size
            cv2.putText(hsv_image, shape, (xbar, ybar), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            global xbar, ybar
        """

        for c in cnts2:
            reticle = [320, 200]  # reticle centered location
            cv2.rectangle(hsv_image, (318, 198), (322, 202), (0, 0, 255), 1)
            # compute the center of the contour, then detect the name of the shape using only the contour
            #M2 = cv2.moments(c)

            shape = self.detect(c)
            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")      #reduced size
            c *= ratio             #reduced size
            c = c.astype("int")    #reduced size
            cv2.drawContours(hsv_image, [c], -1, (0, 255, 0), 2)   #reduced size
            #cv2.drawContours(hsv_image, c, -1, (0,255,0), 3)       #for full size

              #what are benefits to being here instead of outside the loop?  may not matter due to being called by subsciber so often
            if M2["m00"] > 0:
                xbar = int((M2["m10"] / M2["m00"]) * ratio)
                ybar = int((M2["m01"] / M2["m00"]) * ratio)
                #print("shape centroid", cX,cY)
                #xbar = int((M2["m10"] / M2["m00"]))  #for full size
                #ybar = int((M2["m01"] / M2["m00"]))  #for full size
                cv2.putText(hsv_image, shape, (xbar, ybar), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                global xbar, ybar


            if xbar > reticle[0]:
                adjust_high_X = True
                adjust_low_X = False

            if xbar < reticle[0]:
                adjust_high_X = False
                adjust_low_X = True

            if ybar > reticle[1]:
                adjust_high_Y = True
                adjust_low_Y = False

            if ybar < reticle[1]:
                adjust_high_Y = False
                adjust_low_Y = True

            ############3#stuff down here is trying to find edges or vertices of countours##############
            #cn = max(cnts2, key=cv2.contourArea)
            #cv2.drawContours(hsv_image, [cn], -1, (36, 255, 12), 2)


            #cv2.circle(hsv_image, tuple([t1,t2]), 8, (0, 50, 255), -1)
            #cv2.circle(hsv_image, tuple([t1, t2]), 8, (0, 50, 255), -1)

            #cv2.circle(hsv_image, tuple(vertices[1]), 8, (0, 255, 255), -1)
            #cv2.circle(hsv_image, tuple(vertices[2]), 8, (255, 50, 0), -1)
            #cv2.circle(hsv_image, tuple(vertices[3]), 8, (255, 255, 0), -1)

        #############end test shape detection################

        global adjust_high_X, adjust_low_X, adjust_high_Y, adjust_low_Y
        #cv2.imshow("output", np.hstack([cv_image, output]))
        #cv2.imshow("output", np.hstack([cv_image, hsv_image]))

        #need this b/c images are of different dimensions, or fix that and use stack
        cv2.imshow('Shape Detection', hsv_image)  # display shape
        cv2.imshow('Color Detection', cv_image)  # display color
        cv2.waitKey(1)

    def detect(self, c):
        #test function change or delte or do what John was doing, call from another script, b/c this script is getting too long
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.90 and ar <= 1.10 else "rectangle"
        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"
        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"
        # return the name of the shape
        return shape

    def get_distance(self, irdist):
        rangeD = irdist.range
        #if rangeD > 0.4:
            #print("Warning, Range is out of bounds, move closer")
        # is there better way than using global to return function value, I cant get return working right in main due to being called from ros_subscriber

        global rangeD


    def get_pose(self, z_theta):  # rangeD is global, get rid of this input
        current_pose = self._limb.endpoint_pose()
        x = current_pose['position'].x
        y = current_pose['position'].y
        z = current_pose['position'].z
        qx = current_pose['orientation'].x
        qy = current_pose['orientation'].y
        qz = current_pose['orientation'].z
        qw = current_pose['orientation'].w

        xp = [x, y, z]

        current_quaternion = [qx, qy, qz, qw]
        T_BG = quaternion_matrix(current_quaternion)  # gives back rotational homogenous transform matrix from quaternion
        P_BG = [x, y, z, 1]
        T_BG[:, 3] = P_BG  # with translation of the gripper from the base

        R_BG = T_BG[:3, :3]
        #print R_BG
        # print euler_from_quaternion(QB) #quat from base frame quat
        # print euler_from_quaternion(overhead_orientation_to_list)   #euler from gripper quat
        # print quaternion_multiply(QB, quaternion_conjugate(overhead_orientation_to_list))

        #change orientation
        new_rotation = quaternion_about_axis(z_theta, [0, 0, 1])

        x1 = new_rotation[0]
        y1 = new_rotation[1]
        z1 = new_rotation[2]
        w1 = new_rotation[3]
        new_orientation = [x1, y1, z1, w1]

        temp = self.quaternion_multiply(current_quaternion, new_orientation)
        new_quaternion = temp/np.linalg.norm(temp)
        new_quaternion = Quaternion(new_quaternion[0].item(), new_quaternion[1].item(), new_quaternion[2].item(), new_quaternion[3].item())

        #print "\n"
        #print "Distance to Object:", rangeD, "[m]"
        #print "\n"

        # find vector 2 target object
        conv = self.convert_image_pixel_to_cm()
        reticle = [320, 200]
        dist2obj = np.subtract([xbar, ybar], reticle)
        convert_dist2obj = np.array(conv) * np.array(dist2obj) / 100

        b = np.array([0])  # prob no needed, keep for now incase

        camera_vec = np.concatenate((convert_dist2obj, b))

        # camera to gripper frame
        T_GO = euler_matrix(0, 0, -math.pi / 2, 'sxyz')
        R_GO = T_GO[:3,:3]  # extract rotational componenet, find way to just get rot matrix, make function yourself if needed
        P_gripperFrame = R_GO.dot(camera_vec)  # in gripper coordinates

        #1.25 measured but this works better idk
        gripper_offset = np.array([4.0, 1.25, 0]) / 100.0  # z is really 10 but accounted for elsewhere
        calibrate_offset = np.array([6.0, 1.25, 0]) / 100.0  # this should search around target, get surface value for r
        calibrate_offset = np.array([0.0, 8.25, 0]) / 100.0  #for rectangle, have get data if rectnage based on shape or something

        P_gripperFrame_offset = R_GO.dot(camera_vec) + gripper_offset
        P_calibrate_offset = R_GO.dot(camera_vec) + calibrate_offset

        # print P_gripperFrame
        P_baseFrame = R_BG.dot(P_gripperFrame)
        P_baseFrame_offset = R_BG.dot(P_gripperFrame_offset)
        P_baseFrame_calibrate_offset = R_BG.dot(P_calibrate_offset)


        # T_BO=np.matmul(T_BG, T_GO)

        return xp, current_quaternion, new_quaternion, P_baseFrame, P_baseFrame_offset, P_baseFrame_calibrate_offset

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

    def gripper_control(self, grip_control_param):
        #most can get delted, just for trouble shooting
        moveForce = self._gripper.parameters()['moving_force']
        holdForce = self._gripper.parameters()['holding_force']
        GF=self._gripper.force() #ratio of max force float
        print("Initial [Move Force, Hold Force, Ratio]", moveForce, holdForce)
        offsetMove = grip_control_param[0]        #these will be inputs later
        offsetHold = grip_control_param[1]
        #used to update current
        #gripper.set_moving_force(moveForce + offsetMove)
        #gripper.set_holding_force(holdForce + offsetHold)

        #used to change total
        self._gripper.set_moving_force(grip_control_param[0])
        #self._gripper.set_holding_force(100)
        self._gripper.set_holding_force(grip_control_param[1])

        moveForce = self._gripper.parameters()['moving_force']
        holdForce = self._gripper.parameters()['holding_force']
        print("Updated [Move Force, Hold Force, Ratio]", moveForce, holdForce)
        rospy.sleep(0.1)

    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2
        y = -x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2
        z = x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2
        w = -x1 * x2 - y1 * y2 - z1 * x2 + w1 * w2
        return x, y, z, w

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

    xCenter = (xbar >= 315 and xbar <= 325)
    yCenter = (ybar >= 195 and ybar <= 205)
    return xCenter, yCenter, realX, realY, calX, calY


def main():
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
    pnp.tuck('u')

    rospy.Subscriber('/cameras/right_hand_camera/image', Image, pnp.image_callback)
    rospy.Subscriber('/robot/range/right_hand_range/state', Range, pnp.get_distance)

    #rospy.spin()
    # actual Starting Joint angles for arm
    starting_joint_angles = limb_choice.joint_angles()

    # gripper calibrate
    gripper.calibrate()
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
    idx = 0
    ss = .02  # just for now, later have inner loop (better searching) with smaller stepsize to get accurate position
    print("Step Size:", ss, "[m]")

    pnp._approach(initial_poses[idx], hd)

    input = ""
    i = 0
    while not rospy.is_shutdown():
        center_cond = CenterOnBall(pnp, idx, hd, initial_poses, z_theta)

        if (center_cond[0]==False and center_cond[1]==False):
            #this causes problems when too close to ball, maybe save range data from plane
            #while rangeD > 0.18:
                #hd -= 0.01
                #pnp._approach(initial_poses[idx], hd)
            CenterOnBall(pnp, idx, hd, initial_poses, z_theta)
            print("centering")
        elif center_cond[0]==True and center_cond[1]==True:

            obj_range_est = rangeD  #need to find ir offset to get this
            #print("Object Range Estimate",obj_range_est)

            #input = raw_input("Would you like to pick? [Y/n/a: micro adjustments(not working atm)]\n")

            #if input == "Y":
            if True:
                ik_gripper_pose = CenterOnBall(pnp, idx, hd, initial_poses, z_theta)

                ik_X = ik_gripper_pose[2]
                ik_Y = ik_gripper_pose[3]

                ik_calX = ik_gripper_pose[4]
                ik_calY = ik_gripper_pose[5]

                print("Moving to distance calibration position")
                initial_poses[idx].position.x = ik_calX
                initial_poses[idx].position.y = ik_calY
                pnp._approach(initial_poses[idx], hd)
                rospy.sleep(1.0)
                surface_range = rangeD
                print("Surface Range Estimate", rangeD)

                initial_poses[idx].position.x = ik_X
                initial_poses[idx].position.y = ik_Y
                pnp._approach(initial_poses[idx], hd)

                obj_depth = obj_range_est-surface_range
                #print("Estimated Object Depth", obj_depth)

                gripper_Zoffset = 0.12  #10 cm from camera with 2 cm safety factor, this can change later
                z2pick=gripper_Zoffset-surface_range
                print("Moving to pick position", z2pick * 100, "[cm]")
                initial_poses[idx].position.z = initial_poses[idx].position.z+z2pick    #z2pick is negative value so add to z pose below

                more_force = [[40,20],[100, 100]]
                #pnp.gripper_control([20, 0.01])
                #rospy.sleep(.1)
                print("Pick Function init")
                pnp.pick(initial_poses[idx], hd, more_force)

                pnp.place(initial_poses[idx], hd)

                # temporary for trouble shooting, maybe not even use while loop, not really necessary, its from legacy code
                pnp.gripper_open()
                break
            elif input == "n":
                continue
            elif input == "a":
                new_input = raw_input("What do you want to move? [x+, x-, y+, y-, z+, z-]\n")
                if new_input == "x+":
                    initial_poses[idx].position.x += 0.01
                elif new_input == "x-":
                    initial_poses[idx].position.x -= 0.01
                elif new_input == "y+":
                    initial_poses[idx].position.y += 0.1
                elif new_input == "y-":
                    initial_poses[idx].position.y -= 0.1
                elif new_input == "z+":
                    hd += 0.1
                elif new_input == "z-":
                    hd -= 0.1
                pnp._approach(initial_poses[idx], hd)
            elif input == "readjust":
                isCentered = False
            elif input == "exit" or input == "quit":
                pnp.gripper_open()
                pnp.tuck('t')
                exit(1)

        # Center Onto Yellow Object
        # P_B = [xp, quat, T_BG, P_baseFrame]
        # P_B = pnp.get_pose()
        # course_Adjust = P_B[3]
        # newX = initial_poses[idx].position.x + course_Adjust[0]
        # newY = initial_poses[idx].position.y + course_Adjust[1]
        # print("pb3", P_B[3])
        # initial_poses[idx].position.x = newX
        # initial_poses[idx].position.y = newY
        # pnp._approach(initial_poses[idx],hd)



        # if range is below 18cm prepare for pick up.  This can change, if its too low it will collide with ball
        # so this offset depends on object being picked up
        # if rangeD>0.18:
        #     hd += -.01
        # else:
        #     gripper_offset=0.12  #10 cm from camera with 2 cm safety factor, this can change later
        #     z2pick=gripper_offset-rangeD
        #     #z2pick is negative value so add to z pose below
        #     print initial_poses[idx].position.z,initial_poses[idx].position.z+z2pick
        #     initial_poses[idx].position.z = intial_poses[idx].position.z+z2pick
            # pnp.pick(initial_poses[idx], hd)
        # raw_input("\nValidate offset to not collide with table\nPress Enter to continue...")
    return 0

if __name__ == '__main__':
    main()
