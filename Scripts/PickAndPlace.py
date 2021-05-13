import rospy
import baxter_interface
import cv_bridge
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import struct
import copy
import cv2
import numpy as np
import imutils
from imutils import perspective
from imutils import contours
import math
from scipy.spatial import distance as dist
from transformations import quaternion_from_euler, euler_from_quaternion
from tf.transformations import *

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

from sklearn import cluster

bridge = cv_bridge.CvBridge()


params = cv2.SimpleBlobDetector_Params()

params.filterByInertia
params.minInertiaRatio = 0.6  # this ratio is how circular the blobs are (the die dots). Higher inertia means more circular

detector = cv2.SimpleBlobDetector_create(params)


class Die:
    x = int()
    y = int()
    value = int()

    def __init__(self, value, x, y):
        self.value = value
        self.x = x
        self.y = y


def get_blobs(frame):
    frame_blurred = cv2.medianBlur(frame, 7)
    frame_gray = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2GRAY)
    blobs = detector.detect(frame_gray)
    return blobs


def get_dice_from_blobs(blobs):
    # Get centroids of all blobs
    X = []
    for b in blobs:
        pos = b.pt

        if pos != None:
            X.append(pos)

    X = np.asarray(X)

    if len(X) > 0:
        # eps = radius of blobs together that make a die
        # min_sample to 0, as a dice may only have one dot

        clustering = cluster.DBSCAN(eps=60, min_samples=0).fit(X)

        # Find the largest label assigned + 1, that's the number of dice found
        num_dice = max(clustering.labels_) + 1

        dice = []

        # Calculate centroid of each dice, the average between all a dice's dots
        for i in range(num_dice):
            X_dice = X[clustering.labels_ == i]

            centroid_dice = np.mean(X_dice, axis=0)
            dice.append([len(X_dice), centroid_dice[0], centroid_dice[1]])

        return dice

    else:
        return []

def overlay_info(frame, dice, blobs):
    die = None
    # Overlay blobs
    for b in blobs:
        pos = b.pt
        r = b.size / 2

        cv2.circle(frame, (int(pos[0]), int(pos[1])),
                   int(r), (255, 0, 0), 2)
    # Overlay dice number

    # iterate through dice
    for d in dice:
        textsize = cv2.getTextSize(str(d[0]), cv2.FONT_HERSHEY_PLAIN, 3, 2)[0]
        cv2.putText(frame, str(d[0]), (int(d[1] - textsize[0] / 2), int(d[2] + textsize[1] / 2)), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2)
        die = Die(d[0], d[1], d[2])

    return die
    # for key in dice:
    #     textsize = cv2.getTextSize(str(dice.get(key).value), cv2.FONT_HERSHEY_PLAIN, 3, 2)[0]
    #     cv2.putText(frame, str(dice.get(key).value), (int(dice.get(key).x - textsize[0] / 2), int(dice.get(key).y + textsize[1] / 2)), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2)
    #     cv2.putText(frame, str("Dice: " + str(key)), (int(dice.get(key).x), int(dice.get(key).y)), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

class PickAndPlace:
    # instead of global variables, made them object variables of pick and place
    xbar = int()
    ybar = int()
    dices = {}  # keeps track of the dice and their respective information {"1": Dice obj}
    die = None

    v1 = [None] * 2
    v4 = [None] * 2
    #v1 = np.empty(2)
    #v4 = np.empty(2)

    adjust_high_X = False
    adjust_low_X = False
    adjust_high_Y = False
    adjust_low_Y = False

    rangeD = float()

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

    # copy of pick, modified to release gripper prior to approach
    def roll_die(self, pose, hd, grip_control_param):
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
        self.gripper_open()
        self._retract(hd)


    def place(self, pose, hd):
        # servo above pose
        self._approach(pose, hd)
        # servo to pose
        #self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract(hd)

    def image_callback(self, ros_img):
        # Convert received image message to OpenCv image
        cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
        output = cv_image.copy()
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) #not sure why duplicate, can probably just assign 2nd to 1st var
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # yellow
        #lower = np.array([22, 93, 0], dtype="uint8")
        #upper = np.array([45, 255, 255], dtype="uint8")

        blobs = get_blobs(cv_image)
        dice = get_dice_from_blobs(blobs)

        # clear dices
        # self.dices.clear()
        # for index, d in enumerate(dice):
        #     self.dices[index] = Die(d[0]0, d[1], d[2])

        self.die = overlay_info(cv_image, dice, blobs)

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

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)  #also try 7,7
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        edged = cv2.Canny(blurred, 50, 100)  #for vertices test
        edged = cv2.dilate(edged, None, iterations=1)   #for vertices test
        edged = cv2.erode(edged, None, iterations=1)    #for vertices test
        #cnts3 = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)   #for vertices test
        #cnts3 = imutils.grab_contours(cnts) #for vertices test

        cnts2 = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = imutils.grab_contours(cnts2)
        cnt2 = cnts2[0]
        M2 = cv2.moments(cnt2)

        #(cnts2, _) = contours.sort_contours(cnts2)        #for vertices test
        colors = ((0, 0, 255), (240, 0, 159), (255, 0, 0), (255, 255, 0))       #for vertices test red purp blue cyan

        #inside or outside function? seems to work better inside
        """
        if M2["m00"] > 0:
            xbar = int((M2["m10"] / M2["m00"]) * ratio)
            ybar = int((M2["m01"] / M2["m00"]) * ratio)
            # print("shape centroid", cX,cY)
            # cX = int((M2["m10"] / M2["m00"]))  #for full size
            # cY = int((M2["m01"] / M2["m00"]))  #for full size
            cv2.putText(hsv_image, shape, (xbar, ybar), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        #"""

        for (i, c) in enumerate(cnts2):
            reticle = [320, 200]  # reticle centered location
            cv2.rectangle(hsv_image, (318, 198), (322, 202), (0, 0, 255), 1)
            # compute the center of the contour, then detect the name of the shape using only the contour
            M2 = cv2.moments(c)

            # if the contour is not sufficiently large, ignore it - #for vertices test
            if cv2.contourArea(c) < 50:
                continue

            shape = self.detect(c)
            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")      #reduced size
            c *= ratio             #reduced size
            c = c.astype("int")    #reduced size
            cv2.drawContours(hsv_image, [c], -1, (0, 255, 0), 2)   #reduced size
            #cv2.drawContours(hsv_image, c, -1, (0,255,0), 3)       #for full size

            box = cv2.minAreaRect(c)    #for vertices test
            box = cv2.boxPoints(box) #for vertices test
            box = np.array(box, dtype="int")   #for vertices test

            rect = self.order_points(box)    #for vertices test
            #print(rect.astype("int"))   #for vertices test
            #print("")   #for vertices test
            for ((x, y), color) in zip(rect, colors):   #for vertices test
                cv2.circle(hsv_image, (int(x), int(y)), 5, color, -1)   #for vertices test
            cv2.putText(hsv_image, "Object #{}".format(i + 1), (int(rect[0][0] - 15), int(rect[0][1] - 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

              #what are benefits to being here instead of outside the loop?  may not matter due to being called by subsciber so often
            #"""
            if M2["m00"] > 0:
                self.xbar = int((M2["m10"] / M2["m00"]) * ratio)
                self.ybar = int((M2["m01"] / M2["m00"]) * ratio)
                #print("shape centroid", cX,cY)
                #xbar = int((M2["m10"] / M2["m00"]))  #for full size
                #ybar = int((M2["m01"] / M2["m00"]))  #for full size
                cv2.putText(hsv_image, shape, (self.xbar, self.ybar), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            vert1 = rect[0]
            vert4 = rect[3]
            self.v1 = [vert1[0], vert1[1]]
            self.v4 = [vert4[0], vert4[1]]
            #print self.v1[0],self.v4


            if self.xbar > reticle[0]:
                self.adjust_high_X = True
                self.adjust_low_X = False

            if self.xbar < reticle[0]:
                self.adjust_high_X = False
                self.adjust_low_X = True

            if self.ybar > reticle[1]:
                self.adjust_high_Y = True
                self.adjust_low_Y = False

            if self.ybar < reticle[1]:
                self.adjust_high_Y = False
                self.adjust_low_Y = True

        #############end test shape detection################


        #cv2.imshow("output", np.hstack([cv_image, output]))
        #cv2.imshow("output", np.hstack([cv_image, hsv_image]))

        #need this b/c images are of different dimensions, or fix that and use stack

        # Trying to locate checkerboard
        # ret, corners = cv2.findChessboardCorners(gray, (8, 5), None)
        # # If found, add object points, image points (after refining them)
        # if ret == True:
        #     objpoints.append(objp)
        #     corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        #     imgpoints.append(corners)
        #     # Draw and display the corners
        #     cv2.drawChessboardCorners(cv_image, (8, 5), corners2, ret)
        #     cv2.imshow('img', cv_image)
        #     cv2.waitKey(500)

        cv2.imshow('Shape Detection', hsv_image)  # display shape
        cv2.imshow('Color Detection', cv_image)  # display color


        cv2.waitKey(1)
    #def find_ang2rot(self, x1, x2):

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

    def order_points(self, pts):
        #test funtion to find vertices - https://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
        # sort the points based on their x-coordinates
        xSorted = pts[np.argsort(pts[:, 0]), :]
        ySorted = pts[np.argsort(pts[:, 1]), :]
        #print ySorted
        # grab the left-most and right-most points from the sorted
        # x-roodinate points
        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]
        topMost = ySorted[2:, :]
        # now, sort the left-most coordinates according to their
        # y-coordinates so we can grab the top-left and bottom-left
        # points, respectively
        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        (tl, bl) = leftMost
        # now that we have the top-left coordinate, use it as an
        # anchor to calculate the Euclidean distance between the
        # top-left and right-most points; by the Pythagorean
        # theorem, the point with the largest distance will be
        # our bottom-right point
        D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
        (br, tr) = rightMost[np.argsort(D)[::-1], :]
        # return the coordinates in top-left, top-right,
        # bottom-right, and bottom-left order
        #principal axis - below only works for rectangle so far
        rect = np.array([tl, tr, br, bl], dtype="float32")

        return rect

    def get_distance(self, irdist):
        self.rangeD = irdist.range
        #if rangeD > 0.4:
            #print("Warning, Range is out of bounds, move closer")
        # is there better way than using global to return function value, I cant get return working right in main due to being called from ros_subscriber

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

        R_BG = T_BG[:3, :3] #   vector

        vert4 = self.v4
        vert1 = self.v1
        if None in vert1:
            QM_adjust = None
            pass
        else:
            tri_1x = -vert4[0] + vert1[0]  # opp signs - check frames
            tri_1y = vert4[1] - vert1[1]

            #print v1, v4
            #print tri_1x, tri_1y
            gripper_phi = math.pi - math.atan2(tri_1y, tri_1x)
            g_adj_o = quaternion_about_axis(gripper_phi, [0, 0, 1])
            QM_adjust = self.quaternion_multiply(current_quaternion, g_adj_o)
            QM_adjust = QM_adjust / np.linalg.norm(QM_adjust)
            QM_adjust = Quaternion(QM_adjust[0].item(), QM_adjust[1].item(), QM_adjust[2].item(), QM_adjust[3].item())
            print math.degrees(gripper_phi)


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

        QM = self.quaternion_multiply(current_quaternion, new_orientation)
        new_quaternion = QM/np.linalg.norm(QM)
        new_quaternion = Quaternion(new_quaternion[0].item(), new_quaternion[1].item(), new_quaternion[2].item(), new_quaternion[3].item())

        # find vector 2 target object
        conv = self.convert_image_pixel_to_cm()
        reticle = [320, 200]
        dist2obj = np.subtract([self.xbar, self.ybar], reticle)
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

        return xp, current_quaternion, new_quaternion, P_baseFrame, P_baseFrame_offset, P_baseFrame_calibrate_offset, QM_adjust

    def convert_image_pixel_to_cm(self):
        # linear interpolated function based on range data, converts pixels to [cm] width and height
        # need averages for width b/c camera is skewed top and bottom,  top dimension is slightly larger than bottom
        # found this using MATLAB, can do in python but don't have time atm
        W = 151.522 * self.rangeD + 3.4151
        H = 88.3429 * self.rangeD + 2.8822
        CW = 640
        CH = 400
        cm_pixel_ratio = [W / CW, H / CH]
        return cm_pixel_ratio

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