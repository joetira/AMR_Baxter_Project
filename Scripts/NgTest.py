import subprocess
import rospy
import baxter_interface
import os
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image

bridge = cv_bridge.CvBridge()


right_s0 = 0.25502430598595005
right_s1 = -0.9767622666860372
right_w0 = -0.69757776329089190
right_w1 = 0.8597962316097744
right_w2 = 0.6761020322604961
right_e0 = 1.0845244170349875
right_e1 = 1.6367575006737365


def getRightAngleDict():
	temp = {	
			"right_s0" : right_s0,
			"right_s1" : right_s1,
			"right_w0" : right_w0,
			"right_w1" : right_w1,
			"right_w2" : right_w2,
			"right_e0" : right_e0,
			"right_e1" : right_e1
		}
	return temp

# s should be:
# u : untuck
# t : tuck
def tuck(s):
	os.system("python tuck_arms.py -" + s)

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
	



def armsout():
	# initialize our ROS node, registering it with the Master
	rospy.init_node('Zero_Arms')

	# store the zero position of the arms
	zero_zero_right = {'right_s0': 0.0, 'right_s1': 0.00, 'right_w0': 0.00, 'right_w1': 0.00, 'right_w2': 0.00, 'right_e0': 0.00, 'right_e1': 0.00}
	zero_zero_left = {'left_s0': 0.0, 'left_s1': 0.00, 'left_w0': 0.00, 'left_w1': 0.00, 'left_w2': 0.00, 'left_e0': 0.00, 'left_e1': 0.00}

	# move both arms to zero position
	limb_right.move_to_joint_positions(zero_zero_right)
	limb_left.move_to_joint_positions(zero_zero_left)


	right_gripper.close()
	left_gripper.close()

def image_callback(ros_img):
	# Convert received image message to OpenCv image
	cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
	hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	lower = np.array([22, 93, 0], dtype="uint8")
	upper = np.array([45, 255, 255], dtype="uint8")


	mask = cv2.inRange(hsv_image, lower, upper)
	
	cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if len(cnts) == 2 else cnts[1]

	for c in cnts:
		x,y,w,h = cv2.boundingRect(c)
		cv2.rectangle(cv_image, (x,y), (x+w, y+h), (36,255,12), 2)
		centroid_x, centroid_y = x+w/2, y+h/2
		print(centroid_x, centroid_y)
		if (centroid_x > 316+5):
			print("X is too HIGH", 316-centroid_x)
			global right_w1
			right_w1 += -0.001
			moveRightArmToPos(getRightAngleDict())
		if (centroid_y > 205+5 or centroid_y < 205-5):
			print("Y is not centered", 205-centroid_y)


#	cv2.namedWindow('image', cv2.WINDOW_NORMAL)
#	cv2.resizeWindow('image', 1280, 800)
	# cv2.imshow('mask', mask)
	cv2.imshow('Image', cv_image) # display image
	cv2.waitKey(1)

def moveRightArmToPos(pos):
	limb_right.move_to_joint_positions(pos)



if __name__ == '__main__':
	# Initialize Baxter
	# initializeCameras("head")

	rospy.init_node('Test')
	limb_right = baxter_interface.Limb('right')
	limb_left = baxter_interface.Limb('left')
	right_gripper = baxter_interface.Gripper('right')
	left_gripper = baxter_interface.Gripper('left')
	head = baxter_interface.Head()

	# untuck arms
	tuck('u')

	# limb_right.move_to_neutral()

	moveRightArmToPos(getRightAngleDict())
	#armsout()

	

	 # Subscribe to head_camera image topic
 	rospy.Subscriber('/cameras/right_hand_camera/image', Image, image_callback)
 	rospy.spin()
 	cv2.destroyAllWindows() # Destroy CV image window on shut_down

	

	tuck('t')
	print("FINISHED")
	quit()
