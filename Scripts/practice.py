#! /usr/bin/python
import rospy
from geometry_msgs.msg import TransformStamped

class vals:
    ball_x = TransformStamped()
    ball_y = TransformStamped()
    ball_z = TransformStamped()

def ball_cb(msg):
    vals.ball_x = msg.transform.translation.x
    vals.ball_y = msg.transform.translation.y
    vals.ball_z = msg.transform.translation.z

def main():

    while not rospy.is_shutdown():

        x = vals.ball_x
        y = vals.ball_y
        z = vals.ball_z


        print(x, y, z)

        rate.sleep()

if __name__ == "__main__":

    rospy.init_node('practice', anonymous=False)
    rospy.Subscriber('/vicon/red_ball/red_ball',TransformStamped, ball_cb, queue_size=10)
    rospy.Publisher('')
    rate = rospy.Rate(10)

    main()
