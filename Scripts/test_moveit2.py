import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial",anonymous=True)

robot=moveit_commander.RobotCommander()
scene=moveit_commander.PlanningSceneInterface()
group=moveit_commander.MoveGroupCommander("left_arm")
display_trajectory=rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=10)

rospy.sleep(10)

pose_target=geometry_msgs.msg.Pose()
pose_target.orientation.w=1.0
pose_target.position.x=0.96
pose_target.position.y=0
pose_target.position.z=1.18

group.set_named_target("start")

plan1=group.plan()

rospy.sleep(5)

moveit_commander.roscpp_shutdown()
