import rospy
import baxter_interface
from baxter_pykdl import baxter_kinematics
from tf.transformations import quaternion_matrix


def moveRightArmToPos(pos):
    limb_right.move_to_joint_positions(pos)


if __name__ == "__main__":
    rospy.init_node('baxter_kinematics')
    print '*** Baxter PyKDL Kinematics ***\n'

    limb_right = baxter_interface.Limb('right')
    limb_left = baxter_interface.Limb('left')
    right_gripper = baxter_interface.Gripper('right')
    left_gripper = baxter_interface.Gripper('left')
    head = baxter_interface.Head()

    kin = baxter_kinematics('right')

    print '\n*** Baxter Description ***\n'
    kin.print_robot_description()
    print '\n*** Baxter KDL Chain ***\n'
    kin.print_kdl_chain()
    # FK Position
    print '\n*** Baxter Position FK ***\n'
    print kin.forward_position_kinematics()
    # FK Velocity
    print '\n*** Baxter Velocity FK ***\n'
    kin.forward_velocity_kinematics()
    # IK
    print '\n*** Baxter Position IK ***\n'
    pos = [0.582583, -0.180819, 0.316003]
    rot = [0.03085, 0.9945, 0.0561, 0.0829]

    #pos = [0.7+.064027, 0-0.0259, 0-0.2296]
    #rot = [0.03085, 0.9945, 0.0561, 0.0829]
    #pd_L=kin.inverse_kinematics(pos)
    #print kin.inverse_kinematics(pos)  # position, don't care orientation
    print '\n*** Baxter Pose IK ***\n'

    pd_L=kin.inverse_kinematics(pos, rot)  # position & orientation
    print pd_L
    pd_dict={'right_s0': pd_L[0],'right_s1':pd_L[1],'right_w0':pd_L[2],'right_w1':pd_L[3],'right_w2':pd_L[4],'right_e0':pd_L[5],'right_e1':pd_L[6]}
    print pd_dict




    moveRightArmToPos(pd_dict)
