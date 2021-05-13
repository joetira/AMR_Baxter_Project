def main():
    import sys
    def usage():
        print("Tests for kdl_parser:\n")
        print("kdl_parser <urdf file>")
        print("\tLoad the URDF from file.")
        print("kdl_parser")
        print("\tLoad the URDF from the parameter server.")
        sys.exit(1)

    if len(sys.argv) > 2:
        usage()
    if len(sys.argv) == 2 and (sys.argv[1] == "-h" or sys.argv[1] == "--help"):
        usage()
    if (len(sys.argv) == 1):
        robot = URDF.load_from_parameter_server(verbose=False)
    else:
        robot = URDF.load_xml_file(sys.argv[1], verbose=False)
    tree = kdl_tree_from_urdf_model(robot)
    num_non_fixed_joints = 0
    for j in robot.joints:
        if robot.joints[j].joint_type != 'fixed':
            num_non_fixed_joints += 1
    print "URDF non-fixed joints: %d;" % num_non_fixed_joints,
    print "KDL joints: %d" % tree.getNrOfJoints()
    print "URDF joints: %d; KDL segments: %d" %(len(robot.joints),
                                                tree.getNrofSegments())
    import random
    base_link = robot.get_root()
    end_link = robot.links.keys()[random.randint(0, len(robot.links)-1)]
    chain = tree.getChain(base_link, end_link)
    print "Root link: %s; Random end link: %s" % (base_link, end_link)
    for i in range(chain.getNrOfSegments()):
        print chain.getSegment(i).getName()