#!/usr/bin/env python3
import rospy
import smach_ros

import smach_definition
import motionlib

if __name__ == '__main__':
    rospy.init_node('smach_ros_demo')

    mlib = motionlib.MotionLib()
    sm = smach_definition.create_state_machine(lib=mlib)
    ## ROS(smach-viewer)
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    rospy.sleep(1)

    result = sm.execute()

    rospy.loginfo('result: {}'.format(result))

    rospy.spin()
    sis.stop()
