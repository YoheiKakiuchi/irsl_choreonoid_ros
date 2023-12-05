# jupyter console --kernel=choreonoid
### sample gripper server
### ROS
import rospy
rospy.init_node('right_gserver')

### IRSL
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())
from irsl_choreonoid_ros.RobotInterface import GripperAction

def passthrough(inp):
    return inp

def position_to_positions(inp):
    return [inp]*2

r_ga = GripperAction('/kohaku_righthand/gripper_cmd', '/kohaku_righthand/follow_joint_trajectory', [ 'right_l_finger_joint', 'right_r_finger_joint' ],
                     position_to_positions=position_to_positions, effort_to_duration=passthrough)

#l_ga = GripperAction('/left_gripper', '/kohaku_lefthand/follow_joint_trajectory', [ 'left_l_finger_joint', 'left_r_finger_joint' ],
#                     position_to_positions=position_to_positions, effort_to_duration=passthrough)

rospy.spin()

#import rospy
#import GripperAction
#if __name__ == '__main__':
#    rospy.init_node('fibonacci')
#    rospy.get_param('~joint_names')
#    rospy.get_param('~func_positions')
#    rospy.get_param('~func_positions')
#    server = GripperAction(rospay.resolve_name('gripper_action'),
#                           rospay.resolvename('trajectory_action'),
#                           )
#    rospy.spin()
