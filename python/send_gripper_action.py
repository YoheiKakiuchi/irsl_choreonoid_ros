### sample gripper server
### ROS
import rospy
rospy.init_node('test', anonymous=True)

### IRSL
exec(open('/choreonoid_ws/install/share/irsl_choreonoid/sample/irsl_import.py').read())

from irsl_choreonoid_ros.RobotInterface import GripperInterface
from irsl_choreonoid_ros.RobotInterface import GripperAction

gi = GripperInterface('/kohaku_righthand/gripper_cmd')

while not gi.connected:
    rospy.sleep(1.0)
print('find server')

gi.sendCommand(1.0)
print('command sent')

gi.waitUntilFinish()
print('finish command')
