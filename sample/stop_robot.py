import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist 

class RobotControl:
    def __init__(self):
        rospy.init_node("robot_control")
        rospy.Subscriber("/TOFsensor", Float64, self.tof_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    def tof_callback(self, data):
        self.tof_data = data.data
        print(self.tof_data)
        if self.tof_data < 0.25:
            self.cmd_pub.publish(Twist())
        else:
            dat = Twist()
            dat.linear.y = -0.05
            self.cmd_pub.publish(dat)

if __name__ == '__main__':
    rc = RobotControl()
    rospy.spin()

    