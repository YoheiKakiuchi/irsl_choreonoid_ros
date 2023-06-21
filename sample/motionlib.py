import rospy
from irsl_choreonoid_ros.RobotInterface import RobotInterface

class MotionLib(object):
  def __init__(self):
    self.ri = RobotInterface('fullset_robot_robotinterface.yaml')
    self.stop_count = 0
  def stop_func(self, **agrs):
    while True:
      self.ri.move_velocity(0, 0, 0)
      rospy.sleep(0.2)
      tm, tof_data = self.ri.waitNextData('TOFsensor', timeout=1.0, clear=True)
      if tof_data.data < 0.25:
        return "stop_back"
      if tof_data.data > 0.5:
        return "stop_forward"
      self.stop_count += 1
      if self.stop_count >= 10:
        return "stop_end"
    return "fail"
  def forward_func(self, **agrs):
    while True:
      self.ri.move_velocity(0, -0.05, 0)
      rospy.sleep(0.2)
      tm, tof_data = self.ri.waitNextData('TOFsensor', timeout=1.0, clear=True)
      if tof_data.data < 0.25:
        return "forward_back"
      if tof_data.data < 0.5:
        return "forward_stop"
    return "fail"
  def back_func(self, **agrs):
    while True:
      self.ri.move_velocity(0, 0.05, 0)
      rospy.sleep(0.2)
      tm, tof_data = self.ri.waitNextData('TOFsensor', timeout=1.0, clear=True)
      if tof_data.data > 0.25:
        return "back_stop"
      if tof_data.data > 0.5:
        return "back_forward"
    return "fail"
