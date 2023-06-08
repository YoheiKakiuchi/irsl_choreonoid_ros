import yaml
import sys
import os

# ROS
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
## TODO: action

# choreonoid
import cnoid.Body

# irsl
import cnoid.IRSLUtil as iu
import irsl_choreonoid.cnoid_util as cu
import irsl_choreonoid.robot_util as ru
from .cnoid_ros_util import parseURLROS

class MobileBaseInterface(object):
    def __init__(self, mobile_dict, body=None):
        self.body = body
        if not 'type' in mobile_dict:
            from geometry_msgs.msg import Twist
            self.msg = Twist
        else:
            tp = mobile_dict['type'].split('/')
            exec('from {}.msg import {}'.format(tp[0], tp[1]), locals(), globals())
            self.msg = eval('{}'.format(tp[1]))

        # self.msg
        self.pub = rospy.Publisher('{}'.format(mobile_dict['topic']), self.msg, queue_size=1)
        self.baselink = None
        if 'baselink' in mobile_dict:
            self.baselink = mobile_dict['baselink']

    def stop(self):
        self.move_velocity(0.0, 0.0, 0.0)

    def move_velocity(self, vel_x, vel_y, vel_th):
        msg = self.msg()
        msg.linear.x = vel_x
        msg.linear.y = vel_y
        msg.angular.y = vel_th
        self.pub.publish(msg)

    def move_position(self):
        pass

    def move_on_map(self, pos):
        pass

    def move_trajectory(self, traj):
        pass

class JointGroupTopic(object):
    def __init__(self, group, body=None):
        self.body = body
        self.pub = rospy.Publisher(group['topic'], JointTrajectory, queue_size=1)
        self.joint_names = group['joint_names']
        self.joints  = []
        for j in self.joint_names:
            j = body.joint(j)
            self.joints.append(j)

        self.finish_time = rospy.get_rostime()

    def sendAngles(self, tm = None):
        if tm is None:
            ### TODO: do not use hard coded number
            tm = 4.0
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [j.q for j in self.joints]
        point.time_from_start = rospy.Duration(tm)
        msg.points.append(point)
        self.finish_time = rospy.get_rostime() + rospy.Duration(tm)
        self.pub.publish(msg)

    def isFinished(self):
        diff = (rospy.get_rostime() - self.finish_time).to_sec()
        if diff > 0:
            return True
        else:
            return False

class JointGroupAction(object):
    def __init__(self, group, body=None):
        self.body = body
        print('JointGroupAction not implemented', file=sys.stderr)
        raise Exception
    def sendAngles(self, tm = None):
        pass
    def isFinished(self):
        return False

class JointInterface(object):
    def __init__(self, group_list, body=None):
        self.body = body
        self.joint_groups = {}
        self.default_group = None
        for group in group_list:
            if not 'name' in group:
                name = 'default'
            else:
                name = group['name']
            if ('type' in group) and (group['type'] == 'action'):
                jg = JointGroupAction(group, body)
            else:
                jg = JointGroupTopic(group, body)
            self.joint_groups[name] = jg
            if self.default_group is None:
                self.default_group = jg

    def sendAngles(self, tm=None, group=None):
        if group is None:
            gp = self.default_group
        else:
            gp = self.joint_groups[group]
        gp.sendAngles(tm)

    def sendAngleVector(self, av, tm=None, group=None):
        self.body.angleVector(av)
        self.sendAngles(tm=tm,group=group)

    def sendAngleDict(self, angle_dict, tm):
        for name, angle in angle_dict.items():
            self.body.joint(name).q = angle
        self.sendAngles(tm=tm,group=group)

    def isFinished(self, group = None):
        if group is None:
            gp = self.default_group
        else:
            gp = self.joint_groups[group]
        return gp.isFinished()

class RosDevice(object):
    def __init__(self, dev_dict, body=None):
        self.body = body
        self.topic = dev_dict['topic']
        self.name  = dev_dict['name']
        ## TODO: search device in body
        tp = dev_dict['type'].split('/')
        exec('from {}.msg import {}'.format(tp[0], tp[1]), locals(), globals())
        self.msg = eval('{}'.format(tp[1]))

        self.sub = rospy.Subscriber(self.topic, self.msg, self.callback)

    def callback(self, msg):
        self.current_msg = msg
        ## TODO: update robot-model

class DeviceInterface(object):
    def __init__(self, device_list, body=None):
        self.body = body
        self.devices = {}

        for dev in device_list:
            self.devices[dev['name']] = RosDevice(dev)

    def data(self, name):
        dev = self.devices[name]
        return dev.current_msg

class RobotInterface(object):
    def __init__(self, fname, name='robot_interface', anonymous=False):
        rospy.init_node(name, anonymous=anonymous)

        with open(parseURLROS(fname)) as f:
            self.info = yaml.safe_load(f)

        self.mobile_base = None
        self.joints = None
        self.devices = None

        self.load_robot()
        self.setup_modele_base()
        self.setup_joint_groups()
        self.setup_devices()

    def load_robot(self):
        mdl = self.info['robot_model']
        self.robot_name = mdl['name']
        self.model_file = parseURLROS(mdl['url'])

        rospy.loginfo('loading model from {}'.format(self.model_file))

        if not os.path.isfile(self.model_file):
            raise Exception('file: {} does not exist'.format(self.model_file))

        bl = cnoid.Body.BodyLoader()
        self.body = bl.load(self.model_file)
        if self.body is None:
            raise Exception('body can not be loaded by file: {}'.format(self.model_file))
        ## RobotModel??

    def copyRobotModel(self):
        bl = cnoid.Body.BodyLoader()
        return bl.load(self.model_file)

    def setup_modele_base(self):
        if not 'mobile_base' in self.info:
            return

        self.mobile_base = MobileBaseInterface(self.info['mobile_base'], body=self.body)

    def setup_joint_groups(self):
        if not 'joint_groups' in self.info:
            return

        self.joint_groups = JointInterface(self.info['joint_groups'], body=self.body)

    def setup_devices(self):
        if not 'devices' in self.info:
            return

        self.devices = DeviceInterface(self.info['devices'], body=self.body)

# from irsl_choreonoid_ros.RobotInterface import RobotInterface
# ri = RobotInterface('robot_interface.yaml')
# robot_model = ri.copyRobotModel()
# ri.mobile_base.move_velocity(1, 0, 0)
# av = robot_model.angleVector()
# ri.joint_groups.sendAngleVector(av, 2.0)
# data = ri.devices.data('TOF_sensor0')
