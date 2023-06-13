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
#from irsl_choreonoid_ros.cnoid_ros_util import parseURLROS

#
# MobileBaseInterface
#
class MobileBaseInterface(object):
    def __init__(self, info, body=None, **kwargs):
        if 'mobile_base' in info:
            self.mobile_init(info['mobile_base'], body)
    def mobile_init(self, mobile_dict, body):
        print('mobile: {}'.format(mobile_dict))
        if body is not None:
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
            ## TODO check baselink in self.body
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

#
# JointInterface
#
class JointInterface(object):
    def __init__(self, info, body=None, **kwargs):
        if 'joint_groups' in info:
            self.joint_init(info['joint_groups'], body)
    def joint_init(self, group_list, body):
        print('joint: {}'.format(group_list))
        if body is not None:
            self.body = body
        self.joint_groups = {}
        self.default_group = None
        for group in group_list:
            if not 'name' in group:
                name = 'default'
            else:
                name = group['name']
            if ('type' in group) and (group['type'] == 'action'):
                jg = JointGroupAction(group, self.body)
            else:
                jg = JointGroupTopic(group, self.body)
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

class JointGroupTopic(object):
    def __init__(self, group, body=None):
        super().__init__()
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
        super().__init__()
        self.body = body
        print('JointGroupAction not implemented', file=sys.stderr)
        raise Exception
    def sendAngles(self, tm = None):
        pass
    def isFinished(self):
        return False

#
# DeviceInterface
#
class DeviceInterface(object):
    def __init__(self, info, body=None, **kwargs):
        if 'devices' in info:
            self.device_init(info['devices'], body)
    def device_init(self, device_list, body):
        print('devices: {}'.format(device_list))
        if body is not None:
            self.body = body
        self.devices = {}

        for dev in device_list:
            if 'class' in dev:
                cls = eval('{}'.format(dev['class']))
                self.devices[dev['name']] = cls(dev, body=self.body)
            else:
                self.devices[dev['name']] = RosDevice(dev, body=self.body)

    def data(self, name, clear=False):
        '''Get data from the device'''
        dev = self.devices[name]
        return dev.data(clear)
    def waitData(self, name, timeout=None, clear=False):
        '''Wait if there is no current data'''
        dev = self.devices[name]
        return dev.waitData(timeout, clear=clear)
    def waitNextData(self, name, timeout=None, clear=False):
        '''Wait until subscribing new data'''
        dev = self.devices[name]
        return dev.waitNextData(timeout, clear=clear)
    def dataArray(self, names, clear=False):
        '''Get data array from devices'''
        return [ self.devices[name].data(clear) for name in names ]
    def waitDataArray(self, names, timeout=None, clear=False):
        for name in names:
            self.devices[name]._pre_wait(timeout)
        return [ self.devices[name]._fetch_data(clear) for name in names ]
    def waitNextDataArray(self, names, timeout=None, clear=False):
        for name in names:
            self.devices[name]._pre_wait_next(timeout)
        return [ self.devices[name]._fetch_data(clear) for name in names ]

class RosDeviceBase(object):
    def __init__(self, dev_dict, body=None):
        super().__init__()
        self.robot_callback = None
        self.body = body
        self.topic = dev_dict['topic']
        self.name  = dev_dict['name']
        ## TODO: implement rate
        if 'rate' in dev_dict:
            self.rate = dev_dict['rate']
        else:
            self.rate = None
        ## TODO: search device in body
        self.msg_time = None
        self.current_msg = None
        self.timeout = None
    def parseType(self, type_str):
        tp = type_str.split('/')
        exec('from {}.msg import {}'.format(tp[0], tp[1]), locals(), globals())
        self.msg = eval('{}'.format(tp[1]))
    def subscribe(self):
        self.sub = rospy.Subscriber(self.topic, self.msg, self.callback)

    def callback(self, msg):
        if hasattr(msg, 'header'):
            self.msg_time = msg.header.stamp
        else:
            self.msg_time = rospy.get_rostime()
        if self.robot_callback is not None:
            self.robot_callback(self.msg_time, msg)
        self.current_msg = msg
    def returnData(self):
        return (self.msg_time, self.current_msg)
    def data(self, clear=False):
        res = self.returnData()
        if clear:
            self.msg_time = None
            self.current_msg = None
        return res
    ## protected functions
    def _pre_wait(self, timeout):
        if timeout is None:
            self.timeout = None
        else:
            self.timeout = rospy.get_rostime() + rospy.Duration.from_sec(timeout)

    def _pre_wait_next(self, timeout):
        self._pre_wait(timeout)
        self.msg_time = None
        self.current_msg = None

    def _fetch_data(self, clear=False):
        while ( self.timeout is None ) or ( self.timeout >= rospy.get_rostime() ):
            if self.current_msg is not None:
                return self.data(clear)
            else:
                rospy.sleep(0.002)
    ##
    def waitData(self, timeout=None, clear=False):
        self._pre_wait(timeout)
        return self._fetch_data(clear)
    def waitNextData(self, timeout=None, clear=False):
        self._pre_wait_next(timeout)
        return self._fetch_data(clear)
## generic device (using type: tag in robotinterface.yaml)
class RosDevice(RosDeviceBase):
    def __init__(self, dev_dict, body=None):
        super().__init__(dev_dict, body)
        self.parseType(dev_dict['type'])
        self.subscribe()
## specific device (using class: tag in robotinterface.yaml)
class StringDevice(RosDeviceBase):
    def __init__(self, dev_dict, body=None):
        super().__init__(dev_dict, body)
        import std_msgs.msg
        self.msg = std_msgs.msg.String
        self.subscribe()
    def returnData(self):
        if self.current_msg is None:
            return (None, None)
        else:
            return (self.msg_time, self.current_msg.data)
class JointState(RosDeviceBase):
    def __init__(self, dev_dict, body=None):
        super().__init__(dev_dict, body)
        import sensor_msgs.msg
        self.msg = sensor_msgs.msg.JointState
        self.robot_callback = self.joint_callback
        self.subscribe()
    def joint_msg_to_body(self, msg):
        for idx, nm in zip(range(len(msg.name)), msg.name):
            lk = self.body.joint(nm)
            if lk:
                lk.q = msg.position[idx]
    def joint_callback(self, rtime, msg):
        #print('js: {} {}'.format(rtime, msg))
        self.joint_msg_to_body(msg)
#
# RobotInterface
#
class RobotInterface(JointInterface, DeviceInterface, MobileBaseInterface):
    def __init__(self, fname, name='robot_interface', anonymous=False):
        rospy.init_node(name, anonymous=anonymous)

        with open(parseURLROS(fname)) as f:
            self.info = yaml.safe_load(f)

        self.load_robot()
        JointInterface.__init__(self, self.info)
        DeviceInterface.__init__(self, self.info)
        MobileBaseInterface.__init__(self, self.info)

    def load_robot(self):
        if 'robot_model' in self.info:
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
#    def setup_modele_base(self):
#        if 'mobile_base' in self.info:
#            self.mobile_base = MobileBaseInterface(self.info['mobile_base'], body=self.body)
#    def setup_joint_groups(self):
#        if 'joint_groups' in self.info:
#            self.joint_groups = JointInterface(self.info['joint_groups'], body=self.body)
#    def setup_devices(self):
#        if 'devices' in self.info:
#            self.devices = DeviceInterface(self.info['devices'], body=self.body)

##
## sample usage
##
# from irsl_choreonoid_ros.RobotInterface import RobotInterface
# ri = RobotInterface('robot_interface.yaml')
# robot_model = ri.copyRobotModel()
# ri.move_velocity(1, 0, 0)
# av = robot_model.angleVector()
# ri.sendAngleVector(av, 2.0)
# data = ri.data('TOF_sensor0')
