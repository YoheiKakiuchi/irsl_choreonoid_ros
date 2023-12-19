import yaml
import sys
import os

# ROS
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Header as std_msgs_header

# action
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from control_msgs.msg import GripperCommandFeedback
from control_msgs.msg import GripperCommandResult
from control_msgs.msg import GripperCommand

# choreonoid
import cnoid.Body

# irsl
from cnoid.IRSLCoords import coordinates
import irsl_choreonoid.cnoid_util as iu
import irsl_choreonoid.robot_util as ru
from .cnoid_ros_util import parseURLROS
#from irsl_choreonoid_ros.cnoid_ros_util import parseURLROS
if iu.isInChoreonoid():
    ## in base
    import irsl_choreonoid.cnoid_base as ib
    import cnoid.Base as cbase

from numpy import array as npa

#
# MobileBaseInterface
#
class MobileBaseInterface(object):
    """Interface for controlling locomotion of the robot
    """
    def __init__(self, info, robot=None, **kwargs):
        self.pub = None
        if 'mobile_base' in info:
            self.__mobile_init(info['mobile_base'], robot)
    def __mobile_init(self, mobile_dict, robot):
        print('mobile: {}'.format(mobile_dict))
        if robot is not None:
            self.instanceOfBody = robot
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
            ## TODO check baselink in self.instanceOfBody

    @property
    def mobile_initialized(self):
        """Initialized check of MobileBase

        Returns:
            boolean : True returns, if MobileBase instance has been initialized

        """
        if self.pub is None:
            return False
        else:
            return True

    @property
    def mobile_connected(self):
        """Connection check of MobileBase

        Returns:
            boolean : True returns, if MobileBase instance has been connected

        """
        if self.pub is not None and self.pub.get_num_connections() > 0:
            return True
        return False

    def waitFinishMoving(self, timeout=1.0):
        """Wait to finish moving of MobileBase

        Args:
            timeout(float, default=1.0) :

        Returns:
            boolean : False returns, if timeout.

        """
        if timeout is not None:
            rospy.sleep(timeout)
        return False

    def stop(self):
        """Stop moving of MobileBase

        Args:
            None

        """
        self.move_velocity(0.0, 0.0, 0.0)

    def move_velocity(self, vel_x, vel_y, vel_th):
        """Set moving velocity of MobileBase

        Args:
            vel_x (float) : Velocity of x-axis [ m/sec ]
            vel_y (float) : Velocity of y-axis [ m/sec ]
            vel_th (float) : Angular velocity of yaw [ radian/sec ]


        """
        msg = self.msg()
        msg.linear.x = vel_x
        msg.linear.y = vel_y
        msg.angular.z = vel_th
        self.pub.publish(msg)

    @property
    def currentMapCoords(self):
        """Current robot's coordinate on the map

        **Not implemented yet**

        Returns:
            cnoid.IRSLCoords.coordinates : Current robot's coordinate on the map


        """
        return coordinates()

    def move_position(self, coords):
        """Set target position for MobileBase, target is reletive to current robot's coordinates

        Args:
            coords (cnoid.IRSLCoords.coordinates) : Target coordinates for moving (reletive to current robot's coordinates)

        """
        pass

    def move_on_map(self, coords):
        """Set target position on map for MobileBase, target is relative to map coordinates

        Args:
            coords (cnoid.IRSLCoords.coordinates) : Target coordinates for moving (map coordinates)

        """
        pass

    def move_trajectory(self, traj, relative = False):
        """Set target trajectory for MobileBase

        **Not implemented yet**

        Args:
            traj (list[(cnoid.IRSLCoords.coordinates, float)]) : List of pair of coordinates and time
            relative (boolean, default = False) : If True, trajectory is considered it is relative to robot's coordinates.

        """
        pass
#
# JointInterface
#
class JointInterface(object):
    """Interface for controlling joints of the robot
    """
    def __init__(self, info, robot=None, **kwargs):
        self.joint_groups = {}
        self.default_group = None
        if 'joint_groups' in info:
            self.__joint_init(info['joint_groups'], robot)
    def __joint_init(self, group_list, robot):
        print('joint: {}'.format(group_list))
        if robot is not None:
            self.instanceOfJointBody = robot
        for group in group_list:
            if not 'name' in group:
                name = 'default'
            else:
                name = group['name']
            if ('type' in group) and (group['type'] == 'action'):
                jg = JointGroupAction(group, name, self.jointRobot)
            elif ('type' in group) and (group['type'] == 'combined'):
                jg = JointGroupCombined(group, name, self.jointRobot)
                jg.setGroups(self.joint_groups)
            else:
                jg = JointGroupTopic(group, name, self.jointRobot)
            self.joint_groups[name] = jg
            if self.default_group is None:
                self.default_group = jg
        if 'default' in self.joint_groups:
            self.default_group = self.joint_groups['default']

    @property
    def joint_initialized(self):
        """Initialized check of JointInterface

        Returns:
            boolean : True returns, if JointInterface instance has been initialized

        """
        if self.default_group is None:
            return False
        else:
            return True

    @property
    def joint_connected(self):
        """Connection check of JointInterface

        Returns:
            boolean : True returns, if JointInterface instance has been connected

        """
        if self.default_group is not None and self.default_group.connected:
            return True
        return False

    @property
    def jointGroupList(self):
        """Getting list of instance of joint-group

        Returns:
            list [ JointGroup ] : List of instance of joint-groups

        """
        return list(self.joint_groups.values())

    @property
    def jointGroupNames(self):
        """Getting list of name of joint-groups

        Returns:
            list [ str ] : List of name of joint-groups

        """
        return list(self.joint_groups.keys())

    def getJointGroup(self, name):
        """Getting a instance of joint-group

        Args:
            name (str) : Name of joint-group

        Returns:
            JointGroupTopic : Instance of joint-group

        """
        if name in self.joint_groups:
            return self.joint_groups[name]
        return None

    def sendAngles(self, tm=None, group=None):
        """Sending angles of self.robot to the actual robot

        Args:
            tm (float) : Moving duration in second
            group (str or list[str], optional): Name(s) of group to be used

        """
        tp = type(group)
        if tp is list or tp is tuple:
            for gp in group:
                gp.sendAngles(tm)
        else:
            if group is None:
                gp = self.default_group
            else:
                gp = self.joint_groups[group]
            gp.sendAngles(tm)

    def sendAngleVector(self, angle_vector, tm=None, group=None, wait=False, waitTimeout=None):
        """Sending angle-vector to the actual robot. angle_vector is set to self.robot

        Args:
            angle_vector (numpy.array) : Vector of angles of whole body
            tm (float) : Moving duration in second
            group (str or list[str], optional): Name(s) of group to be used
            wait (boolean, defaut=False) : Wait until finishing moveving
            waitTimeout (float, optional) : Pass to waitUntilFinish

        """
        self.jointRobot.angleVector(angle_vector)
        self.sendAngles(tm=tm, group=group)
        if wait:
            self.waitUntilFinish(group=group, timeout=waitTimeout)

    def sendAngleVectorSequence(self, angle_vector_list, tm_list, group=None, wait=False, waitTimeout=None):
        """
        Args:
            angle_vector_list ( list [numpy.array] ) : List of vectors of angles
            tm_list ( list[ float ] ) : List of moving durations in second
            group (str or list[str], optional): Name(s) of group to be used
            wait (bolean, default=False) : wait until finishing move
            waitTimeout (float, optional) : Pass to waitUntilFinish

        """
        tp = type(group)
        if tp is not list and tp is not tuple:
            if group is None:
                group = [ self.default_group ]
            else:
                group = [ self.joint_groups[group] ]
        _group=[]
        for g in group:
            if type(g) == JointGroupCombined:
                _group.extend(g.groups)
            else:
                _group.append(g)
        vec_list_group = []
        for i in range(len(_group)):
            vec_list_group.append([])
        for angle_vector in angle_vector_list:
            self.jointRobot.angleVector(angle_vector)
            for idx, gp in enumerate(_group):
                vec_list_group[idx].append(gp.getAngleVector())
        for gp, vec in zip(_group, vec_list_group):
            gp.sendAnglesSequence(vec, tm_list)
        if wait:
            self.waitUntilFinish(group=group, timeout=waitTimeout)

    def sendAngleMap(self, angle_map, tm, group=None):
        """Sending angles to the actual robot. angles is set to self.robot

        Args:
            angle_map ( dict[name, float] ) : Dictionary, whose key is joint-name, and value is joint-angle
            tm (float) : Moving duration in second
            group (str or list[str], optional): Name(s) of group to be used

        """
        for name, angle in angle_map.items():
            self.jointRobot.joint(name).q = angle
        self.sendAngles(tm=tm, group=group)
        if wait:
            self.waitUntilFinish(group=group, timeout=waitTimeout)

    def isFinished(self, group = None):
        """Checking method for finishing to send angles

        Args:
            group (str or list[str], optional): Name(s) of group to be used

        Returns:
               boolean : If True, the robot is not moving

        """
        tp = type(group)
        if tp is list or tp is tuple:
            return all( [ gp.isFinished() for gp in group ] )
        else:
            if group is None:
                gp = self.default_group
            else:
                gp = self.joint_groups[group]
            return gp.isFinished()

    def waitUntilFinish(self, timeout = None, group = None):
        """Waiting until finishing joint moving

        Args:
            timeout (float, optional): Time for timeout
            group (str or list[str], optional): Name(s) of group to be used

        Returns:
               boolean : False returns, if timeout.

        """
        tp = type(group)
        if tp is list or tp is tuple:
            res = []
            for gp in group:
                res.append(gp.waitUntilFinish(timeout))
            return all(res)
        else:
            if group is None:
                gp = self.default_group
            else:
                gp = self.joint_groups[group]
            return gp.waitUntilFinish(timeout)

    def cancel(self):
        pass

class JointGroupBase(object):
    def __init__(self, name, robot=None):
        self._robot = robot
        self._group_name = name

    def setJointNames(self, names):
        self.joint_names = names
        self.joints  = []
        for j in self.joint_names:
            j = self._robot.joint(j)
            if j is None:
                print('JointGroupTopic({}): joint-name: {} is invalid'.format(name, j))
            else:
                self.joints.append(j)

    def getAngleVector(self, whole_angles=None):
        if whole_angles is not None:
            self.robot.angleVector(whole_angles)
        return [j.q for j in self.joints]

    @property
    def name(self):
        return self._group_name

    @property
    def robot(self):
        return self._robot

    @property
    def jointNames(self):
        return [ j.jointName for j in self.joints ]

    @property
    def jointList(self):
        return self.joints

    @property
    def connected(self):
        return True

    def sendAngles(self, tm = None):
        pass

    def sendAnglesSequence(self, vec_list, tm_list):
        pass

    def isFinished(self):
        return True

    def waitUntilFinish(self, timeout=None):
        pass

    def cancel(self):
        pass

class JointGroupTopic(JointGroupBase):
    def __init__(self, group, name, robot=None):
        super().__init__(name, robot)

        self.pub = rospy.Publisher(group['topic'], JointTrajectory, queue_size=1)

        self.setJointNames(group['joint_names'])

        self.finish_time = rospy.get_rostime()

    @property
    def connected(self):  ## override
        if self.pub.get_num_connections() > 0:
            return True
        return False

    def sendAngles(self, tm = None):  ## override
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

    def sendAnglesSequence(self, vec_list, tm_list):  ## override
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        time_ = 0.0
        for vec, tm in zip(vec_list, tm_list):
            point = JointTrajectoryPoint()
            point.positions = vec
            time_ += tm
            point.time_from_start = rospy.Duration(time_)
            msg.points.append(point)
        self.finish_time = rospy.get_rostime() + rospy.Duration(time_)
        self.pub.publish(msg)

    def isFinished(self):  ## override
        diff = (rospy.get_rostime() - self.finish_time).to_sec()
        if diff > 0:
            return True
        else:
            return False

    def waitUntilFinish(self, timeout=None):  ## override
        if timeout is None:
            timeout = 1000000000.0
        st = rospy.get_rostime()
        while (rospy.get_rostime() - st).to_sec() < timeout:
            if self.isFinished():
                break
            rospy.sleep(0.01)

class JointGroupAction(JointGroupBase):
    def __init__(self, group, name, robot=None):
        super().__init__(name, robot)
        self.setJointNames(group['joint_names'])
        ##
        self._client = actionlib.SimpleActionClient(group['topic'],  FollowJointTrajectoryAction)

    @property
    def connected(self):  ## override
        return self._client.wait_for_server(rospy.Duration(0.0001))

    def sendAngles(self, tm=None):  ## override
        if tm is None:
            ### TODO: do not use hard coded number
            tm = 4.0
        header_ = std_msgs_header(stamp=rospy.Time(0))
        _traj = JointTrajectory(header=header_, joint_names=self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = [j.q for j in self.joints]
        point.time_from_start = rospy.Duration(tm)
        _traj.points.append(point)

        _goal = FollowJointTrajectoryGoal(trajectory=_traj, goal_time_tolerance=rospy.Time(0.1))
        self._client.send_goal(_goal)

    def sendAnglesSequence(self, vec_list, tm_list):  ## override
        header_ = std_msgs_header(stamp=rospy.Time(0))
        _traj = JointTrajectory(header=header_, joint_names=self.joint_names)
        time_ = 0.0
        for vec, tm in zip(vec_list, tm_list):
            point = JointTrajectoryPoint()
            point.positions = vec
            time_ += tm
            point.time_from_start = rospy.Duration(time_)
            _traj.points.append(point)
        _goal = FollowJointTrajectoryGoal(trajectory=_traj, goal_time_tolerance=rospy.Time(0.1))
        self._client.send_goal(_goal)

    def isFinished(self):  ## override
        return self._client.simple_state != actionlib.SimpleGoalState.ACTIVE

    def waitUntilFinish(self, timeout=None):  ## override
        if timeout is None:
            timeout = rospy.Duration(1000000000.0)
        else:
            if timeout <= 0.0:
                timeout = 0.001
            timeout = rospy.Duration(timeout)
        self._client.wait_for_result(timeout)

    def cancel(self):
        self._client.cancel_all_goals()

class JointGroupCombined(JointGroupBase):
    def __init__(self, group, name, robot=None):
        super().__init__(name, robot)
        self.group_names = group['groups']

    def setGroups(self, dict_group):
        self.groups = []
        _jnames=[]
        for gn in self.group_names:
            if gn in dict_group:
                gp=dict_group[gn]
                _jnames.extend(gp.jointNames)
                self.groups.append(gp)
            else:
                raise Exception('group name : {} is not defined'.format(gn))
        self.setJointNames(_jnames)

    @property
    def connected(self):  ## override
        return all( [ g.connected for g in self.groups ] )

    def sendAngles(self, tm = None): ## override
        for g in self.groups:
            g.sendAngles(tm)

    def sendAnglesSequence(self, vec_list, tm_list):  ## override
        ## TODO: fix
        for g in self.groups:
            g.sendAnglesSequence(vec_list, tm_list)

    def isFinished(self):  ## override
        return all( [ g.isFinished() for g in self.groups ] )

    def waitUntilFinish(self, timeout=None):  ## override
        for g in self.groups:
            g.waitUntilFinish(timeout)

    def cancel(self): ## override
        for g in self.groups:
            g.cancel()

#
# DeviceInterface
#
class DeviceInterface(object):
    """Interface for receiving data from sensors on the robot
    """
    def __init__(self, info, robot=None, **kwargs):
        self.devices = {}
        if 'devices' in info:
            self.__device_init(info['devices'], robot)
    def __device_init(self, device_list, robot):
        print('devices: {}'.format(device_list))
        if robot is not None:
            self.instanceOfBody = robot
        for dev in device_list:
            if 'class' in dev:
                cls = eval('{}'.format(dev['class']))
                self.devices[dev['name']] = cls(dev, robot=self.robot)
            else:
                self.devices[dev['name']] = RosDevice(dev, robot=self.robot)

    @property
    def device_initialized(self):
        """Initialized check of DeviceInterface

        Returns:
            boolean : True returns, if DeviceInterface instance has been initialized

        """
        if len(self.devices) > 0:
            return True
        return False

    @property
    def device_connected(self):
        """Connection check of DeviceInterface

        Returns:
            boolean : True returns, if all devices in this instance have been connected

        """
        if not self.device_initialized:
            return False
        for dev in self.devices.values():
            if not dev.connected:
                return False
        return True

    @property
    def deviceList(self):
        """Getting list of the instances for gathering data from the device

        Returns:
            list [ RosDeviceBase ] : List of the instances for gathering data

        """
        return list(self.devices.values())

    @property
    def deviceNames(self):
        """Getting list of name of the devices

        Returns:
            list [ str ] : List of name of the devices

        """
        return list(self.devices.keys())

    def getDevice(self, name):
        """Getting the instance for gathering data from the device

        Args:
            name (str) : Name of the device

        Returns:
            RosDeviceBase : The instance witch has the same name as given name

        """
        return self.devices[name]

    def getDevicesByClass(self, cls):
        """Getting the list of device which is a typical class

        Args:
            cls (Class) : Class of the device

        Returns:
            list [obj] : List of the instance whose class is subclass of cls

        """
        return [ d for d in self.devices.values() if issubclass(type(d), cls) ]

    def data(self, name, clear=False):
        """Getting data from the device

        Args:
            name (str) : Name of the device
            clear (boolean, default = False) : Clear current-data

        Returns:
            ANY : Data from the device. Type of a return value depends on type of the devide.

        """
        dev = self.devices[name]
        return dev.data(clear)

    def waitData(self, name, timeout=None, clear=False):
        """Getting data, waiting if there is no current data

        Args:
            name (str) : Name of the device
            timeout (float, optional) : Time out in second
            clear (boolean, default = False) : Clear current-data

        Returns:
            ANY : Data from the device. Type of a return value depends on type of the devide.

        """
        dev = self.devices[name]
        return dev.waitData(timeout, clear=clear)

    def waitNextData(self, name, timeout=None, clear=False):
        """Getting data, waiting until subscribing new data

        Args:
            name (str) : Name of the device
            timeout (float, optional) : Time out in second
            clear (boolean, default = False) : Clear current-data

        Returns:
            ANY : Data from the device. Type of a return value depends on type of the devide.

        """
        dev = self.devices[name]
        return dev.waitNextData(timeout, clear=clear)

    def dataArray(self, names, clear=False):
        """Getting list of data from devices

        Args:
            names (list[str]) : Name of the device
            clear (boolean, default = False) : Clear current-data

        Returns:
            list[ANY] : Data from the device. Type of a return value depends on type of the devide.

        """
        return [ self.devices[name].data(clear) for name in names ]

    def waitDataArray(self, names, timeout=None, clear=False):
        """Getting list of data, waiting if there is no current data

        Args:
            names (list[str]) : Name of the device
            timeout (float, optional) : Time out in second
            clear (boolean, default = False) : Clear current-data

        Returns:
            ANY : Data from the device. Type of a return value depends on type of the devide.

        """
        for name in names:
            self.devices[name]._pre_wait(timeout)
        return [ self.devices[name]._fetch_data(clear) for name in names ]

    def waitNextDataArray(self, names, timeout=None, clear=False):
        """Getting list of data, waiting until subscribing new data

        Args:
            names (list[str]) : Name of the device
            timeout (float, optional) : Time out in second
            clear (boolean, default = False) : Clear current-data

        Returns:
            ANY : Data from the device. Type of a return value depends on type of the devide.

        """
        for name in names:
            self.devices[name]._pre_wait_next(timeout)
        return [ self.devices[name]._fetch_data(clear) for name in names ]

class RosDeviceBase(object):
    def __init__(self, dev_dict, robot=None):
        super().__init__()
        self.robot_callback = None
        self.__robot = robot
        self.topic = dev_dict['topic']
        self.name  = dev_dict['name']
        ## TODO: implement rate
        if 'rate' in dev_dict:
            self.rate = dev_dict['rate']
        else:
            self.rate = None
        ## TODO: search device in robot
        self.msg_time = None
        self.current_msg = None
        self.timeout = None
        self.sub = None

    @property
    def connected(self):
        if self.sub is not None and self.sub.get_num_connections() > 0:
            return True
        return False

    def parseType(self, type_str):
        tp = type_str.split('/')
        exec('from {}.msg import {}'.format(tp[0], tp[1]), locals(), globals())
        self.msg = eval('{}'.format(tp[1]))
        ## exec('class {}Wrapped({}):\n  __slots__ = ("header")')
        ## self.msg_wrapped = eval('{}Wrapped'.format(tp[1]))
    def subscribe(self):
        self.sub = rospy.Subscriber(self.topic, self.msg, self.callback)

    def callback(self, msg):
        #
        #if not hasattr(msg, 'header'):
        #    setattr(msg, 'header', std_msgs_header(stamp=rospy.get_rostime(), frame_id='add_by_ri'))
        #    ## msg.header = std_msgs_header(stamp=rospy.get_rostime(), frame_id='add_by_ri')
        self.msg_time = rospy.get_rostime()
        if self.robot_callback is not None:
            self.robot_callback(msg)
        self.current_msg = msg

    def returnData(self):
        return self.current_msg

    def data(self, clear=False):
        res = self.returnData()
        tm = self.msg_time
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
    ##
    @property
    def robot(self):
        return self.__robot
#    @robot.setter
#    def robot(self, in_robot):
#        self.__robot = in_robot

# generic device (using type: tag in robotinterface.yaml)
class RosDevice(RosDeviceBase):
    def __init__(self, dev_dict, robot=None):
        super().__init__(dev_dict, robot)
        self.parseType(dev_dict['type'])
        self.subscribe()
# specific device (using class: tag in robotinterface.yaml)
class StringDevice(RosDeviceBase):
    def __init__(self, dev_dict, robot=None):
        super().__init__(dev_dict, robot)
        import std_msgs.msg
        self.msg = std_msgs.msg.String
        self.subscribe()

class JointState(RosDeviceBase):
    def __init__(self, dev_dict, robot=None):
        super().__init__(dev_dict, robot)
        import sensor_msgs.msg
        self.msg = sensor_msgs.msg.JointState
        self.robot_callback = self.joint_callback
        self.subscribe()

    def joint_msg_to_robot(self, msg):
        for idx, nm in enumerate(msg.name):
            lk = self.robot.joint(nm)
            if lk:
                lk.q  = msg.position[idx]
                lk.dq = msg.velocity[idx]
                lk.u  = msg.effort[idx]

    def joint_callback(self, msg):
        #print('js: {} {}'.format(rtime, msg))
        self.joint_msg_to_robot(msg)

class JointTrajectoryState(RosDeviceBase): ## just store reference
    def __init__(self, dev_dict, robot=None):
        super().__init__(dev_dict, robot)
        from control_msgs.msg import JointTrajectoryControllerState as JState_msg
        self.msg = JState_msg
        self.subscribe()

class JointTrajectoryStateCallback(JointTrajectoryState):
    def __init__(self, dev_dict, robot=None):
        super().__init__(dev_dict, robot)
        self.robot_callback = self.joint_callback

    def joint_msg_to_robot(self, msg):
        for idx, nm in enumerate(msg.joint_names):
            lk = self.robot.joint(nm)
            if lk:
                lk.q = msg.actual.positions[idx]
                lk.q = msg.actual.velocities[idx]

    def joint_callback(self, msg):
        #print('js: {} {}'.format(rtime, msg))
        self.joint_msg_to_robot(msg)
#
# RobotInterface
#
class RobotInterface(JointInterface, DeviceInterface, MobileBaseInterface):
    """Interface for controllring robot (inheriting classes JointInterface, DeviceInterface and MobileBaseInterface)

    At a instance of this interface, all methods in JointInterface, DeviceInterface and MobileBaseInterface can be used.

    Then, please refer methods of these classes.
    """
    def __init__(self, file_name, node_name='robot_interface', anonymous=False, connection_wait=3.0, connection=True):
        """

        Args:
            file_name (str) : Name of setting.yaml file
            node_name (str, default='robot_interface') : Name of node
            anonymous (boolean, default = False) : If True, ROS node will start with this node-name.
            connection_wait (float, default=3.0) : Wait until ROS connection has established
            connection (boolean, default=True) : If false, create instace without ROS connection

        """
        with open(parseURLROS(file_name)) as f:
            self.info = yaml.safe_load(f)
        self.__load_robot()
        #
        self.__connection=False
        if not connection:
            return
        #
        ## ROS setup
        rospy.init_node(node_name, anonymous=anonymous)
        ## Interface(ROS) setup
        JointInterface.__init__(self, self.info)
        DeviceInterface.__init__(self, self.info)
        MobileBaseInterface.__init__(self, self.info)

        tmp = rospy.get_rostime()
        connect_=False
        while (rospy.get_rostime() - tmp).to_sec() < connection_wait:
            res = True
            if self.mobile_initialized:
                if not self.mobile_connected:
                    res = False
            if self.joint_initialized:
                if not self.joint_connected:
                    res = False
            if self.device_initialized:
                if not self.device_connected:
                    res = False
            if res:
                connect_=True
                break
            rospy.sleep(0.1)
        self.__connection=connect_

    def __load_robot(self):
        if 'robot_model' in self.info:
            mdl = self.info['robot_model']
            self.robot_name = mdl['name']
            self.model_file = parseURLROS(mdl['url'])

            ##rospy.loginfo('loading model from {}'.format(self.model_file))
            print('loading model from {}'.format(self.model_file))
            if not os.path.isfile(self.model_file):
                raise Exception('file: {} does not exist'.format(self.model_file))

            self.instanceOfBody = iu.loadRobot(self.model_file)
            if self.instanceOfBody is None:
                raise Exception('body can not be loaded by file: {}'.format(self.model_file))
            self.instanceOfJointBody = self.copyRobot()

            if 'class' in mdl:
                if 'import' in mdl:
                    import_ = mdl['import']
                    import_[-2:]
                    if '://' in import_ or import_[-3:] == '.py':
                        fname = parseURLROS(import_)
                        exec(open(fname).read(), locals(), globals())
                        exec('self.model_cls = {}'.format(mdl['class']), locals(), globals())
                    else:
                        # print('from {} import {}'.format(mdl['import'], mdl['class']))
                        exec('from {} import {}'.format(import_, mdl['class']), locals(), globals())
                        exec('self.model_cls = {}'.format(mdl['class']), locals(), globals())
                else:
                    exec('self.model_cls = {}'.format(mdl['class']), locals(), globals())
            else:
                self.model_cls = ru.RobotModelWrapped

    def getRobotModel(self, asItem=True):
        """Return an instance of RobotModel (irsl_choreonoid.robot_util.RobotModelWrapped)

        Args:
            asItem(boolean, default=True) : If True, model is generated as cnoid.BodyPlugin.BodyItem

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped : Newly generated instance

        """
        if asItem and iu.isInChoreonoid():
            rb = ib.loadRobotItem(self.model_file, world=False)
        else:
            rb = iu.loadRobot(self.model_file)
        return self.model_cls(rb)

    def getActualRobotModel(self, asItem=True):
        """Return an instance of RobotModel (irsl_choreonoid.robot_util.RobotModelWrapped) with RobotInterface internal robot-model

        Args:
            asItem(boolean, default=True) : If True, model is generated as cnoid.BodyPlugin.BodyItem

        Returns:
            irsl_choreonoid.robot_util.RobotModelWrapped : RobotModel created from RobotInterface internal robot-model

        """
        return self.model_cls(self.robot)

    def copyRobot(self):
        """Return other instance of the robot model

        Args:
            None

        Returns:
            cnoid.Body.Body : Copy of self.robot (this is not identical to self.robot)

        """
        return iu.loadRobot(self.model_file)

    def isRealRobot(self):
        """Checking this interface working with a real robot

        Args:
            None

        Retuns:
            boolean : If True, this interface connected to a real robot, otherwise connected to simulator

        """
        res_ = rospy.has_param('/use_sim_time')
        if res_:
            res_ = rospy.get_param('/use_sim_time')
        return not res_

    @property
    def connected(self):
        return self.__connection

    @property
    def effortVector(self):
        """Return vector of effort of actual robot \(sensing value\)

        This method requires JointState class in devices

        Returns:
            numpy.array : 1 x N vector ( N is len(jointList) )

        """
        return npa([ j.u for j in self.instanceOfBody.joints ])

    @property
    def velocityVector(self):
        """Return vector of velocity of actual robot \(sensing value\)

        This method requires JointState class in devices

        Returns:
            numpy.array : 1 x N vector ( N is len(jointList) )

        """
        return npa([ j.dq for j in self.instanceOfBody.joints ])

    @property
    def actualAngleVector(self):
        """Return angle-vector of actual robot \(sensing value\)

        This method requires JointState class in devices

        Returns:
            numpy.array : 1 x N vector ( N is len(jointList) )

        """
        return self.instanceOfBody.angleVector()

    @property
    def referenceAngleVector(self):
        """Return reference angle-vector \(value of past command\)

        This method requires JointTrajectoryState class in devices

        Returns:
            numpy.array : 1 x N vector ( N is len(jointList) )

        """
        res_lst = self.getDevicesByClass(JointTrajectoryState)
        if len(res_lst) < 1:
            return None
        tmp = self.instanceOfJointBody.angleVector()# store
        for res in res_lst:
            val = res.data()
            if val is None:
                return None
            for idx, nm in enumerate(val.joint_names):
                lk = self.instanceOfJointBody.joint(nm)
                if lk:
                    lk.q = val.desired.positions[idx]
        ret = self.instanceOfJointBody.angleVector()
        self.instanceOfJointBody.angleVector(tmp)# restore
        return ret

    @property
    def robot(self):
        """Return instance of the robot model (applying sensor values)

        Returns:
            cnoid.Body.Body : Instance of the robot model using in this instance

        """
        self.instanceOfBody.calcForwardKinematics()
        return self.instanceOfBody

    @property
    def jointRobot(self):
        """Return instance of the robot model (using for sending command)

        Returns:
            cnoid.Body.Body : Instance of the robot model using in this instance

        """
        self.instanceOfJointBody.calcForwardKinematics()
        return self.instanceOfJointBody
#    @body.setter
#    def body(self, in_body):
#        self.instanceOfBody = in_body

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

class GripperInterface(object):
    def __init__(self, topic):
        self._client = actionlib.SimpleActionClient(topic, GripperCommandAction)

    @property
    def connected(self):  ## override
        return self._client.wait_for_server(rospy.Duration(0.0001))

    def sendCommand(self, position, max_effort=10.0):
        com_ = GripperCommand(position=position, max_effort=max_effort)
        goal_ = GripperCommandGoal(command=com_)
        self._client.send_goal(goal_)

    def isFinished(self):  ## override
        return _client.simple_state != actionlib.SimpleGoalState.ACTIVE

    def waitUntilFinish(self, timeout=None):  ## override
        if timeout is None:
            timeout = rospy.Duration(1000000000.0)
        else:
            if timeout <= 0.0:
                timeout = 0.001
            timeout = rospy.Duration(timeout)
        self._client.wait_for_result(timeout)

    def cancel(self):
        self._client.cancel_all_goals()

class GripperAction(object):
    def __init__(self, gripper_topic, trajectory_topic, joint_names,
                 position_to_positions=None, effort_to_duration=None):
        self._action_name = gripper_topic
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                GripperCommandAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)

        self._target_ac = actionlib.SimpleActionClient(trajectory_topic, FollowJointTrajectoryAction)
        self._as.start()

        self._joint_names = joint_names
        self._position_to_positions = position_to_positions
        self._effort_to_duration = effort_to_duration

    def waitForServer(self, timeout=None):
        if timeout is not None:
            tm = rospy.Duration(timeout)
            return self._target_ac.wait_for_server(tm)
        else:
            return self._target_ac.wait_for_server()

    def sendTrajectoryFromGripperGoal(self, goal):
        header_ = std_msgs_header(stamp=rospy.Time(0))
        traj_ = JointTrajectory(header=header_, joint_names=self._joint_names)

        point = JointTrajectoryPoint()
        if self._position_to_positions is not None:
            point.positions = self._position_to_positions(goal.command.position)
        else:
            point.positions = [0.0] * len(self._joint_names)

        if self._effort_to_duration is not None:
            point.time_from_start = rospy.Duration(self._effort_to_duration(goal.command.max_effort))
        else:
            point.time_from_start = rospy.Duration(1.0)

        traj_.points.append(point)

        goal_ = FollowJointTrajectoryGoal(trajectory=traj_, goal_time_tolerance=rospy.Time(0.1))
        #print('before / st: {}'.format(self._target_ac.simple_state))
        self._target_ac.send_goal(goal_)
        #print('after / st: {}'.format(self._target_ac.simple_state))

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(20)
        success = True

        ## publish info to the console for the user
        rospy.loginfo('%s: Executing' % (self._action_name))

        self.sendTrajectoryFromGripperGoal(goal)

        while True:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self._target_ac.cancel_all_goals()
                success = False
                break
            # publish the feedback
            fd_ = GripperCommandFeedback()
            ## TODO: set value to feedback
            self._as.publish_feedback(fd_)

            res_ = self._target_ac.wait_for_result(rospy.Duration(0.001))
            if res_:
                rospy.loginfo('res_ = {} (state: {})'.format(res_, self._target_ac.simple_state))
                success = True
                break

            # check finish of target
            if self._target_ac.simple_state != actionlib.SimpleGoalState.ACTIVE:
                rospy.loginfo('%s: finished (state: %d)' % (self._action_name, self._target_ac.simple_state))
                if self._target_ac.simple_state == actionlib.SimpleGoalState.PENDING:
                    rospy.logwarn('%s: not started (state: %d)' % (self._action_name, self._target_ac.simple_state))
                    self._as.set_preempted()
                    self._target_ac.cancel_all_goals()
                    success = False
                break

            r.sleep()

        if success:
            res_ = GripperCommandResult()
            #TODO: set value to result
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(res_)
