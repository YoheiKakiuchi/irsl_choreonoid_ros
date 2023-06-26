### for development version
from cnoid.Base import RootItem
from cnoid.Base import ItemTreeView

from cnoid.BodyPlugin import AISTSimulatorItem
from cnoid.BodyPlugin import BodyItem
from cnoid.BodyPlugin import WorldItem

from cnoid.ROSPlugin import WorldROSItem
from cnoid.ROSPlugin import BodyROSItem
from cnoid.ROSPlugin import ROSControlItem

import cnoid.Util

import yaml
# import irsl_choreonoid
# import irsl_choreonoid_ros

#### yaml
## robot:
##   model:
##   name:
##   initial_joint_angles:
##   initial_coords:
##   BodyROSItem:
##   ROSControlItem:
## robots:
##   - { robot: }
## object:
##   model:
##   name:
##   initial_coords:
##   initial_joint_angles:
##   fixed:
## objects:
##   - { object: }
## world:
##   ROS:
##     generate_urdf: ''
##     generate_controller: {param}
##   World:
##     name:
##   WorldROS:
##   AISTSimulator:
##   GL:
def check_item(itm):
    ItemTreeView.instance.checkItem(itm)

param_method_dict = {
#    'param_name':'method_name'
#    'param_name':'variable_name='
    'name_space':'nameSpace='
    'joint_state_update_rate':'jointStateUpdateRate='
    'max_clock_publishing_rate':'maxClockPublishingRate='
    }
def applyParameter(item, param):
    for key,val in param.items():
        eval_str = ''
        if key in param_method_dict:
            method = param_method_dict(key)
            if method[-1] == '=':
                eval_str = 'item.' + method + 'val'
            else:
                eval_str = 'item.' + method + '(val)'
        else:
            eval_str = 'item.set' + ''.join([s.capitalize() for s in key.split('_')]) + '(val)'
        if len(eval_str) > 0:
            eval(eval_str)

class BodyItemWrapper(object):
    def __init__(self, world):
        self.body_item = None
        self.world_item = world

    def addBody(self, info, fix=False, ros_enable=False):
        self.body_item = BodyItem()
        self.body_item.load(info['model'])
        if 'name' in info:
            self.body_item.setName(info['name'])
        self.body_item.body.updateLinkTree()
        self.body_item.body.initializePosition()
        if 'initial_joint_angles' in info:
            pass ## TODO:
        if 'initial_coords' in info:
            pass ## TODO:
        self.body_item.body.calcForwardKinematics()
        self.body_item.storeInitialState()
        if fix:
            pass ## TODO:
        self.world_item.insertChildItem(self.body_item, self.world_item.childItem)
        if 'check' in info and info['check']:
            check_item(self.body_item)

        if ros_enable:
            
    def addBodyROSItem(self, check=True, param=None):
        self.body_ros = BodyROSItem()
        self.body_ros.setName('BodyROSItem')
        # body_ros_item.nameSpace =
        # body_ros_item.jointStateUpdateRate =
        # body_ros_item.publishJointState = 
        self.body_item.addChildItem(self.body_ros)
        if param is not None:
            applyParameter(self.body_ros, param)
        if check:
            check_item(self.body_ros)

    def addROSControl(self, check=True, param=None):
        self.ros_control = ROSControlItem()
        self.ros_control.setName('ROSControlItem')
        ## ros_cntrl_item.nameSpace = ''
        self.body_item.addChildItem(self.ros_control)
        if param is not None:
            applyParameter(self.ros_control, param)
        if check:
            check_item(self.ros_control)

class CnoidSim(object):
    def __init__(self, yaml_file=None):
        self.world_item = None
        self.root_item = RootItem.instance
        self.robots = []
        self.objects = []
        self.ros_enable = False
        if yaml_file is not None:
            pass

    def parseInfo(self, info_dict):
        if 'world' in info_dict:
            ##
            if 'World' in info_dict['world']:
                self.addWorld(param=info_dict['world']['World'])
            else:
                self.addWorld()
            ##
            if 'ROS' in info_dict['world']:
                self.ros_enable = True
                ## TODO
            if 'WorldROS' in info_dict['world']:
                self.ros_enable = True
                self.addWorldROS(param=info_dict['world']['WorldROS'])
            elif self.ros_enable:
                self.addWorldROS()
            ##
            if 'AISTSimulator' in info_dict['world']:
                
        else:
            ## add default_world
            self.addWorld()
            self.addSimulator()
        if 'robot' in info_dict:
            bi = BodyItemWrapper(self.world_item)
            bi.addBody(info_dict['robot'], ros_enable=self.ros_enable)
        if 'robots' in info_dict:
            pass
        if 'object' in info_dict:
            pass
        if 'objects' in info_dict:
            pass

    def addWorld(self, name='World', check=True, param=None):
        wd = self.root_item.findItem(name)
        if wd is None:
            self.world_item = WorldItem()
            self.world_item.setName(name)
            self.root_item.addChildItem(self.world_item)
        else:
            self.world_item = wd
        if param is not None:
            applyParameter(self.world_item, param)
        if check:
            check_item(self.world_item)

    def addRobot(self):
        pass
    def addFixedObject(self):
        pass
    def addBody(self, info=None):
        pass

    def addSimulator(self, name='AISTSimulator', check=True, param=None):
        aist_sim = self.world_item.findItem(name)
        if aist_sim is None:
            self.simulator = AISTSimulatorItem()
            self.simulator.setName(name)
            self.world_item.addChildItem(self.simulator)## ? insert
        else:
            self.simulator = aist_sim
        if param is not None:
            applyParameter(self.simulator, param)
        if check:
            check_item(self.simulator)

    def addGLVISION(self, simulator, target_bodies, target_sensors, param=None):
        vsim = GLVisionSimulatorItem()
        vsim.setTargetBodies(target_bodies)
        vsim.setTargetSensors(target_sensors)
        ## default parameters
        vsim.setMaxFrameRate(1000)
        vsim.setMaxLatency(0)
        vsim.setVisionDataRecordingEnabled(False)
        ## vsim.setThreadEnabled(True)
        vsim.setDedicatedSensorThreadsEnabled(True)
        vsim.setBestEffortMode(True)
        vsim.setRangeSensorPrecisionRatio(2.0)
        vsim.setAllSceneObjectsEnabled(False)
        vsim.setHeadLightEnabled(True)
        vsim.setAdditionalLightsEnabled(True)
        if param is not None:
            applyParameter(vsim, param)
        #add
        self.simulator.addChildItem(vsim)
        self.glvision = vsim

    def addWorldROS(self, name='WorldROS', check=True, param=None):
        ros_world = world_item_inst.findItem(name)
        if ros_world is None:
            self.ros_world = WorldROSItem()
            self.ros_world.setName(name)
            ros_world.maxClockPublishingRate = 100
            self.world_item.addChildItem(self.ros_world)## ? insert
        else:
            self.ros_world = ros_world
        if param is not None:
            applyParameter(self.ros_world, param)
        if check:
            check_item(self.ros_world)
