### for development version
from cnoid.Base import RootItem
from cnoid.Base import ItemTreeView

from cnoid.BodyPlugin import AISTSimulatorItem
from cnoid.BodyPlugin import GLVisionSimulatorItem
from cnoid.BodyPlugin import BodyItem
from cnoid.BodyPlugin import WorldItem

##
from cnoid.ROSPlugin import WorldROSItem
from cnoid.ROSPlugin import BodyROSItem
from cnoid.ROSPlugin import ROSControlItem

import cnoid.Util

import yaml
# import irsl_choreonoid
# import irsl_choreonoid_ros

## TODO: check/fix parameter for choreonoid_ros/pybind11

#### yaml -> cnoid??
## robot:
##   model:
##   name:
##   initial_joint_angles:
##   initial_coords:
##   BodyROSItem:
##       publish_joint_state: false
##       joint_state_update_rate: 100
##       name_space: arm_robot3
##   ROSControlItem:
##       name_space: arm_robot3
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
## world: <== parse world first
##   ROS:
##     generate_urdf: ''
##     generate_controller_param: {param} <-- controller.yaml
##   World:
##     name:
##   WorldROS:
##   Simulator:
##      type: 'AISTSimulator'
##   GLVision:

param_method_dict = {
#    'param_name':'method_name'
#    'param_name':'variable_name='
    'name_space': 'nameSpace=',
    'joint_state_update_rate': 'jointStateUpdateRate=',
    'max_clock_publishing_rate': 'maxClockPublishingRate=',
    }
def _applyParameter(item, param):
    if type(param) is not dict:
        return
    for key,val in param.items():
        eval_str = ''
        if key in param_method_dict:
            method = param_method_dict(key)
            if method[-1] == '=':
                if hasattr(item, method[:-1]):
                    eval_str = 'item.' + method + 'val'
            else:
                if hasattr(item, method):
                    eval_str = 'item.' + method + '(val)'
        else:
            method = 'set' + ''.join([ s.capitalize() for s in key.split('_') ])
            if hasattr(item, method):
                eval_str = 'item.' + method + '(val)'
        if len(eval_str) > 0:
            print('eval: {}'.format(eval_str))
            eval(eval_str)

class BodyItemWrapper(object):
    def __init__(self, world):
        self.body_item = None
        self.world_item = world

    def addObject(self, info, fix=False):
        return self.addRobot(info, fix=fix, ros_enable=False)

    def addRobot(self, info, fix=False, ros_enable=False):
        self.body_item = BodyItem()
        fname = info['model']
        ## TODO: parse fname
        self.body_item.load(fname)
        if 'name' in info:
            self.body_item.setName(info['name'])
        self.body_item.body.updateLinkTree()
        self.body_item.body.initializePosition()
        if 'initial_joint_angles' in info:
            pass ## TODO:
        if 'initial_coords' in info:
            info['initial_coords']
            pass ## TODO:
        self.body_item.body.calcForwardKinematics()
        self.body_item.storeInitialState()
        if 'fix' in info:
            fix = info['fix']
        if fix: ## fix is overwrittern by info
            pass ## TODO:
        self.world_item.insertChildItem(self.body_item, self.world_item.childItem)
        if 'check' in info and info['check']:
            ItemTreeView.instance.checkItem(self.body_item)

        if ros_enable:
            if 'BodyROSItem' in info:
                self.addBodyROSItem(check=True, param=info['BodyROSItem'])
            if 'ROSControlItem' in info:
                self.addROSControlItem(check=True, param=info['ROSControlItem'])

    def addBodyROSItem(self, check=True, param=None):
        self.body_ros = BodyROSItem()
        self.body_ros.setName('BodyROSItem')
        # body_ros_item.nameSpace =
        # body_ros_item.jointStateUpdateRate =
        # body_ros_item.publishJointState = 
        self.body_item.addChildItem(self.body_ros)
        if param is not None:
            _applyParameter(self.body_ros, param)
        if check:
            ItemTreeView.instance.checkItem(self.body_ros)

    def addROSControlItem(self, check=True, param=None):
        self.ros_control = ROSControlItem()
        self.ros_control.setName('ROSControlItem')
        ## ros_cntrl_item.nameSpace = ''
        self.body_item.addChildItem(self.ros_control)
        if param is not None:
            _applyParameter(self.ros_control, param)
        if check:
            ItemTreeView.instance.checkItem(self.ros_control)

class CnoidSim(object):
    def __init__(self):
        self.world_item = None
        self.root_item = RootItem.instance
        self.robots =  []
        self.objects = []
        self.ros_enable = False
        self.simulator = None

    def parseInfo(self, info_dict):
        ### parse world first
        if 'world' in info_dict:
            world_info = info_dict['world']
            ## World
            if 'World' in world_info:
                self.addWorld(param=world_info['World'])
            else:
                self.addWorld()
            ## ROS
            if 'ROS' in world_info:
                self.ros_enable = True
                self.execROSScript(param=world_info['ROS'])
            ## WorldROS
            if 'WorldROS' in world_info:
                self.ros_enable = True
                self.addWorldROS(param=world_info['WorldROS'])
            elif self.ros_enable:
                self.addWorldROS()
            ## Simulator
            if 'Simulator' in world_info:
                self.addSimulator(param=world_info['Simulator'])
            ## GLVision
            if 'GLVision' in world_info:
                self.addGLVision(param=world_info['GLVision'])
        else:
            ## add default_world
            self.addWorld()
            self.addSimulator()
        if 'robot' in info_dict:
            self.addRobot(info_dict['robot'], ros_enable=self.ros_enable)
        if 'robots' in info_dict:
            for rb_info in info_dict['robots']:
                self.addRobot(rb_info, ros_enable=self.ros_enable)
        if 'object' in info_dict:
            self.addObject(info_dict['object'])
        if 'objects' in info_dict:
            for obj_info in info_dict['objects']:
                self.addRobot(obj_info)

    def execROSScript(self, param=None):
        pass

    def addWorld(self, name='World', check=True, param=None):
        wd = self.root_item.findItem(name)
        if wd is None:
            self.world_item = WorldItem()
            self.world_item.setName(name)
            self.root_item.addChildItem(self.world_item)
        else:
            self.world_item = wd
        #
        _applyParameter(self.world_item, param)
        if check:
            ItemTreeView.instance.checkItem(self.world_item)

    def addWorldROS(self, name='WorldROS', check=True, param=None):
        ros_world = world_item_inst.findItem(name)
        if ros_world is None:
            self.ros_world = WorldROSItem()
            self.ros_world.setName(name)
            ros_world.maxClockPublishingRate = 100
            self.world_item.addChildItem(self.ros_world)## ? insert
        else:
            self.ros_world = ros_world
        #
        _applyParameter(self.ros_world, param)
        if check:
            ItemTreeView.instance.checkItem(self.ros_world)

    def addRobot(self, info=None, ros_enable=False):
        bi = BodyItemWrapper(self.world_item)
        bi.addRobot(info, ros_enable=ros_enable)

    def addObject(self, info=None):
        bi = BodyItemWrapper(self.world_item)
        bi.addRobot(info)

    #def addFixedObject(self):
    #    pass

    def addSimulator(self, check=True, param=None): ## name is overwritten by param
        if 'type' in param:
            exec('self.simulator = {}Item()'.format(param['type']), locals(), globals())
        print('self.simulator: {}'.format(self.simulator) )
        if self.simulator is not None:
            ## set integrationMode: runge-kutta
            self.world_item.addChildItem(self.simulator)## ? insert
            _applyParameter(self.simulator, param)
            if check:
                ItemTreeView.instance.checkItem(self.simulator)

    def addGLVision(self, simulator=None, target_bodies=None, target_sensors=None, param=None):
        if self.simulator is None:
            return
        vsim = GLVisionSimulatorItem()
        if param is not None and 'target_bodies' in param:
            vsim.setTargetBodies(param['target_bodies'])
        elif target_bodies is not None:
            vsim.setTargetBodies(target_bodies)
        if param is not None and 'target_sensors' in param:
            vsim.setTargetSensors(param['target_sensors'])
        elif target_sensors is not None:
            vsim.setTargetSensors(target_sensors)
        ## default parameters
        vsim.setMaxFrameRate(1000)
        vsim.setMaxLatency(0)
        vsim.setVisionDataRecordingEnabled(False)
        ## vsim.setThreadEnabled(True)
        vsim.setDedicatedSensorThreadsEnabled(True)
        vsim.setBestEffortMode(True)
        vsim.setRangeSensorPrecisionRatio(2.0)
        #vsim.setAllSceneObjectsEnabled(False)
        vsim.setAllSceneObjectsEnabled(True)
        vsim.setHeadLightEnabled(True)
        vsim.setAdditionalLightsEnabled(True)
        ##
        _applyParameter(vsim, param)
        #add
        self.simulator.addChildItem(vsim)
        self.glvision = vsim
