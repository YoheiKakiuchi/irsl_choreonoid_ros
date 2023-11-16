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

#from irsl_choreonoid_ros.cnoid_ros_util import parseURLROS
from .cnoid_ros_util import parseURLROS
from irsl_choreonoid.robot_util import make_coordinates
import irsl_choreonoid.cnoid_base as ib

import yaml

## ROS
import rosgraph
import rospy

#### specification of yaml
## robot:
##   model: @robot_file_name@
##   name: MyRobot
##   initial_joint_angles: []
##   initial_coords: {} ## using make_coordinates from irsl_choreonoid.robot_util
##   BodyROSItem: ## should be launch from choreonoid_ros
##       publish_joint_state: false
##       joint_state_update_rate: 100
##       name_space: arm_robot3
##   ROSControlItem: ## should be launch from choreonoid_ros
##       name_space: arm_robot3
## robots:
##   - { robot: }
## object:
##   model: @object_file_name@
##   name: MyObject
##   initial_coords: []
##   initial_joint_angles: {} ## using make_coordinates from irsl_choreonoid.robot_util
##   fixed: True ##
## objects:
##   - { object: }
## world:
##   World:
##     name:
##     draw_grid: False
##   Simulator:
##     type: 'AISTSimulator'
##   GLVision:
##   Camera:
##     lookEye:
##     lookForDirection:
##     lookAtCenter:
##     lookAtUp:
##     position:
##     fov:
##   WorldROS: ## should be launch from choreonoid_ros
##   ROS:
##     urdf_settings:
##          file: @file_name@
##          robotName:
##          transmission:
##          name:
##     set_parameter: 
##          - type: 'yaml'
##            file: @file_name@
##            name: 
##          - type: 'param'
##            name: 
##            parameter: {}
##          - parameter: {}

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
            # print('eval: {}'.format(eval_str)) ## debug
            eval(eval_str)

def _getDictValue(in_dict, keys):
    for k in keys:
        if k in in_dict:
            return in_dict[k]

def _getDictValueExist(in_dict, keys):
    for k in keys:
        if k in in_dict:
            return (True, in_dict[k])
    return (False, None)

class _BodyItemWrapper(object):
    def __init__(self, world):
        self.body_item = None
        self.world_item = world

    def addObject(self, info, fix=False):
        return self.addRobot(info, fix=fix, ros_enable=False)

    def addRobot(self, info, fix=False, ros_enable=False):
        self.body_item = BodyItem()
        model_ = _getDictValue(info, ('model', 'Model', 'file', 'File', 'body', 'Body', 'model_file', 'modelFile', 'uri', 'URI'))
        if model_ is None:
            return
        fname = parseURLROS(model_)
        self.body_item.load(fname)
        ##
        name_ = _getDictValue(info, ('name', 'Name', 'modelName'))
        if name_ is not None:
            self.body_item.setName(name_)
        ##
        self.body_item.body.updateLinkTree()
        self.body_item.body.initializePosition()
        angles_ = _getDictValue(info, ('initial_joint_angles', 'joint_angles', 'initial_angles', 'jointAngles', 'initialAngles',
                                       'initial_joint_angle', 'joint_angle', 'initial_angle', 'jointAngle', 'initialAngle',
                                       'initialJointAngles', 'initialJointAngle'))
        if angles_ is not None:
            for j, q in zip(self.body_item.body.joints, angles_):
                j.q = q
        cds_dict_ =  _getDictValue(info, ('initial_coords', 'initial_position', 'Coords', 'Position', 'initialCoords', 'initialPosition'))
        if cds_dict_ is not None:
            cds = make_coordinates(cds_dict_)
            self.body_item.body.rootLink.setPosition(cds.cnoidPosition)
        self.body_item.body.calcForwardKinematics()
        self.body_item.storeInitialState()
        ##
        fix_ = _getDictValue(info, ('fix', 'Fix', 'fixedObject', 'fixed', 'Fixed'))
        if fix_ is not None:
            fix = fix_
        if fix: ## fix is overwrittern by info
            self.body_item.body.setRootLinkFixed(True);
        ##
        self.world_item.insertChildItem(self.body_item, self.world_item.childItem)
        ##
        check = True
        if 'no_check' in info and info['no_check']:
            check = False
        else:
            ItemTreeView.instance.checkItem(self.body_item)
        ## for choreonoid_ros
        if ros_enable:
            if 'BodyROSItem' in info:
                self.addBodyROSItem(self.body_item, check=check, param=info['BodyROSItem'])
            if 'ROSControlItem' in info:
                self.addROSControlItem(self.body_item, check=check, param=info['ROSControlItem'])

    def addBodyROSItem(self, parent, check=True, param=None):
        self.body_ros = BodyROSItem()
        self.body_ros.setName('BodyROSItem')
        # body_ros_item.nameSpace =
        # body_ros_item.jointStateUpdateRate =
        # body_ros_item.publishJointState =
        if parent is not None:
            parent.addChildItem(self.body_ros)
        if param is not None:
            _applyParameter(self.body_ros, param)
        if check:
            ItemTreeView.instance.checkItem(self.body_ros)

    def addROSControlItem(self, parent, check=True, param=None):
        self.ros_control = ROSControlItem()
        self.ros_control.setName('ROSControlItem')
        ## ros_cntrl_item.nameSpace = ''
        if parent is not None:
            parent.addChildItem(self.ros_control)
        if param is not None:
            _applyParameter(self.ros_control, param)
        if check:
            ItemTreeView.instance.checkItem(self.ros_control)

class SetupCnoid(object):
    def __init__(self):
        self.world_item = None
        self.root_item = RootItem.instance
        self.robots =  []
        self.objects = []
        self.ros_enable = False
        self.simulator = None

    def buildEnvironment(self, info_dict, world='World', createWorld=False, setCamera=False):
        """
        Building environment (setting objects) under the WorldItem

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing objects on environment
            world (str, default='World') : Name of WorldItem, added objects under this item
            craeteWorld (boolean, default=False) : If True, creating new WorldItem
            setCamera (boolean, default=False) : If True, set camera position
        """
        worldItem = None
        if createWorld:
            if 'world' in info_dict:
                world_info = info_dict['world']
                ## World
                exist_, world_ = _getDictValueExist(world_info, ('World', 'world', 'WORLD'))
                if exist_:
                    self._addWorld(param=world_)
            else:
                self._addWorld(name=world)
            worldItem = self.world_item
        else:
            worldItem = self.root_item.findItem(world)
        ##
        if worldItem is None:
            worldItem = self.root_item
        ##
        if setCamera:
            if 'world' in info_dict:
                world_info = info_dict['world']
                camera_ = _getDictValue(world_info, ('Camera', 'camera', 'View', 'view'))
                if camera_ is not None:
                    self._setCameraPosition(param=camera_)
        ##
        if 'object' in info_dict:
            self._addObject(info_dict['object'], worldItem=worldItem)
        if 'objects' in info_dict:
            for obj_info in info_dict['objects']:
                self._addObject(obj_info, worldItem=worldItem)

    def createCnoid(self, info_dict, addDefaultSimulator=True, addDefaultWorld=True, noEnvironment=False):
        """
        Creating project from parameters

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing the project
            addDefaultSimulator (boolean, default=True) : If True, adding new SimulatorItem if there is no instruction in info_dict
            addDefaultWorld (boolean, default=True) : If True, adding new WorldItem if there is no instruction in info_dict
            noEnvironment (boolean, default=False) : If True, not adding environment(objects). Use buildEnvironment method.

        """
        ### parse world first
        if 'world' in info_dict:
            world_info = info_dict['world']
            ## World
            exist_, world_ = _getDictValueExist(world_info, ('World', 'world', 'WORLD'))
            if exist_:
                self._addWorld(param=world_)
            else:
                self._addWorld()
            is_master_exists = rosgraph.is_master_online()
            ## ROS
            exist_, ros_ = _getDictValueExist(world_info, ('ros', 'ROS', 'Ros'))
            if exist_:
                if is_master_exists:
                    self.ros_enable = True
                self._execROSScript(param=ros_)
            ## WorldROS
            exist_, world_ros_ = _getDictValueExist(world_info, ('WorldROS', 'world_ros'))
            if exist_:
                if is_master_exists:
                    self.ros_enable = True
                    self._addWorldROS(param=world_ros_)
            elif self.ros_enable:
                if is_master_exists:
                    self._addWorldROS()
            ## Simulator
            exist_, simulator_ = _getDictValueExist(world_info, ('Simulator', 'simulator', 'SimulatorItem'))
            if exist_:
                self._addSimulator(param=simulator_)
            ## GLVision
            exist_, gl_vision_ = _getDictValueExist(world_info, ('GLVision', 'gl_vision', 'Vision', 'vision'))
            if exist_:
                self._addGLVision(param=gl_vision_)
            ## Camera
            camera_ = _getDictValue(world_info, ('Camera', 'camera', 'View', 'view'))
            if camera_ is not None:
                self._setCameraPosition(param=camera_)
        else:
            ## add default_world
            if addDefaultWorld:
                self._addWorld()
            if addDefaultSimulator:
                self._addSimulator()

        if 'robot' in info_dict:
            self._addRobot(info_dict['robot'], ros_enable=self.ros_enable)
        if 'robots' in info_dict:
            for rb_info in info_dict['robots']:
                self._addRobot(rb_info, ros_enable=self.ros_enable)

        if noEnvironment:
            if 'object' in info_dict:
                self._addObject(info_dict['object'])
            if 'objects' in info_dict:
                for obj_info in info_dict['objects']:
                    self._addObject(obj_info)

    def _addWorld(self, name='World', check=True, param=None):
        wd = self.root_item.findItem(name)
        if wd is None:
            self.world_item = WorldItem()
            _applyParameter(self.world_item, param)
            self.root_item.addChildItem(self.world_item)
        else:
            self.world_item = wd
        ##
        grid_ = _getDictValue(param, ('draw_grid', 'DrawGrid', 'grid', 'Grid'))
        if grid_ is not None:
            if grid_ is False:
                ib.disableGrid()
            elif grid_ == 'XY':
                ib.disableGrid()
                ib.enableGrid(0)
            elif grid_ == 'XZ':
                ib.disableGrid()
                ib.enableGrid(1)
            elif grid_ == 'YZ':
                ib.disableGrid()
                ib.enableGrid(2)
            elif type(grid_) is list or type(grid_) is tuple:
                ib.disableGrid()
                for g_ in grid_:
                    if g_ == 'XY':
                        ib.enableGrid(0)
                    elif g_ == 'XZ':
                        ib.enableGrid(1)
                    elif g_ == 'YZ':
                        ib.enableGrid(2)
        ##
        if check:
            ItemTreeView.instance.checkItem(self.world_item)

    def _addWorldROS(self, name='WorldROS', check=True, param=None):
        ros_world = self.root_item.findItem(name)
        if ros_world is None:
            self.ros_world = WorldROSItem()
            self.ros_world.setName(name)
            _applyParameter(self.ros_world, param)
            self.ros_world.maxClockPublishingRate = 100
            self.world_item.addChildItem(self.ros_world)## ? insert
        else:
            self.ros_world = ros_world
        #
        if check:
            ItemTreeView.instance.checkItem(self.ros_world)

    def _addRobot(self, info=None, ros_enable=False, worldItem=None):
        if worldItem is not None:
            bi = _BodyItemWrapper(worldItem)
        else:
            bi = _BodyItemWrapper(self.world_item)
        bi.addRobot(info, ros_enable=ros_enable)

    def _addObject(self, info=None, worldItem=None):
        if worldItem is not None:
            bi = _BodyItemWrapper(worldItem)
        else:
            bi = _BodyItemWrapper(self.world_item)
        bi.addRobot(info)

    def _addSimulator(self, check=True, param=None): ## name is overwritten by param
        if 'type' in param:
            exec('self.simulator = {}Item()'.format(param['type']), locals(), globals())
        if self.simulator is not None:
            ## set integrationMode: runge-kutta
            self.world_item.addChildItem(self.simulator)## ? insert
            _applyParameter(self.simulator, param)
            if check:
                ItemTreeView.instance.checkItem(self.simulator)

    def _addGLVision(self, simulator=None, target_bodies=None, target_sensors=None, param=None):
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

    def _setCameraPosition(self, param=None):
        if param is None:
            return
        fov_ = _getDictValue(param, ('fov', 'FOV'))
        cds_ = _getDictValue(param, ('position', 'Position', 'coords'))
        if cds_ is not None:
            cds_ = make_coordinates(cds_)
            ib.setCameraCoords(cds_, fov_)
        else:
            eye_ = _getDictValue(param, ('lookEye', 'eye', 'Eye'))
            up_ = _getDictValue(param, ('lookUp', 'up', 'Up'))
            if eye_ is not None and up_ is not None:
                dir_ = _getDictValue(param, ('lookForDirection', 'lookDirection', 'Direction', 'direction'))
                if dir_ is not None:
                    cds = ib.cameraPositionLookingFor(eye_, dir_, up_)
                    ib.setCameraCoords(cds, fov_, opencv=False)
                else:
                    center_ = _getDictValue(param, ('lookAtCenter', 'lookAt', 'at', 'center'))
                    if center_ is not None:
                        cds = ib.cameraPositionLookingAt(eye_, center_, up_)
                        ib.setCameraCoords(cds, fov_, opencv=False)

    def _execROSScript(self, param=None):
        if param is None:
            return
        urdf_ = _getDictValue(param, ('urdf_settings', 'URDFSettings', 'URDF_settings', 'urdf_setting', 'URDFSetting', 'URDF_setting', 'URDF', 'urdf'))
        if urdf_ is not None:
            self._parseURDF(urdf_)
        parameters_ = _getDictValue(param, ('set_parameter', 'set_parameters', 'setParameter', 'setParameters', 'parameters', 'Parameters'))
        if parameters_ is not None:
            self._parseParam(parameters_)

    def _parseURDF(self, param):
        pass

    def _parseParam(self, param):
        if type(param) is not list and type(param) is not tuple:
            return
        for one in param:
            if type(one) is dict:
                self._parseSingleParam(one)

    def _parseSingleParam(self, param):
        if 'type' in param:
            if param['type'] == 'yaml':
                file_ = _getDictValue(param, ('file', 'File', 'URI', 'uri', 'yaml', 'yaml_file'))
                if file_ is not None:
                    fn = parseURLROS(file_)
                    yparam = yaml.safe_load(open(fn))
                    name_ = _getDictValue(param, ('name', 'param', 'Name', 'Param', 'parameter_name', 'parameterName'))
                    if name_ is not None:
                        rospy.set_param(name_, yparam)
                    else:
                        for k,v in yparam.items():
                            if type(k) is str:
                                rospy.set_param(k, v)
                return
        if 'parameter' in param:
            p = param['parameter']
            name_ = _getDictValue(param, ('name', 'param', 'Name', 'Param', 'parameter_name', 'parameterName'))
            if name_ is not None:
                rospy.set_param(name_, p)
            else:
                for k,v in p.items():
                    if type(k) is str:
                        rospy.set_param(k, v)