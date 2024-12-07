### for development version
from cnoid.Base import RootItem
from cnoid.Base import ItemTreeView

from cnoid.BodyPlugin import AISTSimulatorItem
from cnoid.BodyPlugin import GLVisionSimulatorItem
from cnoid.BodyPlugin import BodyItem
from cnoid.BodyPlugin import WorldItem
from cnoid.BodyPlugin import SimulationBar
##
from cnoid.ROSPlugin import WorldROSItem
from cnoid.ROSPlugin import BodyROSItem
from cnoid.ROSPlugin import ROSControlItem
##
import cnoid.Body as cbody

import cnoid.Util
from cnoid.IRSLCoords import coordinates

#from irsl_choreonoid_ros.cnoid_ros_util import parseURLROS
from .cnoid_ros_util import parseURLROS
from irsl_choreonoid.robot_util import make_coordinates
import irsl_choreonoid.cnoid_util as iu
import irsl_choreonoid.cnoid_base as ib
import irsl_choreonoid.make_shapes as mkshapes

import yaml
import math
import sys

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
##       joint_state_publication: false
##       joint_state_update_rate: 100
##       name_space: arm_robot3
##   ROSControlItem: ## should be launch from choreonoid_ros
##       name_space: arm_robot3
## robots:
##   - { robot: }
## object:
##   model: @object_file_name@
##   name: MyObject
##   initial_joint_angles: []
##   initial_coords: {} ## using make_coordinates from irsl_choreonoid.robot_util
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
##     position: { pos: [], aa: [] }
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
##     generate_settings:
##        robot: @robot_file_name@
##        controllers: [ {name: '', type: '', joints: [] } ]
## pythonScript:
####

param_method_dict = {
#    'param_name':'method_name'
#    'param_name':'variable_name='
    'name_space': 'nameSpace=',
    'max_clock_publishing_rate': 'maxClockPublishingRate=',
    'joint_state_update_rate': 'jointStateUpdateRate=',
    'joint_state_publication': 'jointStatePublication=',
    }
def _applyParameter(item, param):
    if type(param) is not dict:
        return
    for key,val in param.items():
        #print("{}/{}".format(key, val))
        eval_str = ''
        if key in param_method_dict:
            method = param_method_dict[key]
            #print("method: {}".format(method))
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
            elif hasattr(item, 'set' + key):
                eval_str = 'item.set' + key + '(val)'
            elif hasattr(item, key):
                eval_str = 'item.' + key + '(val)'

        if len(eval_str) > 0:
            print('eval: {} / val={}'.format(eval_str, val)) ## debug
            exec(eval_str, locals(), globals())

def _getDictValue(in_dict, keys):
    if in_dict is None:
        return
    for k in keys:
        if k in in_dict:
            return in_dict[k]

def _getDictValueExist(in_dict, keys):
    if in_dict is None:
        return (False, None)
    for k in keys:
        if k in in_dict:
            return (True, in_dict[k])
    return (False, None)

def _splitFiles(fname):
    res = fname.split(';')
    if len(res) > 1:
        return res
    return None

class _BodyItemWrapper(object):
    def __init__(self, world, offset=None):
        self.body_item = None
        self.world_item = world
        self.offset = offset

    def addObject(self, info, fix=False):
        model_ = _getDictValue(info, ('model', 'Model', 'file', 'File', 'body', 'Body', 'model_file', 'modelFile', 'uri', 'URI'))
        if model_ is None:
            return self.addPrimitive(info)## always fixed
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
            if self.offset is not None:
                cds.transform(self.offset, coordinates.wrt.world)
            self.body_item.body.rootLink.setPosition(cds.cnoidPosition)
        elif self.offset is not None:
            self.body_item.body.rootLink.setPosition(self.offset.cnoidPosition)
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

    def _addPrimitiveWithShape(self, shape, info):
        bitem = BodyItem()
        nm_ = _getDictValue(info, ('name', 'Name', 'NAME'))
        if nm_ is not None:
            bitem.setName(nm_)
        baselk = bitem.body.createLink()
        baselk.addShapeNode(shape)
        cds = make_coordinates(info)
        baselk.setPosition(cds.cnoidPosition)
        baselk.setJointType(cbody.Link.JointType.FixedJoint)
        bitem.body.setRootLink(baselk)
        ##
        bitem.body.updateLinkTree()
        bitem.body.calcForwardKinematics()
        ##
        self.world_item.insertChildItem(bitem, self.world_item.childItem)
        ItemTreeView.instance.checkItem(bitem)
        return bitem

    def addPrimitive(self, info, fix=True):
        model_ = _getDictValue(info, ('box', 'Box', 'BOX'))
        if model_ is not None:
            shape_ = mkshapes.makeBox(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('cylinder', 'Cylinder', 'CYLINDER'))
        if model_ is not None:
            shape_ = mkshapes.makeCylinder(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('sphere', 'Sphere', 'SPHERE'))
        if model_ is not None:
            shape_ = mkshapes.makeSphere(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('cone', 'Cone', 'CONE'))
        if model_ is not None:
            shape_ = mkshapes.makeCone(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('capsule', 'Capsule', 'CAPSULE'))
        if model_ is not None:
            shape_ = mkshapes.makeCapsule(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return
        model_ = _getDictValue(info, ('torus', 'Torus', 'TORUS'))
        if model_ is not None:
            shape_ = mkshapes.makeTorus(*model_, rawShape=True, **info)
            self._addPrimitiveWithShape(shape_, info)
            return

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
    """
    Utility class for setting .cnoid file from python script
    """
    def __init__(self, rootItem=None, worldItem=None):
        """
        Args:
            rootItem (cnoid.Base.Item, optional) : If set, it is used as root for creating environment
            worldItem (cnoid.Base.WorldItem, optional) : If set, it is used as a worldItem
        """
        self.world_item = worldItem
        if rootItem:
            self.root_item = rootItem
        else:
            self.root_item = RootItem.instance
        #self.robots =  []
        #self.objects = []
        self.ros_enable = False
        self.simulator = None
        self.simulatorRobot = None

    def buildEnvironment(self, info_dict, world='World', createWorld=False, setCamera=False, offset=None):
        """
        Building environment (setting objects) under the WorldItem

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing objects on environment
            world (str, default='World') : Name of WorldItem, added objects under this item
            craeteWorld (boolean, default=False) : If True, creating new WorldItem
            setCamera (boolean, default=False) : If True, set camera position
            offset (cnoid.IRSLCoords.coordinates) : Offset of objects
        """
        if type(info_dict) is list:
            for obj_info in info_dict:
                self._addObject(obj_info, worldItem=worldItem, offset=offset)
            return

        if type(info_dict) is not dict:
            raise Exception('type of {} is not dict'.format(info_dict))

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
            if self.world_item:
                worldItem = self.world_item
            else:
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
            self._addObject(info_dict['object'], worldItem=worldItem, offset=offset)
        if 'objects' in info_dict:
            for obj_info in info_dict['objects']:
                self._addObject(obj_info, worldItem=worldItem, offset=offset)
            # notify
    def createCnoid(self, info_dict, addDefaultSimulator=True, addDefaultWorld=True, noEnvironment=False):
        """
        Creating project from parameters

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing the project
            addDefaultSimulator (boolean, default=True) : If True, adding new SimulatorItem if there is no instruction in info_dict
            addDefaultWorld (boolean, default=True) : If True, adding new WorldItem if there is no instruction in info_dict
            noEnvironment (boolean, default=False) : If True, not adding environment(objects). Use buildEnvironment method.

        """
        if type(info_dict) is not dict:
            raise Exception('type of {} is not dict'.format(info_dict))

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

        if not noEnvironment:
            if 'object' in info_dict:
                self._addObject(info_dict['object'])
            if 'objects' in info_dict:
                for obj_info in info_dict['objects']:
                    self._addObject(obj_info)
            # notify
    def buildEnvironmentFromYaml(self, yamlFile, **kwargs):
        """
        Building environment from yaml-file

        Args:
            yamlFile (str) : File name to load
            kwargs (dict) : Keyword to pass to setup_cnoid.buildEnvironment

        """
        files = _splitFiles(yamlFile)
        if files is None:
            fname = parseURLROS(yamlFile)
            info_ = yaml.safe_load(open(fname))
            self.buildEnvironment(info_, **kwargs)
        else:
            fname = parseURLROS(files[0])
            info_ = yaml.safe_load(open(fname))
            self.buildEnvironment(info_, **kwargs)
            kwargs['createWorld'] = False
            kwargs['setCamera'] = False
            for f in files[1:]:
                fname = parseURLROS(f)
                info_ = yaml.safe_load(open(fname))
                self.buildEnvironment(info_, **kwargs)

    def createCnoidFromYaml(self, yamlFile, **kwargs):
        """
        Creating project from yaml-file

        Args:
            yamlFile (str) : File name to load
            kwargs (dict) : Keyword to pass to setup_cnoid.createCnoid

        """
        files = _splitFiles(yamlFile)
        if files is None:
            fname = parseURLROS(yamlFile)
            info_ = yaml.safe_load(open(fname))
            self.createCnoid(info_, **kwargs)
        else:
            fname = parseURLROS(files[0])
            info_ = yaml.safe_load(open(fname))
            self.createCnoid(info_, **kwargs)
            kwargs['createWorld'] = False
            kwargs['setCamera'] = False
            for f in files[1:]:
                fname = parseURLROS(f)
                info_ = yaml.safe_load(open(fname))
                self.buildEnvironment(info_, **kwargs)

    @classmethod
    def setEnvironmentFromYaml(cls, yamlFile, **kwargs):
        """
        Setup environment from yaml-file (classmethod of buildEnvironmentFromYaml)

        Args:
            yamlFile (str) : File name to load
            kwargs (dict) : Keyword to pass to setup_cnoid.buildEnvironment

        Returns:
            irsl_choreonoid_ros.setup_cnoid : Instance of setup_cnoid

        """
        cnoid = cls()
        cnoid.buildEnvironmentFromYaml(yamlFile, **kwargs)
        return cnoid

    @classmethod
    def setCnoidFromYaml(cls, yamlFile, **kwargs):
        """
        Setup project from yaml-file (classmethod of createCnoidFromYaml)

        Args:
            yamlFile (str) : File name to load
            kwargs (dict) : Keyword to pass to setup_cnoid.createCnoid

        Returns:
            irsl_choreonoid_ros.setup_cnoid : Instance of setup_cnoid

        """
        cnoid = cls()
        cnoid.createCnoidFromYaml(yamlFile, **kwargs)
        return cnoid
    @classmethod
    def setupEnvironment(cls, info, **kwargs):
        """
        Building environment (setting objects) under the WorldItem (class method)

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing objects on environment
            world (str, default='World') : Name of WorldItem, added objects under this item
            craeteWorld (boolean, default=False) : If True, creating new WorldItem
            setCamera (boolean, default=False) : If True, set camera position
            offset (cnoid.IRSLCoords.coordinates) : Offset of objects
        """
        cnoid = cls()
        cnoid.buildEnvironment(info, **kwargs)
        return cnoid
    @classmethod
    def setupCnoid(cls, info, **kwargs):
        """
        Creating project from parameters (class method)

        Args:
            info_dict ( dict['key': value] ) : Dictionary for representing the project
            addDefaultSimulator (boolean, default=True) : If True, adding new SimulatorItem if there is no instruction in info_dict
            addDefaultWorld (boolean, default=True) : If True, adding new WorldItem if there is no instruction in info_dict
            noEnvironment (boolean, default=False) : If True, not adding environment(objects). Use buildEnvironment method.

        """
        cnoid = cls()
        cnoid.createCnoid(info, **kwargs)
        return cnoid
    def startSimulator(self, realTime=None):
        """
        Starting simulation

        Args:
            realTime (boolean, default=None) : If True, simulator will run with realtime sync mode

        """
        if self.simulator is not None:
            if realTime is not None:
                self.simulator.setRealtimeSyncMode(realTime)
            ItemTreeView.instance.checkItem(self.simulator)
            ItemTreeView.instance.selectItem(self.simulator)
            # self.simulator.startSimulation(True)
            SimulationBar.instance.startSimulation(True)## doRest=True

    def _addWorld(self, name='World', check=True, param=None):
        wd = self.root_item.findItem(name)
        if wd is None:
            if self.world_item is None:
                self.world_item = WorldItem()
                self.world_item.name = name
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

    def _addRobot(self, info=None, ros_enable=False, worldItem=None, offset=None):
        if worldItem is not None:
            bi = _BodyItemWrapper(worldItem, offset=offset)
        else:
            bi = _BodyItemWrapper(self.world_item, offset=offset)
        bi.addRobot(info, ros_enable=ros_enable)

    def _addObject(self, info=None, worldItem=None, offset=None):
        if worldItem is not None:
            bi = _BodyItemWrapper(worldItem, offset=offset)
        else:
            bi = _BodyItemWrapper(self.world_item, offset=offset)
        bi.addObject(info)

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
        cds_ = _getDictValue(param, ('position', 'Position', 'coords', 'coordinates'))
        if cds_ is not None:
            cds_ = make_coordinates(cds_)
            ib.setCameraCoords(cds_, fov_)
        else:
            eye_ = _getDictValue(param, ('lookEye', 'eye', 'Eye'))
            up_ = _getDictValue(param, ('lookUp', 'up', 'Up'))
            if eye_ is not None and up_ is not None:
                dir_ = _getDictValue(param, ('lookForDirection', 'lookDirection', 'Direction', 'direction'))
                if dir_ is not None:
                    cds = ib.cameraPositionLookingFor(eye_, dir_, up_, opencv=False)
                    ib.setCameraCoords(cds, fov_, opencv=False)
                else:
                    center_ = _getDictValue(param, ('lookAtCenter', 'lookAt', 'at', 'center'))
                    if center_ is not None:
                        cds = ib.cameraPositionLookingAt(eye_, center_, up_, opencv=False)
                        ib.setCameraCoords(cds, fov_, opencv=False)

    def _execROSScript(self, param=None):
        if param is None:
            return
        urdf_ = _getDictValue(param, ('urdf_settings', 'URDFSettings', 'URDF_settings', 'urdf_setting', 'URDFSetting', 'URDF_setting', 'URDF', 'urdf'))
        if urdf_ is not None:
            self._parseURDF(urdf_)
        parameters_ = _getDictValue(param, ('set_parameter', 'set_parameters', 'setParameter', 'setParameters', 'parameters', 'Parameters'))
        if parameters_ is not None:
            self._parseROSParam(parameters_)
        gen_set_ = _getDictValue(param, ('generate_settings', 'generateSettings', 'generate'))
        if gen_set_ is not None:
            self._generateROSParam(gen_set_)

    def _parseURDF(self, param):
        pass

    def _parseROSParam(self, param):
        if type(param) is not list and type(param) is not tuple:
            ####
            return
        for one in param:
            if type(one) is dict:
                self._parseSingleParam(one)

    def _generateROSParam(self, param):
        model_ = _getDictValue(param, ('robot', 'model', 'Model', 'file', 'File', 'body', 'Body', 'model_file', 'modelFile'))
        if model_ is None:
            return
        nspace_ = _getDictValue(param, ('ns', 'name_space', 'nameSpace', 'robotName', 'name'))

        fname = parseURLROS(model_)
        robot_ = iu.loadRobot(fname)

        if nspace_ is None:
            if len(model_.name) > 0:
                nspace_ = robot_.name
            else:
                nspace_ = robot_.modelName

        if type(param) is dict:
            cont_list_ = _getDictValue(param, ('controllers', 'Controllers'))
        else:
            cont_list_ = param

        urdf_str = _generate_joint_urdf_header(name=nspace_) ## TODO param
        cont_param = _generate_roscontrol_config_base()## TODO param

        for cont_ in cont_list_:
            nm_   = _getDictValue(cont_, ('name', 'Name', 'controller_name', 'controllerName'))
            tp_   = _getDictValue(cont_, ('type', 'Type', 'controller_type', 'controllerType'))
            jnms_ = _getDictValue(cont_, ('joints', 'joint_list', 'joint_names', 'jointList', 'jointNames'))
            if nm_ is None:
                continue
            if jnms_ is None or ( type(jnms_) is str and jnms_.lower() == 'all' ):
                jlst_ = robot_.joints
            else:
                jlst_ = [ robot_.joint(j) for j in jnms_ ]
            if tp_ is not None:
                res = _generate_roscontrol_config(jlst_, controller_type=tp_.lower())
                cont_param[nm_] = res
                urdf_str += _generate_joint_urdf_joint(jlst_, interface_type=tp_.capitalize())
            else:
                res = _generate_roscontrol_config(jlst_)
                cont_param[nm_] = res
                urdf_str += _generate_joint_urdf_joint(jlst_)
        urdf_str += _generate_joint_urdf_footer()

        rospy.set_param('{}'.format(nspace_), cont_param)
        rospy.set_param('{}/robot_description'.format(nspace_), urdf_str)

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

    def _addScriptItem(self, param):
        script = PythonSimScriptItem()
        filename = _getDictValue(param, ('file', 'file_name', 'filename', 'File', 'FileName', 'script', 'Script'))
        if filename is not None:
            script.load(filename)
            # script.setExecutionTiming(SimulationScriptItem.ExecutionTiming.AFTER_INITIALIZATION)
            # script.setBackgroundMode(False)
            _applyParameter(script, param)
            self.world_item.addChildItem(script)## ? insert
        itemTreeView.checkItem(script)

def _generate_roscontrol_config_base(joint_state_publish_rate=50):
    # controller_type: 'position', 'effort', 'velocity'
    param = {}
    param['joint_state_controller'] = {'type': 'joint_state_controller/JointStateController',
                                       'publish_rate': joint_state_publish_rate}
    return param

def _generate_roscontrol_config(jointList, gain_p=0.0, gain_i=0.0, gain_d=0.0, controller_type='position'):
    # controller_type: 'position', 'effort', 'velocity'
    controller_param = {'type': '{}_controllers/JointTrajectoryController'.format(controller_type) }
    names = [] # names
    gains_param = {}
    for j in jointList:
        if j is None:
            continue
        nm = j.jointName
        names.append( nm )
        gains_param[nm] = {'p': gain_p, 'd': gain_d, 'i': gain_i }
    controller_param['joints'] = names
    controller_param['gains'] = gains_param

    return controller_param

def _generate_joint_urdf_header(robot=None, name=None):
    if name is None:
        if robot is None:
            name = 'robot'
        else:
            if len(robot.name) > 0:
                name = robot.name
            else:
                name = robot.modelName
    res = '<?xml version="1.0" ?>\n<robot name="{}">\n'.format(name)
    res += '<link name="root" />\n'
    return res

def _generate_joint_urdf_footer():
    return '</robot>\n'

def _generate_joint_urdf_joint(jointList, interface_type='Position'):
    # controller_type: 'Position', 'Effort', 'Velocity'

    res = ''
    for j in jointList:
        if j is None:
            continue
        ##
        q_upper = j.q_upper
        q_lower = j.q_lower
        if j.q_upper >= float('inf'):
            q_upper = sys.float_info.max
        if j.q_lower <= float('-inf'):
            q_lower = -sys.float_info.max
        ##
        jname = j.jointName
        res += '<link name="link_{}" />\n'.format(jname)

        res += '<joint name="{}" type="revolute">\n'.format(jname)
        res += '  <parent link="root" />\n'
        res += '  <child  link="link_{}" />\n'.format(jname)
        res += '  <limit lower="{}" upper="{}" effort="{}" velocity="{}" />\n'.format(q_lower, q_upper,
                                                                                      j.u_upper if j.u_upper < math.fabs(j.u_lower) else math.fabs(j.u_lower),
                                                                                      j.dq_upper if j.dq_upper < math.fabs(j.dq_lower) else math.fabs(j.dq_lower))
        res += '</joint>\n'.format(jname)

        res += '<transmission name="{}_trans">\n'.format(jname)
        res += '  <type>transmission_interface/SimpleTransmission</type>\n'
        res += '  <joint name="{}">\n'.format(jname)
        res += '    <hardwareInterface>hardware_interface/{}JointInterface</hardwareInterface>\n'.format(interface_type)
        res += '  </joint>\n'
        res += '  <actuator name="{}_motor">\n'.format(jname)
        res += '    <mechanicalReduction>1</mechanicalReduction>\n'
        res += '  </actuator>\n'
        res += '</transmission>\n'

    return res
