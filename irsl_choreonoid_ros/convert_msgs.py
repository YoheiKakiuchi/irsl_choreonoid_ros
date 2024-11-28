# from tf import transformations
import std_msgs.msg
import geometry_msgs.msg

import numpy

from cnoid.IRSLCoords import coordinates

###
____to_function_map__ = {}
__from_function_map__ = {}

def convertToROSMsg(pyexpr, class_rosmsg, **kwargs):
    """
    """
    if not class_rosmsg._md5sum in ____to_function_map__:
        raise Exception('class found')
    return ____to_function_map__[class_rosmsg._md5sum](pyexpr, **kwargs)

def convertFromROSMsg(rosmsg):
    """
    """
    if not rosmsg.__class__._md5sum in __from_function_map__:
        raise Exception('class found')
    return __from_function_map__[rosmsg.__class__._md5sum](rosmsg)

##geometry_msgs/Vector3
def _to_vector3(pyexpr, **kwargs):
    return geometry_msgs.msg.Vector3(x=pyexpr[0], y=pyexpr[1], z=pyexpr[2])
def _from_vector3(rosmsg):
    return numpy.array((rosmsg.x, rosmsg.y, rosmsg.z), dtype='float64')

##geometry_msgs/Vector3Stamped
def _to_vector3_stamped(pyexpr, **kwargs):
    return geometry_msgs.msg.Vector3Stamped(vector=_to_vector3(pyexpr), **kwargs)
def _from_vector3_stamped(rosmsg):
    hdr = rosmsg.header
    res = _from_vector3(rosmsg.vector)
    return (hdr, res)

##geometry_msgs/Quaternion
def _to_quaternion(pyexpr, **kwargs):
    return geometry_msgs.msg.Quaternion(x=pyexpr[0], y=pyexpr[1], z=pyexpr[2], w=pyexpr[3])
def _from_quaternion(rosmsg):
    return numpy.array((rosmsg.x, rosmsg.y, rosmsg.z, rosmsg.w), dtype='float64')

##geometry_msgs/QuaternionStamped
def _to_quaternion_stamped(pyexpr, **kwargs):
    return geometry_msgs.msg.QuaternionStamped(quaternion=_to_quaternion(pyexpr), **kwargs)
def _from_quaternion_stamped(rosmsg):
    hdr = rosmsg.header
    res = _from_quaternion(rosmsg.vector)
    return (hdr, res)

##geometry_msgs/Point
def _to_point(pyexpr, **kwargs):
    return geometry_msgs.msg.Point(x=pyexpr[0], y=pyexpr[1], z=pyexpr[2])
def _from_point(rosmsg):
    return numpy.array((rosmsg.x, rosmsg.y, rosmsg.z), dtype='float64')

##geometry_msgs/PointStamped
def _to_point_stamped(pyexpr, **kwargs):
    return geometry_msgs.msg.PointStamped(point=_to_point(pyexpr), **kwargs)
def _from_point_stamped(rosmsg):
    hdr = rosmsg.header
    res = _from_point(rosmsg.vector)
    return (hdr, res)

##geometry_msgs/Pose
def _to_pose(pyexpr, **kwargs):
    return geometry_msgs.msg.Pose(position = _to_point(pyexpr.pos),
                                  orientation = _to_quaternion(pyexpr.quaternion))
def _from_pose(rosmsg):
    return coordinates(_from_point(rosmsg.position), _from_quaternion(rosmsg.orientation))

##geometry_msgs/PoseStamped
def _to_pose_stamped(pyexpr, **kwargs):
    return geometry_msgs.msg.PoseStamped(pose = _to_pose(pyexpr), **kwargs)
def _from_pose_stamped(rosmsg):
    return rosmsg.header, _from_pose(rosmsg.point)

##geometry_msgs/Transform
def _to_transform(pyexpr, **kwargs):
    return geometry_msgs.msg.Transform(translation = _to_vector3(pyexpr.pos),
                                       rotation = _to_quaternion(pyexpr.quaternion))
def _from_transform(rosmsg):
    return coordinates(_from_vector3(rosmsg.translation), _from_quaternion(rosmsg.rotation))

##geometry_msgs/TransformStamped
def _to_transform_stamped(pyexpr, **kwargs):
    return geometry_msgs.msg.TransformStamped(transform = _to_transform(pyexpr), **kwargs)
def _from_transform_stamped(rosmsg):
    return rosmsg.header, _from_transform(rosmsg.point)

# geometry_msgs/Twist
def _to_twist(pyexpr, **kwargs):
    return geometry_msgs.msg.Twist(linear=_to_vector3(pyexpr[0:3]), angular=_to_vector3(pyexpr[3:]))
def _from_twist(rosmsg):
    return numpy.array((rosmsg.linear.x, rosmsg.linear.y, rosmsg.linear.z,
                        rosmsg.angular.x, rosmsg.angular.y, rosmsg.angular.z), dtype='float64')

# geometry_msgs/TwistStamped
def _to_twist_stamped(pyexpr, **kwargs):
    return geometry_msgs.msg.TwistStamped(twist = _to_twist(pyexpr), **kwargs)
def _from_twist_stamped(rosmsg):
    return rosmsg.header, _from_twist(rosmsg.twist)

# geometry_msgs/Accel
def _to_accel(pyexpr, **kwargs):
    return geometry_msgs.msg.Accel(linear=_to_vector3(pyexpr[0:3]), angular=_to_vector3(pyexpr[3:]))
def _from_accel(rosmsg):
    return numpy.array((rosmsg.linear.x, rosmsg.linear.y, rosmsg.linear.z,
                        rosmsg.angular.x, rosmsg.angular.y, rosmsg.angular.z), dtype='float64')

# geometry_msgs/AccelStamped
def _to_accel_stamped(pyexpr, **kwargs):
    return geometry_msgs.msg.AccelStamped(accel = _to_accel(pyexpr), **kwargs)
def _from_accel_stamped(rosmsg):
    return rosmsg.header, _from_accel(rosmsg.accel)

# geometry_msgs/Wrench
def _to_wrench(pyexpr, **kwargs):
    return geometry_msgs.msg.Wrench(force=_to_vector3(pyexpr[0:3]), torque=_to_vector3(pyexpr[3:]))
def _from_wrench(rosmsg):
    return numpy.array((rosmsg.force.x, rosmsg.force.y, rosmsg.force.z,
                        rosmsg.torque.x, rosmsg.torque.y, rosmsg.torque.z), dtype='float64')

# geometry_msgs/WrenchStamped
def _to_wrench_stamped(pyexpr, **kwargs):
    return geometry_msgs.msg.WrenchStamped(wrench = _to_wrench(pyexpr), **kwargs)
def _from_wrench_stamped(rosmsg):
    return rosmsg.header, _from_wrench(rosmsg.wrench)

##geometry_msgs/PoseWithCovariance
##geometry_msgs/PoseWithCovarianceStamped
##
##geometry_msgs/AccelWithCovariance
##geometry_msgs/AccelWithCovarianceStamped
##
##geometry_msgs/TwistWithCovariance
##geometry_msgs/TwistWithCovarianceStamped
##
##geometry_msgs/Inertia
##geometry_msgs/InertiaStamped
##
## not implemented
##geometry_msgs/Point32
##geometry_msgs/Pose2D
##geometry_msgs/PoseArray
##geometry_msgs/Polygon
##geometry_msgs/PolygonStamped
# std_msgs/ColorRGBA
# nav_msgs/Odom

def checkLayout(MultiArrayLayout_msg):
    dims = MultiArrayLayout_msg.dim
    ## MultiArrayLayout_msg.data_offset
    dim_size = len(dims)
    cur_stride = 1
    for dim in dims[::-1]: ## reverse iterator
        if dim.size < 1:
            raise Exception('dim[{}] == {} is less than 0, ', dim.label, dim.size)
        cur_stride *= dim.size
        if dim.stride != cur_stride:
            raise Exception('dim[{}].stride == {} is not qeual to culculated {}', dim.label, dim.stride, cur_stride)
    return cur_stride - MultiArrayLayout_msg.data_offset

def makeLayout(nparray):
    _shape = nparray.shape
    dim_size = len(_shape)
    cntr = 0
    cur_stride = 1
    dims = []
    for sz in _shape[::-1]: ## reverse iterator
        dim = MultiArrayDimension
        dim.size   = sz
        dim.stride = cur_stride * sz
        dim.label  = 'dim{}'.format(dim_size - cntr)
        dims.append(dim)
        cntr += 1
    return MultiArrayLayout(dim = reversed(dims), data_stride = 0)

def convertToMultiArray(nparray, ary_cls):
    layout = makeLayout(nparray)
    res = ary_cls(data=nparray.reshape((1, -1))[0].tolist(), layout=layout)
    return res

def convertFromMultiArray(rosmsg):
    pass
    #res = numpy.array(rosmsg.data)
    #res = res.reshape()
    #return res

def generateConversionMap():
    ____to_function_map__[ geometry_msgs.msg.Twist._md5sum ] = _to_twist
    __from_function_map__[ geometry_msgs.msg.Twist._md5sum ] = _from_twist
    ____to_function_map__[ geometry_msgs.msg.TwistStamped._md5sum ] = _to_twist_stamped
    __from_function_map__[ geometry_msgs.msg.TwistStamped._md5sum ] = _from_twist_stamped
    ____to_function_map__[ geometry_msgs.msg.Accel._md5sum ] = _to_accel
    __from_function_map__[ geometry_msgs.msg.Accel._md5sum ] = _from_accel
    ____to_function_map__[ geometry_msgs.msg.AccelStamped._md5sum ] = _to_accel_stamped
    __from_function_map__[ geometry_msgs.msg.AccelStamped._md5sum ] = _from_accel_stamped
    ____to_function_map__[ geometry_msgs.msg.Wrench._md5sum ] = _to_wrench
    __from_function_map__[ geometry_msgs.msg.Wrench._md5sum ] = _from_wrench
    ____to_function_map__[ geometry_msgs.msg.WrenchStamped._md5sum ] = _to_wrench_stamped
    __from_function_map__[ geometry_msgs.msg.WrenchStamped._md5sum ] = _from_wrench_stamped
    ____to_function_map__[ geometry_msgs.msg.Vector3._md5sum ] = _to_vector3
    __from_function_map__[ geometry_msgs.msg.Vector3._md5sum ] = _from_vector3
    ____to_function_map__[ geometry_msgs.msg.Vector3Stamped._md5sum ] = _to_vector3_stamped
    __from_function_map__[ geometry_msgs.msg.Vector3Stamped._md5sum ] = _from_vector3_stamped
    ____to_function_map__[ geometry_msgs.msg.Point._md5sum ] = _to_point
    __from_function_map__[ geometry_msgs.msg.Point._md5sum ] = _from_point
    ____to_function_map__[ geometry_msgs.msg.PointStamped._md5sum ] = _to_point_stamped
    __from_function_map__[ geometry_msgs.msg.PointStamped._md5sum ] = _from_point_stamped
    ____to_function_map__[ geometry_msgs.msg.Quaternion._md5sum ] = _to_quaternion
    __from_function_map__[ geometry_msgs.msg.Quaternion._md5sum ] = _from_quaternion
    ____to_function_map__[ geometry_msgs.msg.QuaternionStamped._md5sum ] = _to_quaternion_stamped
    __from_function_map__[ geometry_msgs.msg.QuaternionStamped._md5sum ] = _from_quaternion_stamped
    ____to_function_map__[ geometry_msgs.msg.Pose._md5sum ] = _to_pose
    __from_function_map__[ geometry_msgs.msg.Pose._md5sum ] = _from_pose
    ____to_function_map__[ geometry_msgs.msg.PoseStamped._md5sum ] = _to_pose_stamped
    __from_function_map__[ geometry_msgs.msg.PoseStamped._md5sum ] = _from_pose_stamped
    ____to_function_map__[ geometry_msgs.msg.Transform._md5sum ] = _to_transform
    __from_function_map__[ geometry_msgs.msg.Transform._md5sum ] = _from_transform
    ____to_function_map__[ geometry_msgs.msg.TransformStamped._md5sum ] = _to_transform_stamped
    __from_function_map__[ geometry_msgs.msg.TransformStamped._md5sum ] = _from_transform_stamped

generateConversionMap()
