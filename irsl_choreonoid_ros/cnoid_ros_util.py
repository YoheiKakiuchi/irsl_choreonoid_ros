import rospy
import roslib.packages

from urllib.parse import urlparse
from irsl_choreonoid.cnoid_util import parseURL

def parseURLROS(url):
    """parseURLROS

    Refere parseURL in irsl_choreonoid.cnoid_util

    Args:
        url (str): url

    Returns:
        str: Absolute path

    Examples:
        >>> parseURLROS('package://ros_package_name/dir/file')
        /catkin_ws/install/share/ros_package_name/dir/file

    """
    try:
        return parseURL(url)
    except Exception as e:
        res = urlparse(url)
        if res.scheme == 'package':
            return roslib.packages.get_pkg_dir(res.netloc) + res.path
        elif res.scheme == 'rosparam':
            raise SyntaxError('not implemented scheme {} / {}'.format(res.scheme, url))
            if res.netloc == '':
                return rospy.get_param(res.path)
            elif res.netloc == '.':
                ## relative
                path = res.path
                if path[0] == '/':
                    path = path[1:]
                return rospy.get_param(path)
            elif res.netloc == '~':
                path = res.path
                if path[0] == '/':
                    path = path[1:]
                return rospy.get_param(res.netloc + path)
        else:
            raise e
