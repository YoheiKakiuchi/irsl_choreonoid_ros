#!/bin/python3
import argparse
import numpy
import os
import sys

from distutils.util import strtobool

try:
    import cnoid.Body
    import cnoid.Util
except ImportError:
    import sys
    import shutil
    choreonoid_path = os.path.join(os.path.dirname(shutil.which('choreonoid')), '../lib/choreonoid-2.0/python') if shutil.which('choreonoid') is not None else None
    if choreonoid_path is None:
        print('Error: choreonoid_path not found.', file=sys.stderr)
        sys.exit(1)
    sys.path.append(choreonoid_path)
    import cnoid.Body
    import cnoid.Util


if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='generate_ri_config.py', # プログラム名
            usage='Demonstration of cnoid_dump_model', # プログラムの利用方法

            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--controller_name', type=str, default="wheel_controller")
    parser.add_argument('--wheeljoints', nargs="*", type=str, default=[])
    parser.add_argument('--wheel_radius', type=float, default=0.024)
    args = parser.parse_args()
    
    fname = args.bodyfile
    if not os.path.isfile(str(fname)):
        print("File is not exist.", file=sys.stderr)
        print("Please check file : {}".format(fname), file=sys.stderr)
        exit(1)
    rbody = cnoid.Body.BodyLoader().load(str(fname))
    if rbody is None:
        print("File is broken.", file=sys.stderr)
        print("Please check file : {}".format(fname), file=sys.stderr)
        exit(1)
    
    rbody.updateLinkTree()
    rbody.initializePosition()
    rbody.calcForwardKinematics()

    joint_list = []

    num_link = rbody.getNumLinks()
    num_joint = rbody.getNumJoints()
    num_device = rbody.getNumDevices()

    for idx in range(num_joint):
        joint = rbody.getJoint(idx)
        joint_list.append(joint)
        
    robotname = rbody.getModelName()
        
    print("controller_name: {}".format(args.controller_name))
    print("robot_name: {}".format(robotname))
    print("wheels:")
    for wj_name in args.wheeljoints:
        for joint in joint_list:
            if wj_name == joint.jointName:
                print("- name: {}".format(joint.jointName))
                position = joint.getPosition()
                print("  rot_axis:")
                print("  - {}".format(position[0,2]))
                print("  - {}".format(position[1,2]))
                print("  - {}".format(position[2,2]))
                print("  translation:")
                print("  - {}".format(position[0,3]))
                print("  - {}".format(position[1,3]))
                print("  - {}".format(position[2,3]))
                print("  wheel_radius: {}".format(args.wheel_radius))