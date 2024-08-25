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
            prog='generate_dynamixel_config.py', # プログラム名
            usage='', # プログラムの利用方法
            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    parser.add_argument('--wheeljoints', nargs="*", type=str, default=[])    
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
    
    num_joint = rbody.getNumJoints()
    joint_list = []
    for idx in range(num_joint):
        joint = rbody.getJoint(idx)
        joint_list.append(joint)

    for jointname in [j.jointName for j in joint_list]:
        print("{}:".format(jointname))
        print("  ID: XX # input your motor id")
        print("  Return_Delay_Time: 0")
        print("  Operating_Mode: {}".format('3' if jointname not in args.wheeljoints else '1'))