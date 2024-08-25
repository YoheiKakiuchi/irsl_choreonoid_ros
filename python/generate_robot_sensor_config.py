#!/usr/bin/python3
import argparse
import numpy
import os
import sys

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
            prog='generate controller config', # プログラム名
            usage='', # プログラムの利用方法
            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, default="robotname.body")
    
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
    
    num_device = rbody.getNumDevices()

    print("I2CHubPublisher:")
    print("    'address': '0x70'")
    for idx in range(num_device):
        dev = rbody.getDevice(idx)
        print("    '{}':".format(idx))
        print("        address: 'XXXX' # Input sensor address. color sensor is 0x70. Other sensor is 0x29.")
        print("        name: XXXX # input sensor type. ex. ColorSensorPublisher, TOFPublisher " )
        print("        topic_name: XXXX/value # Input sensor topic name")