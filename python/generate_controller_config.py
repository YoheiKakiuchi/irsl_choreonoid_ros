#!/usr/bin/python3
import argparse
import numpy
import os
import sys

from distutils.util import strtobool

if __name__=='__main__':
    parser = argparse.ArgumentParser(
            prog='generate controller config', # プログラム名
            usage='', # プログラムの利用方法
            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--use_wheel', type=strtobool, default=False)
    
    args = parser.parse_args()

    default_configs = ["dxl_read_period: 0.01",
    "dxl_write_period: 0.01",
    "publish_period: 0.01",]

    wheel_configs =["# wheel_controller_setting",
    " mobile_robot_config:",
    "   actuator_id: # Input your wheel motor id",
    "   - MOTOR_ID1" ,
    "   - MOTOR_ID2",
    "   - MOTOR_ID3",
    "   - MOTOR_ID4",
    "   actuator_mounting_angle:",
    "   - -1.57079632679",
    "   - 0.0",
    "   - 1.57079632679",
    "   - 3.14159265359",
    "   omni_mode: true",
    "   radius_of_wheel: 0.024",
    "   seperation_between_wheels: X.XX # Input your wheel base"]

    for s in default_configs:
        print(s)
    print("")
    for s in wheel_configs:
        print("{}".format("" if args.use_wheel else "# ")+s)
