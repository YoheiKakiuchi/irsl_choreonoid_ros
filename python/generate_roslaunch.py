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
            prog='generate_roslaunch.py', # プログラム名
            usage='Demonstration of cnoid_dump_model', # プログラムの利用方法

            add_help=True, # -h/–help オプションの追加
            )
    parser.add_argument('--bodyfile', type=str, required=True)
    parser.add_argument('--use_wheel', type=strtobool, default=False)
    parser.add_argument('--controllers', type=str, default='joint_controller joint_state_controller')
    parser.add_argument('--demo_base_dir', type=str, default='/userdir')
    parser.add_argument('--urdffile', type=str, required=True)
    parser.add_argument('--roscontrolfile', type=str, required=True)
    parser.add_argument('--cnoidfile', type=str, required=True)
    
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
    
    print('<launch>')
    print('  <arg name="demo_base_dir" default="/userdir"/>')
    print('  <!-- choreonoid -->')
    print('  <arg name="project_file" default="$(arg demo_base_dir)/{}"/>'.format(args.cnoidfile))
    print('  <arg name="robot_name" default="{}"/>'.format(robotname))
    print('  <!-- ros_control -->')
    print('  <arg name="model" default="$(arg demo_base_dir)/{}"/>'.format(args.urdffile))
    print('  <arg name="control_config" default="$(arg demo_base_dir)/{}"/>'.format(args.roscontrolfile))
    print('  <arg name="controllers" default="{}" />'.format(args.controllers))
    # print('  <!-- joint_controller wheel_controller joint_state_controller -->')
    if args.use_wheel:
        print('  <!-- wheel -->')
        print('  <arg name="wheelconfigfile" default="$(arg demo_base_dir)/wheel.yaml"/>')
    print('')
    print('  <include file="$(find irsl_choreonoid_ros)/launch/sim_robot.launch">')
    print('    <arg name="project_file" value="$(arg project_file)" />')
    print('    <arg name="robot_name" default="$(arg robot_name)"/>')
    print('    <arg name="model" value="$(arg model)" />')
    print('    <arg name="control_config" value="$(arg control_config)" />')
    print('    <arg name="controllers" value="$(arg controllers)" />')
    print('  </include>')
    print('')
    print('  <group ns="$(arg robot_name)" >')
    for idx in range(num_device):
        dev = rbody.getDevice(idx)
        if dev.getName().lower().find('tof')>=0 or dev.getName().lower().find('ultra')>=0 :
            print('    <node name="tof_converter_node_{}" pkg="irsl_choreonoid_ros" type="tof_converter_node.py" output="screen">'.format(idx))
            print('      <remap from="input_points" to="{}/point_cloud"/>'.format(dev.getName()))
            print('      <remap from="output_sensor_data" to="{}/value"/>'.format(dev.getName()))
            print('    </node>')
            print('')
        elif dev.getName().lower().find('color')>=0:
            print('    <node name="colorsensor_converter_node_{}" pkg="irsl_choreonoid_ros" type="colorsensor_converter_node.py" output="screen">'.format(idx))
            print('      <remap from="input_image" to="{}/color/image_raw"/>'.foramt(dev.getName()))
            print('      <remap from="output_sensor_data" to="{}/value"/>'.format(dev.getName()))
            print('    </node>')
            print('')
    if args.use_wheel:
        print('    <node name="wheelcontroller_node" pkg="irsl_choreonoid_ros" type="wheelcontroller.py" output="screen">')
        print('      <param name="wheelconfigfile" value="$(arg wheelconfigfile)" />')
        print('    </node>')
    print('  </group>')
    print('</launch>')
