#!/bin/bash

set -e

# See http://unix.stackexchange.com/questions/101080/realpath-command-not-found
realpath ()
{
    f=$@;
    if [ -d "$f" ]; then
        base="";
        dir="$f";
    else
        base="/$(basename "$f")";
        dir=$(dirname "$f");
    fi;
    dir=$(cd "$dir" && /bin/pwd);
    echo "$dir$base"
}


BODYFILE=''
ROSCONTROLFILE=roscontrol_config.yaml
RICONFIGFILE=robot_interface.yaml
WHEELCONFIG=wheel.yaml
WORLDSETTINGFILE=world.yaml


USE_WHEEL=False
USE_ARM=True
WHEEL_LIST=()
CONTROLLERS=("joint_controller" "joint_state_controller")
GEN_TYPE=2 ## generate world.yaml

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--body)
            BODYFILE="$2"
            shift
            shift
            ;;
        -c|--cnoid) ## generate .cnoid file
            GEN_TYPE=1
            shift
            ;;
        --wheel-joint)
            USE_WHEEL=True
            CONTROLLERS+=("wheel_controller")
            shift
            while [[ $# -gt 0 ]]; do
                tmp=$1
                if [ ${tmp:0:1} == "-" ]; then
                    exit
                fi
                WHEEL_LIST+=($tmp)
                shift
            done
            ;;
        --help)
            echo "generator_settings.sh [ -b | --body <bodyfile.body> ] [ -c | --cnoid ] [ --wheel-joint <joint_0> <joint_1> .. <joint_n>]"
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            POSITIONAL_ARGS+=("$1") # save positional arg
            shift # past argument
            ;;
    esac
done

if [ -z "$BODYFILE" ]; then
    echo "Please input body file using -b/--body <bodyfile>"
    exit
fi

if [ ! -e "$BODYFILE" ]; then
    echo "BODYFILE ${BODYFILE} does not exist!"
    exit
fi

set -x

ROBOTNAME=`python3 -c "import cnoid.Body;import sys;print(cnoid.Body.BodyLoader().load('$BODYFILE').getModelName())"`

URDFFILE=`echo $BODYFILE |sed 's/.body$/.urdf/g'`
CNOIDFILE=sim_$ROBOTNAME.cnoid

CHOREONOID_DIR_PATH=$(dirname $(which choreonoid))

wheel_length=${#WHEEL_LIST[@]}
WHEELOPTION=
if [ $wheel_length -gt 0 ] ; then
    WHEELOPTION=--wheeljoints
fi

choreonoid_body2urdf $BODYFILE > $URDFFILE 2>/dev/null

if [ $GEN_TYPE -eq 1 ] ; then
    rosrun irsl_choreonoid_ros generate_roscontroller_config.py --bodyfile $BODYFILE $WHEELOPTION ${WHEEL_LIST[@]} > $ROSCONTROLFILE
    CONTROLLERS_LIST=`python3 -c "import yaml; f=open('$ROSCONTROLFILE'); obj=yaml.safe_load(f); f.close(); [ print(str(c)+' ', end='') for c in obj.keys() ]"`
fi

if [ $wheel_length -gt 0 ] ; then
   rosrun irsl_choreonoid_ros generate_wheelconfig.py --bodyfile $BODYFILE --wheeljoints ${WHEEL_LIST[@]} > $WHEELCONFIG
fi

if [ $GEN_TYPE -eq 1 ] ; then
    rosrun irsl_choreonoid_ros generate_cnoid.py --bodyfile $BODYFILE --templatefile $CHOREONOID_DIR_PATH/../share/irsl_choreonoid_ros/config/choreonoid_ros_template.cnoid > $CNOIDFILE
fi

if [ $wheel_length -gt 0 ] ; then
    rosrun irsl_choreonoid_ros generate_ri_config.py --bodyfile $BODYFILE --use_wheel $USE_WHEEL --wheeljoints ${WHEEL_LIST[@]}  > $RICONFIGFILE
else
    rosrun irsl_choreonoid_ros generate_ri_config.py --bodyfile $BODYFILE --use_wheel $USE_WHEEL > $RICONFIGFILE
fi

if [ $GEN_TYPE -eq 1 ] ; then
    rosrun irsl_choreonoid_ros generate_roslaunch.py --gen_type 1 --bodyfile $BODYFILE  --urdffile $URDFFILE --cnoidfile $CNOIDFILE --roscontrolfile $ROSCONTROLFILE --use_wheel $USE_WHEEL --controllers "$CONTROLLERS_LIST" > run_sim_robot.launch
else
    if [ $wheel_length -gt 0 ] ; then
        rosrun irsl_choreonoid_ros generate_roslaunch.py --gen_type 2 --bodyfile $BODYFILE --use_wheel True  --controllers "joint_controller wheel_controller joint_state_controller" --demo_base_dir `pwd` --urdffile $URDFFILE --worldsettings $WORLDSETTINGFILE > run_sim_robot.launch
    else
        rosrun irsl_choreonoid_ros generate_roslaunch.py --gen_type 2 --bodyfile $BODYFILE --use_wheel False --controllers "joint_controller joint_state_controller" --demo_base_dir `pwd` --urdffile $URDFFILE --worldsettings $WORLDSETTINGFILE > run_sim_robot.launch
    fi
fi

if [ $GEN_TYPE -eq 2 ] ; then
    if [ $wheel_length -gt 0 ] ; then
    rosrun irsl_choreonoid_ros generate_world_config.py  $BODYFILE --wheeljoints ${WHEEL_LIST[@]} > $WORLDSETTINGFILE
    else
    rosrun irsl_choreonoid_ros generate_world_config.py  $BODYFILE > $WORLDSETTINGFILE
    fi
fi

# generate real robot setting files
rosrun irsl_choreonoid_ros generate_controller_config.py --use_wheel $USE_WHEEL > dx_controller_config.yaml
rosrun irsl_choreonoid_ros generate_dynamixel_config.py --bodyfile $BODYFILE --wheeljoints ${WHEEL_LIST[@]} > dx_servo_config.yaml
rosrun irsl_choreonoid_ros generate_robot_sensor_config.py --bodyfile $BODYFILE > sensor_config.yaml
# rosrun irsl_choreonoid_ros generate_ros_settings.py > ros_settings.yaml
