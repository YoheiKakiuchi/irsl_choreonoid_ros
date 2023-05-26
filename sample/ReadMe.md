# サンプル動作方法

いずれもirsl_docker_irsl_systemで動作させることを前提とする．

Terminal 1
```
./run_sim_local.sh
source /choreonoid_ws/install/setup.bash
roslaunch  $(dirname $(which choreonoid))/../share/irsl_choreonoid_ros/launch/sim_robot.launch project_file:=/userdir/fullset_robot.cnoid model:=/userdir/fullset_robot.urdf control_config:=/userdir/fullset_robot_roscontrol.yaml controllers:="joint_controller wheel_controller"
```

Terminal 2
```
docker exec -it docker_irsl_system bash
source /opt/ros/noetic/setup.bash
source /choreonoid_ws/install/setup.bash
rosrun irsl_choreonoid_ros tof_converter_node.py 
```

Terminal 3
```
docker exec -it docker_irsl_system bash
source /opt/ros/noetic/setup.bash
source /choreonoid_ws/install/setup.bash
rosrun irsl_choreonoid_ros wheelcontroller.py _wheelconfigfile:=fullset_robot_wheel.yaml 
```
Terminal 4
```
docker exec -it docker_irsl_system bash
source /opt/ros/noetic/setup.bash
source /choreonoid_ws/install/setup.bash
python3 stop_robot.py TOFsensor:=/AssembleRobot/TOF_Sensor0/value cmd_vel:=/AssembleRobot/cmd_vel
```

# Know-how
- bodyファイルのアクチュエーター中のをlimitを-inf, infへ変更
- urdfファイル中ののlimitを-100pi[rad],100pi[rad]へ手動変更