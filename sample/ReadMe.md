# サンプル動作方法

いずれもirsl_docker_irsl_systemで動作させることを前提とする．

Terminal 1 (ロボット実機)
```
./run_sim_local.sh
source /choreonoid_ws/install/setup.bash
roslaunch run_sim_robot.launch 
```

Terminal 2 (ユーザプログラム)
```
docker exec -it docker_irsl_system bash
source /opt/ros/noetic/setup.bash
source /choreonoid_ws/install/setup.bash
python3 stop_robot.py TOFsensor:=/AssembleRobot/TOF_Sensor0/value cmd_vel:=/AssembleRobot/cmd_vel
```

# Know-how
- bodyファイルのアクチュエーター中のをlimitを-inf, infへ変更
- urdfファイル中ののlimitを-100pi[rad],100pi[rad]へ手動変更