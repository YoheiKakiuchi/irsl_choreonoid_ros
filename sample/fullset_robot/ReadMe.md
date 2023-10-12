# サンプル動作方法

いずれもirsl_docker_irsl_systemで動作させることを前提とする．

## 障害物検出サンプル
### ROSベース
Terminal 1 (ロボット実機)
```
./run.sh
source /choreonoid_ws/install/setup.bash
roslaunch run_sim_robot.launch demo_base_dir:=/choreonoid_ws/src/irsl_choreonoid_ros/sample/
```

Terminal 2 (ユーザプログラム)
```
docker exec -it docker_irsl_system bash
source /opt/ros/noetic/setup.bash
python3 stop_robot.py TOFsensor:=/fullset_robot/TOFSensor/value cmd_vel:=/fullset_robot/cmd_vel
```

### サンプル動作方法（RI）
Terminal 1 (ロボット実機)
```
./run.sh
source /choreonoid_ws/install/setup.bash
roslaunch run_sim_robot.launch demo_base_dir:=/choreonoid_ws/src/irsl_choreonoid_ros/sample/
```

Terminal 2 (ユーザプログラム)
```
docker exec -it docker_irsl_system bash
source /choreonoid_ws/install/setup.bash
PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-1.8/python python3 stop_robot_ri.py
```

### smachプログラム生成方法
stop_robot_sm.dot作成後以下コマンドを実行．
```
python3 smach_gen_sample.py
```

### サンプル動作方法 (smach)
Terminal 1 (ロボット実機)
```
./run.sh
source /choreonoid_ws/install/setup.bash
roslaunch run_sim_robot.launch demo_base_dir:=/choreonoid_ws/src/irsl_choreonoid_ros/sample/
```

Terminal 2 (ユーザプログラム)
```
docker exec -it docker_irsl_system bash
source /choreonoid_ws/install/setup.bash
PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-1.8/python python3 stop_robot_smach.py
```

## 白線検出サンプル

### サンプル動作方法（RI）
Terminal 1 (ロボット実機)
```
./run.sh
source /choreonoid_ws/install/setup.bash
roslaunch run_sim_robot.launch demo_base_dir:=/choreonoid_ws/src/irsl_choreonoid_ros/sample/
```

Terminal 2 (ユーザプログラム)
```
docker exec -it docker_irsl_system bash
source /choreonoid_ws/install/setup.bash
PYTHONPATH=$PYTHONPATH:$(dirname $(which choreonoid))/../lib/choreonoid-1.8/python python3 line_detection_ri.py
```


# Know-how
- bodyファイルのアクチュエーター中のをlimitを-inf, infへ変更
- urdfファイル中ののlimitを-100pi[rad],100pi[rad]へ手動変更