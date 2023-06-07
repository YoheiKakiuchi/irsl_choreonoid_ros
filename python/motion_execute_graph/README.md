# MotionExecuteGraphGenerator

## Files

- MotionExecuteGraphGenerator.py
main library to generate smach-definition and template of motion programs

- gen_sample.py
sample for using MotionExecuteGraphGenerator

- sample.dot
sample graph

- motion_exec_graph_demo.py

- motion_exec_graph_ros_demo.py

## Usage

### Generate
```
python3 gen_sample.py
```

motionlib.py and smach_definition.py would be generated.

### Demo
```
python3 motion_exec_graph_demo.py
```

### Demo(ROS)

```
roscore
```

launch viewer
```
rosrun smach_viewer smach_viewer.py
```

run demo
```
python3 motion_exec_graph_ros_demo.py
```
