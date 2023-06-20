from irsl_choreonoid_ros.RobotInterface import RobotInterface
import time

def stop_robot(ri):
    is_stop = False
    while not is_stop:
        tm, tof_data = ri.waitNextData('TOFsensor', timeout=1.0, clear=True)
        if tof_data.data < 0.25:
            ri.move_velocity(0, 0, 0)
            is_stop = True
        else :
            ri.move_velocity(0, -0.05, 0)
            time.sleep(0.1)

if __name__ == '__main__':
    ri = RobotInterface('fullset_robot_robotinterface.yaml')
    stop_robot(ri)
