from irsl_choreonoid_ros.RobotInterface import RobotInterface
import time

def stop_robot(ri):
    is_stop = False
    while not is_stop:
        tm, color_data = ri.waitNextData('COLORSensor', timeout=1.0, clear=True)
        if color_data.r > 0.8 and color_data.g > 0.8 and color_data.b > 0.8:
            ri.move_velocity(0, 0, 0)
            is_stop = True
        else :
            ri.move_velocity(0, 0.05, 0)
            time.sleep(0.1)

if __name__ == '__main__':
    ri = RobotInterface('fullset_robot_robotinterface.yaml')
    stop_robot(ri)
