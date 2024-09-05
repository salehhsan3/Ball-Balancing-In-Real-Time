import sys
import os

# Append the parent directory of the current script's directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.MotionUtils.motionConstants.constants import *
from src.Robot.RTDERobot import *
import src.MotionUtils.PathFollow as PathFollow

path = [[0.797, -2.788, -0.017, -0.379, -0.055, -1.566],
       [0.351, -2.031, -0.015, -1.383, 1.233, -1.548],
       [0.291, -1.088, -0.012, -2.096, 1.335, -1.574]]

# path = [[0.0, -1.571, 0.0, -1.571, 0.0, 0.0],
#         [0.797, -2.788, -0.017, -0.379, -0.055, -1.566]]

pathfollower = PathFollow.PathFollowStrict(path, TASK_PATH_LOOKAHEAD, TASK_EDGE_CUTOFF)
task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

keep_moving = True
while keep_moving:
    task_state = task_robot.getState()
    cam_state = camera_robot.getState()
    if not task_state or not cam_state:
        break
    task_robot.sendWatchdog(1)
    camera_robot.sendWatchdog(1)

    current_task_config = task_state.actual_q
    current_cam_config = cam_state.actual_q
    lookahead_config = pathfollower.getClampedLookaheadConfig(current_task_config)
    pathfollower.updateCurrentEdge(current_task_config)
    index = pathfollower.current_edge
    print(lookahead_config - current_task_config, index)

    task_robot.sendConfig(lookahead_config)



