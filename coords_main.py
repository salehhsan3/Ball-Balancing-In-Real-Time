
import numpy as np
import cv2.aruco as aruco
import os
from time import time
from math import fmod

# Append the parent directory of the current script's directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.CameraUtils.CameraStreamer import *
from src.MotionUtils import kinematicsUtils as fk
from src.Robot.RTDERobot import *
from src.CameraUtils.CameraFunctions import *


print("here")
camera = CameraStreamer()
task_robot = RTDERobot("192.168.0.12",config_filename="control_loop_configuration.xml")
camera_robot = RTDERobot("192.168.0.10",config_filename="control_loop_configuration.xml")

def get_world_position_from_buffers(pixel_x, pixel_y, depth_frame, depth_intrinsic):
    depth = depth_frame.get_distance(pixel_x, pixel_y)

    # Convert pixel coordinates to world coordinates (from the Camera's Perspective!)
    ball_position = rs.rs2_deproject_pixel_to_point(depth_intrinsic, [pixel_x, pixel_y], depth)
    return np.array(ball_position)

def get_position():
    color_image, depth_image, depth_frame, depth_colormap, depth_intrinsics, is_new_image = camera.get_frames()
    if color_image is None or depth_image is None:
        # print("None Frames")
        return None  # Return None to indicate an error state

    if color_image.size == 0 or depth_image.size == 0:
        print("Can't receive frame (stream end?).")
        return None  # Return None to indicate an error state

    positions = detect_object(color_image)
    if len(positions) == 0:
        print('No object detected!')
        return None

    x1, y1, x2, y2 = positions[0]
    ball_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
    depth_value = camera.depth_frame.get_distance(ball_center[0], ball_center[1])
    distance_meters = depth_value * 1000  # Convert to millimeters
    camera_world_coords = camera.get_world_position_from_camera(ball_center[0], ball_center[1])

    color = (0, 255, 0)  # Green for ball
    cv2.rectangle((color_image), (x1, y1), (x2, y2), color, 2)
    cv2.circle((color_image), ball_center, 2 ,color,2)
    cv2.putText((color_image), f'Ball Distance: {distance_meters:.2f} mm,', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.putText((color_image), f'Ball Center: ({ball_center[0]}, {ball_center[1]}),', (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.putText((color_image), f'Ball World Coordinates: {camera_world_coords}', (x1, y1 - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    # camera.stream_frames(color_image, depth_colormap)
    plate_center, aruco_image = camera.detect_arucos(color_image)
    print("Ball Center: ", ball_center)
    print("Plate Center: ", plate_center)
    return get_world_position_from_buffers(ball_center[0],ball_center[1], depth_frame,depth_intrinsics)

last_pos = [0,0,0,0]
i = 0
print("start!")
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

    ball_position = get_position()
    if ball_position is None:
        continue
    i += 1

    if i > 50:
        plate_pos = fk.ur3e_effector_to_home(current_task_config,fk.plate_from_ee([0,0,0,1]))
        ball_pos = fk.ur5e_effector_to_home(current_cam_config, fk.camera_from_ee(ball_position))
        print("ball_pos: ", ball_pos, [round(num,4) for num in (ball_pos - last_pos)])
        last_pos = ball_pos
        # camera.stream_frames()
        # print("difference: ", plate_pos - ball_pos)
        i = 0
