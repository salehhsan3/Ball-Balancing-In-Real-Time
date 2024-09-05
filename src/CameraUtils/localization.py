import cv2
import numpy as np
import time
from .cameraConstants.constants import *
import logging
"""
These are helper functions for getting the position of the markers from a color image containing arucos.
Additionally, there is a function for getting pixel to aruco plane, given a constant height from the plane z.
"""
# Default value of ball height
BALL_HEIGHT = 0.035

# Aruco board constants
VALID_ARUCOS = [a for a in range(12)]
SIZE = 51.5
VERT_DISP = 23 + SIZE
HOR_DISP = 23 + SIZE
SQUARE_SHAPE = [(-SIZE/2, SIZE/2), (SIZE/2, SIZE/2), (SIZE/2, -SIZE/2), (-SIZE/2, -SIZE/2)]
SHAPE = (3,4)
ARUCO_OBJ = [[((HOR_DISP * (i) + d[0] - HOR_DISP*(SHAPE[0]-1)/2) / 1000., (VERT_DISP*(-j) + d[1] + VERT_DISP*(SHAPE[1]-1)/2)/1000., 0) for d in SQUARE_SHAPE]  for j in range(SHAPE[1]) for i in range(SHAPE[0])]


## ARUCO HELPER FUNCTIONS ##
def get_aruco_corners(color_image):
    """Detect aruco corner and ids"""
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(color_image, ARUCO_DICT, parameters=ARUCO_PARAMS)
    return ids, corners

def get_object(aruco_ids):
    """Return object coords of an aruco id"""
    return [p for a in aruco_ids if a in VALID_ARUCOS for p in ARUCO_OBJ[a]]

MAX_ARUCOS = 100

def get_obj_pxl_points(ids, corners):
    """Return (object points[], input point[]) lists matching the ids, corners supplied"""
    object_pts = np.array(get_object(ids), dtype=np.float32)
    pixel_pts = np.array([c for id, rect in zip(ids, corners) for c in rect[0] if id in VALID_ARUCOS], dtype=np.float32)
    return object_pts[:min(len(object_pts), MAX_ARUCOS)], pixel_pts[:min(len(pixel_pts), MAX_ARUCOS)]


_local_logger = logging.getLogger("LogGenerator")
## Unprojection function ##
def getPixelOnPlane(pixel, rvec, tvec, z_height = BALL_HEIGHT):
    _local_logger.debug("rvecs", rvec,tvec)
    """Given rvec and tvec of a plane and pixel coords of a point, returns it in plane coordinates np.array(x,y,z_height)"""
    uvcoord = np.array([pixel[0], pixel[1], 1])
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    mat1 = rotation_matrix.T @ CAMERA_MATRIX_INV @ uvcoord
    mat2 = rotation_matrix.T @ tvec

    s = (z_height + mat2[2]) / mat1[2]
    wccoord = rotation_matrix.T @ ((s * CAMERA_MATRIX_INV @ uvcoord) - np.ravel(tvec))
    return wccoord
