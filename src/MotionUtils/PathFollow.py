import numpy as np
from math import fmod, modf
from .motionConstants.constants import *
# This python code is meant to follow a path.
# Input: a list of (x,y) coordinates.
# Stream: updates of robot position or returns NaN if not updated.

def clamp(val, v_min, v_max):
    return min(v_max, max(v_min,val))

def getEdgeProjection(config, edge):
    p1 = np.asarray(edge[0])
    p2 = np.asarray(edge[1])
    point = np.asarray(config)

    # Get a vector of the given path edge
    edge_vector = p2 - p1
    edge_length_squared = np.dot(edge_vector,edge_vector)
    if edge_length_squared <= 0.001:    
        return p2, 1

    # Vector from path start to current point
    point_vector = point - p1

    # T is the fraction along the path the projection is on.
    t_distance = edge_vector.dot(point_vector)
    t = t_distance / edge_length_squared

    projection = None
    if(t < 0):
        projection = p1
    elif (t>1):
        projection = p2
    else:
        projection = t*edge_vector + p1
    return projection, clamp(t, 0, 1)

def getClampedTarget(point, target, lookahead_distance):
    if lookahead_distance < 0.00001:
        return point
    target_vector = np.asarray(target) - point
    target_distance = np.linalg.norm(target_vector)
    if target_distance <= lookahead_distance:
        return target
    return point + lookahead_distance * target_vector / target_distance

def getPointFromT(path ,edge_num, t):
        if edge_num >= len(path) - 1:
            return np.array(path[-1])
        p1 = np.asarray(path[edge_num])
        p2 = np.asarray(path[edge_num + 1])

        return (1-t) * p1 + t * p2


class PathFollowStrict:
    path = None     #List of the path configurations
    path_edges = None       #List of edges I.E. pairs of points
    PATH_LOOKAHEAD = 0.3
    EDGE_CUTOFF = 0.3
    current_edge = 0
    def __init__(self, path, path_lookahead = TASK_PATH_LOOKAHEAD, EDGE_CUTOFF = TASK_EDGE_CUTOFF):
        self.path = path
        self.path_edges = [edge for edge in zip(self.path, self.path[1:])]
        self.PATH_LOOKAHEAD = path_lookahead
        self.EDGE_CUTOFF = EDGE_CUTOFF

    # doesn't change self, returns point a certain distance forward from the projection
    def getLookaheadData(self, config, lookahead_distance = None):
        if lookahead_distance == None:
            lookahead_distance = self.PATH_LOOKAHEAD

        if self.current_edge >= len(self.path)-1:
            return np.asarray(self.path[-1]), len(self.path)-1, 1  # P, e, t

        point = np.asarray(config)
        edge = self.path_edges[self.current_edge]

        proj, t = getEdgeProjection(config, edge)
        edge_length = np.linalg.norm(edge[1] - np.array(edge[0]))

        target = None
        target_t = 0
        if edge_length <= 0.001:
            target = edge[1]
            target_t = 1
        else:
            target_t = clamp(t + lookahead_distance / edge_length, 0, 1)
            target = getPointFromT(self.path, self.current_edge, target_t)

        return target, self.current_edge, target_t

    def getClampedLookaheadConfig(self, config, lookahead_distance = None, clamp_distance = None):
        if lookahead_distance == None:
            lookahead_distance = self.PATH_LOOKAHEAD
        if clamp_distance is None:
            clamp_distance = lookahead_distance

        target_conf, _, _ = self.getLookaheadData(config,lookahead_distance)
        return getClampedTarget(config, target_conf, clamp_distance)

    #Changes self
    def updateCurrentEdge(self, config, cutoff_radius = None):
        if cutoff_radius == None:
            cutoff_radius = self.EDGE_CUTOFF

        if self.current_edge >= len(self.path)-1:
            return
        if np.linalg.norm(np.asarray(config) - self.path[self.current_edge + 1]) < cutoff_radius:
            self.current_edge += 1


