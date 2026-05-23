import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # Determine the correct rotation (bearing) and distance from X(4) to L(2) 
    
    pose = result.atPose2(X(4))
    landmark = result.atPoint2(L(2))

    x, y, theta = pose.x(), pose.y(), pose.theta()
    lx, ly = landmark[0], landmark[1]

    distance = np.sqrt((lx - x)**2 + (ly - y)**2)

    degree = np.arctan2(ly - y,lx - x)
    rotation = degree - np.pi/2
    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), gtsam.Rot2(rotation), distance, MEASUREMENT_NOISE))
    return graph