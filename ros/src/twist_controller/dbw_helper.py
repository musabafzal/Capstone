#!/usr/bin/env python


import numpy as np
import tf

from math import sqrt, cos, sin


POINTS_TO_FIT = 15


def fit_polynomial(waypoints, degree):    
    x = [waypoint.pose.pose.position.x for waypoint in waypoints]
    y = [waypoint.pose.pose.position.y for waypoint in waypoints]
    return np.polyfit(x, y, degree)


def get_euler(pose):    
    return tf.transformations.euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])


def eucleidian_distance(x0, y0, x1, y1):    
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2))


def distance2parabola(coefficients, x, y):
    a, b, c = coefficients
    p0 = 2 * a * a
    p1 = 3 * a * b
    p2 = 2 * a * c - 2 * a * y + b * b + 1
    p3 = b * c - b * y - x

    p = [p0, p1, p2, p3]

    roots = np.roots(p)
    # filter the real only root
    x_ = np.real(roots[np.isreal(roots)])[0]
    y_ = np.polyval(coefficients, x_)

    distance = eucleidian_distance(x, y, x_, y_)
    left = -1 if y_ < y else 1

    return distance, left, x_, y_


def shift_and_rotate_waypoints(pose, waypoints, points_to_use=None):
    x = []
    y = []

    _, _, yaw = get_euler(pose)
    originX = pose.position.x
    originY = pose.position.y

    if points_to_use is None:
        points_to_use = len(waypoints)

    for i in range(points_to_use):

        shift_x = waypoints[i].pose.pose.position.x - originX
        shift_y = waypoints[i].pose.pose.position.y - originY

        x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)
        y = shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw)

        x.append(x)
        y.append(y)

    return x, y


def is_waypoint_behind(pose, waypoint):
    _, _, yaw = get_euler(pose)
    originX = pose.position.x
    originY = pose.position.y

    shift_x = waypoint.pose.pose.position.x - originX
    shift_y = waypoint.pose.pose.position.y - originY

    x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)

    if x > 0:
        return False
    return True


def cte(pose, waypoints):
    x, y = shift_and_rotate_waypoints(
        pose, waypoints, POINTS_TO_FIT)
    coefficients = np.polyfit(x, y, 3)
    distance = np.polyval(coefficients, 1.0)

    return distance
