#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Summary of useful helper functions for scenarios
"""

from __future__ import print_function
import math

import numpy as np
import carla
from agents.tools.misc import vector

def get_crossing_point(actor):
    """
    Get the next crossing point location in front of the ego vehicle

    @return point of crossing
    """
    wp_cross = actor.get_world().get_map().get_waypoint(actor.get_location())

    while(not wp_cross.is_intersection):
        wp_cross = wp_cross.next(2)[0]

    crossing = carla.Location(x=wp_cross.transform.location.x,
                              y=wp_cross.transform.location.y, z=wp_cross.transform.location.z)

    return crossing


def get_geometric_linear_intersection(ego_actor, other_actor):
    """
    Obtain a intersection point between two actor's location by using their waypoints (wp)

    @return point of intersection of the two vehicles
    """

    wp_ego_1 = ego_actor.get_world().get_map().get_waypoint(ego_actor.get_location())
    wp_ego_2 = wp_ego_1.next(1)[0]
    x_ego_1 = wp_ego_1.transform.location.x
    y_ego_1 = wp_ego_1.transform.location.y
    x_ego_2 = wp_ego_2.transform.location.x
    y_ego_2 = wp_ego_2.transform.location.y

    wp_other_1 = other_actor.get_world().get_map().get_waypoint(other_actor.get_location())
    wp_other_2 = wp_other_1.next(1)[0]
    x_other_1 = wp_other_1.transform.location.x
    y_other_1 = wp_other_1.transform.location.y
    x_other_2 = wp_other_2.transform.location.x
    y_other_2 = wp_other_2.transform.location.y

    s = np.vstack([(x_ego_1, y_ego_1), (x_ego_2, y_ego_2), (x_other_1, y_other_1), (x_other_2, y_other_2)])
    h = np.hstack((s, np.ones((4, 1))))
    line1 = np.cross(h[0], h[1])
    line2 = np.cross(h[2], h[3])
    x, y, z = np.cross(line1, line2)
    if z == 0:
        return (float('inf'), float('inf'))

    intersection = carla.Location(x=x / z, y=y / z, z=0)

    return intersection

def get_location_in_distance(actor, distance):
    """
    Obtain a location in a given distance from the current actor's location.
    Note: Search is stopped on first intersection.

    @return obtained location and the traveled distance
    """
    waypoint = actor.get_world().get_map().get_waypoint(actor.get_location())
    traveled_distance = 0
    while not waypoint.is_intersection and traveled_distance < distance:
        waypoint_new = waypoint.next(1.0)[-1]
        traveled_distance += waypoint_new.transform.location.distance(waypoint.transform.location)
        waypoint = waypoint_new

    return waypoint.transform.location, traveled_distance

def generate_target_waypoint(waypoint, turn=0):
    """
    This method follow waypoints to a junction and choose path based on turn input.
    Turn input: LEFT -> 1, RIGHT -> -1, STRAIGHT -> 0
    @returns a waypoint list according to turn input
    """
    sampling_radius = 1
    reached_junction = False
    wp_list = []
    threshold = math.radians(0.1)
    while True:
        current_transform = waypoint.transform
        current_location = current_transform.location
        projected_location = current_location + \
        carla.Location(
            x=math.cos(math.radians(current_transform.rotation.yaw)),
            y=math.sin(math.radians(current_transform.rotation.yaw)))
        wp_choice = waypoint.next(sampling_radius)
        #   Choose path at intersection
        if len(wp_choice) > 1:
            reached_junction = True
            waypoint = choose_at_junction(current_location, projected_location, wp_choice, turn)
        else:
            waypoint = wp_choice[0]
        wp_list.append(waypoint)
        #   End condition for the behaviour
        if turn != 0 and reached_junction and len(wp_list) >= 3:
            v_1 = vector(
                wp_list[-2].transform.location,
                wp_list[-1].transform.location)
            v_2 = vector(
                wp_list[-3].transform.location,
                wp_list[-2].transform.location)
            angle_wp = math.acos(
                np.dot(v_1, v_2)/abs((np.linalg.norm(v_1)*np.linalg.norm(v_2))))
            if angle_wp < threshold:
                break
        elif reached_junction and not wp_list[-1].is_intersection:
            break
    return wp_list[-1]

def choose_at_junction(previous, current, next_choices, direction=0):
    """
    This function chooses the appropriate waypoint from next_choices based on direction
    """
    current_vector = vector(previous, current)
    cross_list = []
    cross_to_waypoint = dict()
    for waypoint in next_choices:
        waypoint = waypoint.next(20)[0]
        select_vector = vector(current, waypoint.transform.location)
        cross = np.cross(current_vector, select_vector)[2]
        cross_list.append(cross)
        cross_to_waypoint[cross] = waypoint
    select_cross = None
    if direction > 0:
        select_cross = max(cross_list)
    elif direction < 0:
        select_cross = min(cross_list)
    else:
        select_cross = min(cross_list, key=abs)

    return cross_to_waypoint[select_cross]
