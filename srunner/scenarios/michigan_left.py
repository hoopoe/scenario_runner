#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This scenario force the ego-vehicle to yield to traffic to perform a Michigan Left action.
"""

from __future__ import print_function
import sys

import py_trees
import carla

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenarios.basic_scenario import *


MICHIGAN_LEFT_SCENARIOS = [
    "MichiganLeft"
]


class MichiganLeft(BasicScenario):

    """
    TODO
    """

    category = "MichiganLeft"

    timeout = 1000                       # Timeout of scenario in seconds

    # other vehicle
    _other_actor_source_flow1 = carla.Transform(location=carla.Location(x=12.6, y=279.6, z=0.5),
                                                rotation=carla.Rotation(yaw=-90))
    _other_actor_target_flow1 = carla.Location(x=7.3, y=-112.6, z=0)


    def __init__(self, world, ego_vehicle, other_actors, town, randomize=False, debug_mode=False):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._other_actors = []

        super(MichiganLeft, self).__init__("MichiganLeft", ego_vehicle, self._other_actors, town, world, debug_mode)

    def _create_behavior(self):
        """
        Scenario behavior:
        The other vehicle waits until the ego vehicle is close enough to the
        intersection and that its own traffic light is red. Then, it will start
        driving and 'illegally' cross the intersection. After a short distance
        it should stop again, outside of the intersection. The ego vehicle has
        to avoid the crash, but continue driving after the intersection is clear.

        If this does not happen within 120 seconds, a timeout stops the scenario
        """

        # start condition
        source_flow1 = CreateContinuousTrafficFlow(self._other_actors,
                                                   self._other_actor_source_flow1,
                                                   self._other_actor_target_flow1,
                                                   self.world)


        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(source_flow1)

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicle)
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        for actor in self._other_actors:
            actor.remove()
