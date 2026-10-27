"""This file is the main controller file.

Here, you will design the controller for your for the adaptive cruise control system.
"""

from typing import Tuple

import numpy as np

from mp1_simulator.simulator import Observation, Mode


# NOTE: Very important that the class name remains the same
class Controller:  # class name is required by the autograder
    def __init__(self, distance_threshold: float):  # signature is required by the autograder
        self.distance_threshold = distance_threshold

        # for dist/ closing-speed / lead-speed estimate
        self.prev_dist_to_lead = None
        self.prev_ego_velocity = None
        self.prev_lead_speed = 0.0

    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        """Return (acceleration in [-10, 10], operating mode) for one simulation step."""
        
        ego_velocity = obs.ego_velocity
        desired_speed = obs.desired_speed
        dist_to_lead = obs.distance_to_lead

        # ---- [Yicheng] 1. distance / closing-speed / lead-speed estimate ----
        lead_in_lane = np.isfinite(dist_to_lead)
        lead_in_lane_last_step = True if self.prev_dist_to_lead is not None and np.isfinite(self.prev_dist_to_lead) else False

        if lead_in_lane and lead_in_lane_last_step:  # both ends finite: differencing is meaningful
            avg_ego = 0.5*(ego_velocity+self.prev_ego_velocity)
            lead_speed = max(0.0, avg_ego-(self.prev_dist_to_lead-dist_to_lead)/0.1)
        else:  # first step, lead just left, or lead just came back
            lead_speed = self.prev_lead_speed
        
        if lead_in_lane:
            closing_speed = ego_velocity - lead_speed
        else:
            closing_speed = 0.0
        
        # for next step
        self.prev_dist_to_lead = dist_to_lead
        self.prev_ego_velocity = ego_velocity
        self.prev_lead_speed = lead_speed



        # Do your magic...
        error, gain = desired_speed - ego_velocity, 10
        accel = min(10, max(-10, gain * error))

        return (accel, Mode.CRUISING)
