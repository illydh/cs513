"""This file is the main controller file.

Here, you will design the controller for your for the adaptive cruise control system.
"""

from mp1_simulator.simulator import Observation, Mode

from typing import Tuple


# NOTE: Very important that the class name remains the same
class Controller:
    def __init__(self, distance_threshold: float):
        self.distance_threshold = distance_threshold
        # self.prev_dist_to_lead = None
        self.mode = Mode.FOLLOWING

    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        """This is the main run step of the controller.

        Here, you will have to read in the observations `obs`, process it, and output an
        acceleration value and the operation mode.
        
        The acceleration value must be some value between -10.0 and 10.0.

        For the operation mode output True when "following" and False when "cruising".

        Note that the acceleration value is really some control input that is used
        internally to compute the throttle an brake values of the car.

        Below is some example code where the car just outputs the control value 10.0
        """

        ego_velocity = obs.ego_velocity
        desired_speed = obs.desired_speed
        dist_to_lead = obs.distance_to_lead

        # Do your magic...
        cruise_gain, follow_gain = 10, 0.5
        desired_gap = 20
        self.mode = Mode.CRUISING if dist_to_lead > 40 else Mode.FOLLOWING
        
        if not self.prev_dist_to_lead:
            self.prev_dist_to_lead = dist_to_lead
            
        # if dist_to_lead != float('inf') and self.prev_dist_to_lead != float('inf'):
        #         delta = dist_to_lead = self.prev_dist_to_lead
        #         closing_speed = -delta / 0.1
        #         if closing_speed > 0:
        #             time_left = dist_to_lead / closing_speed
        #             if time_left < 
        
        # error = 
        cruise_accel = (desired_speed - ego_velocity) * cruise_gain
        follow_accel = (dist_to_lead - desired_gap) * follow_gain
        accel = min(10, max(-10, min(cruise_accel, follow_accel)))
        
        return (accel, self.mode)
