"""This file is the main controller file.

Here, you will design the controller for your for the adaptive cruise control system.
"""

from typing import Tuple  # type hint for the (acceleration, mode) return value

import numpy as np  # used for clipping the final acceleration

from mp1_simulator.simulator import Observation, Mode  # simulator data types (do not redefine)


# ---------------------------------------------------------------------------
# Shared constants (both teammates read these; change only after agreeing)
# ---------------------------------------------------------------------------
DT = 0.1  # simulator step length in seconds (CONFIG["dt"])
MAX_ACCEL = 10.0  # largest control value the simulator accepts (full throttle)
MIN_ACCEL = -10.0  # smallest control value the simulator accepts (half brake = -10/20)
FAR_DISTANCE = 40.0  # evaluator: d > 40 m must lead to CRUISING within 2 s
FOLLOW_ENTER = 35.0  # switch to FOLLOWING when d drops below this (must be < FAR_DISTANCE)
FOLLOW_EXIT = 38.0  # switch back to CRUISING when d rises above this (must be < FAR_DISTANCE)
MIN_SAFE_DISTANCE = 4.0  # evaluator: center-to-center distance must stay >= 4 m
STOP_SPEED = 0.1  # evaluator: speeds below this count as "stopped"


# NOTE: Very important that the class name remains the same
class Controller:  # class name is required by the autograder
    def __init__(self, distance_threshold: float):  # signature is required by the autograder
        self.distance_threshold = distance_threshold  # value passed in by the simulator (4 m)
        self.step_count = 0  # number of run_step calls so far in this episode
        self.prev_distance = None  # distance seen on the previous step (None before first step)
        self.lead_speed_est = None  # latest estimate of the lead car's speed (None until known)
        self.mode = Mode.CRUISING  # current operating mode, kept for hysteresis

    def run_step(self, obs: Observation) -> Tuple[float, Mode]:  # called once every 0.1 s
        """Return (acceleration in [-10, 10], operating mode) for one simulation step."""
        distance, closing_speed, lead_speed = self._estimate(obs)  # [Yicheng] clean up sensor data
        self.mode = self._select_mode(distance)  # [Illy] decide CRUISING / FOLLOWING
        a_cruise = self._cruise_accel(obs)  # [Illy] accel that tracks the desired speed
        a_follow = self._follow_accel(obs, distance, closing_speed, lead_speed)  # [Yicheng] accel limit that keeps us safe
        accel = min(a_cruise, a_follow)  # the more cautious of the two commands always wins
        accel = float(np.clip(accel, MIN_ACCEL, MAX_ACCEL))  # keep the command inside the legal range
        self.step_count += 1  # advance the step counter for the next call
        return accel, self.mode  # simulator expects (float, Mode)

    # -----------------------------------------------------------------------
    # Yicheng: distance / closing-speed estimate, inf and first-step cases
    # -----------------------------------------------------------------------
    def _estimate(self, obs: Observation) -> Tuple[float, float, float]:  # returns (distance, closing_speed, lead_speed)
        distance = obs.distance_to_lead  # raw distance; may be inf when the lead leaves the lane
        closing_speed = 0.0  # TODO(Yicheng): (prev_distance - distance) / DT, positive when the gap shrinks
        lead_speed = obs.ego_velocity  # TODO(Yicheng): ego_velocity - closing_speed, filtered against noise
        # TODO(Yicheng): handle step_count == 0 (no prev_distance) and inf / jumps in distance
        self.prev_distance = distance  # remember this distance for the next step
        self.lead_speed_est = lead_speed  # remember this estimate for the next step
        return distance, closing_speed, lead_speed  # hand the cleaned values to run_step

    # -----------------------------------------------------------------------
    # Yicheng: safe-gap logic and braking (safeFollowing, dontStopUnlessLeadStops)
    # -----------------------------------------------------------------------
    def _follow_accel(self, obs: Observation, distance: float, closing_speed: float, lead_speed: float) -> float:  # upper bound on accel
        # TODO(Yicheng): compute desired gap (must stay < FOLLOW_ENTER at every speed) and brake as needed
        # TODO(Yicheng): never fully stop while lead_speed > STOP_SPEED (after t = 3 s)
        return MAX_ACCEL  # placeholder: imposes no limit until the real logic is written

    # -----------------------------------------------------------------------
    # Illy: mode switching (notFollowWhenFarAway)
    # -----------------------------------------------------------------------
    def _select_mode(self, distance: float) -> Mode:  # returns the mode for this step
        # TODO(Illy): FOLLOWING when distance < FOLLOW_ENTER, CRUISING when distance > FOLLOW_EXIT,
        #          otherwise keep self.mode (hysteresis); inf counts as far away
        return Mode.CRUISING  # placeholder: always cruising

    # -----------------------------------------------------------------------
    # Illy: cruise acceleration (reachTargetWhenNotFollowing)
    # -----------------------------------------------------------------------
    def _cruise_accel(self, obs: Observation) -> float:  # accel that drives speed toward desired_speed
        # TODO(Illy): speed controller (e.g. PI) reaching |v - desired_speed| < 2 within 6 s from rest
        error, gain = obs.desired_speed - obs.ego_velocity, 10  # speed error and proportional gain (gain test @ 10)
        return min(MAX_ACCEL, max(MIN_ACCEL, gain * error))  # P control clipped to the legal range
