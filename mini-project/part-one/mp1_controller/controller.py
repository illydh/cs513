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
        ego_velocity = obs.ego_velocity  # current ego speed in m/s
        desired_speed = obs.desired_speed  # cruise set speed, always 20 m/s
        dist_to_lead = obs.distance_to_lead  # raw center-to-center gap; inf when the lead leaves the lane

        # ---- [Yicheng] 1. distance / closing-speed / lead-speed estimate ----
        distance = dist_to_lead  # TODO(Yicheng): turn inf, jumps and the first step into a usable value
        closing_speed = 0.0  # TODO(Yicheng): (prev_distance - distance) / DT, positive when the gap shrinks
        lead_speed = ego_velocity  # TODO(Yicheng): ego_velocity - closing_speed, filtered against noise
        self.prev_distance = distance  # remember this step's distance for the next step
        self.lead_speed_est = lead_speed  # remember this step's lead-speed estimate for the next step

        # ---- [Illy] 2. mode switching (notFollowWhenFarAway) ----
        # TODO(Illy): FOLLOWING if distance < FOLLOW_ENTER, CRUISING if distance > FOLLOW_EXIT, else keep self.mode
        mode = Mode.CRUISING  # placeholder: always cruising
        self.mode = mode  # remember the mode for hysteresis on the next step

        # ---- [Illy] 3. cruise acceleration (reachTargetWhenNotFollowing) ----
        error, gain = desired_speed - ego_velocity, 10  # speed error and proportional gain (gain test @ 10)
        a_cruise = min(MAX_ACCEL, max(MIN_ACCEL, gain * error))  # P control clipped to the legal range

        # ---- [Yicheng] 4. safe-gap limit and braking (safeFollowing, dontStopUnlessLeadStops) ----
        # TODO(Yicheng): desired gap must stay < FOLLOW_ENTER at every speed; brake harder as the gap shrinks
        # TODO(Yicheng): after t = 3 s, never come to a full stop while lead_speed > STOP_SPEED
        a_follow = MAX_ACCEL  # placeholder: imposes no limit until the real logic is written

        # ---- [shared] 5. combine (change only after both agree) ----
        accel = min(a_cruise, a_follow)  # the more cautious of the two commands always wins
        accel = float(np.clip(accel, MIN_ACCEL, MAX_ACCEL))  # keep the command inside the legal range
        self.step_count += 1  # advance the step counter for the next call
        return accel, mode  # simulator expects (float, Mode)
