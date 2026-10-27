"""Brake test controller (do not merge).

Accelerates at full throttle to BRAKE_START_SPEED, then holds a constant brake
command until the episode ends. Each new episode uses the next value in
BRAKE_LEVELS, so running scenario1_0, scenario1_1, scenario1_2 in that order
measures the deceleration for -10, -5 and -2.5.
"""

from mp1_simulator.simulator import Observation, Mode

from typing import Tuple


BRAKE_START_SPEED = 19.5
BRAKE_LEVELS = [-10.0, -5.0, -2.5]


class Controller:
    _episode = 0

    def __init__(self, distance_threshold: float):
        self.distance_threshold = distance_threshold
        self.brake = BRAKE_LEVELS[Controller._episode % len(BRAKE_LEVELS)]
        Controller._episode += 1
        self.braking = False

    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        if obs.ego_velocity >= BRAKE_START_SPEED:
            self.braking = True
        return (self.brake if self.braking else 10.0), Mode.CRUISING
