"""Necessary preconditions only. Never treated as a flight safety certificate."""
from dataclasses import dataclass
from typing import Tuple
from interfaces.contracts import Health, StateEstimate, Trajectory


@dataclass(frozen=True)
class PreflightResult:
    reasons: Tuple[str, ...]

    @property
    def prerequisites_met(self):
        return not self.reasons


def check_inputs(state: StateEstimate, trajectory: Trajectory, now: float,
                 epoch: int, pwm_publishers: int, vel_publishers: int) -> PreflightResult:
    reasons = []
    if not state.metadata.current(now, epoch):
        reasons.append("STATE_TIME_OR_EPOCH")
    if state.health != Health.OK:
        reasons.append("STATE_HEALTH")
    if not trajectory.metadata.current(now, epoch):
        reasons.append("TRAJECTORY_TIME_OR_EPOCH")
    if state.metadata.frame_id != trajectory.metadata.frame_id:
        reasons.append("FRAME_MISMATCH")
    if pwm_publishers != 1 or vel_publishers != 0:
        reasons.append("CONTROL_PUBLISHER_CONFLICT")
    return PreflightResult(tuple(reasons))
