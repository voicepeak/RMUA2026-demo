"""Fail-latched command gate. Does not invent a flight fallback on sensor loss."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PwmCommand:
    stamp: float
    epoch: int
    sequence: int
    values: tuple

    def __post_init__(self):
        if (not math.isfinite(self.stamp) or self.stamp <= 0 or self.epoch < 0
                or self.sequence < 0 or not isinstance(self.values, tuple)
                or len(self.values) != 4
                or not all(math.isfinite(v) and 0 <= v <= 1 for v in self.values)):
            raise ValueError('invalid normalized PWM command')


class CommandGuard:
    def __init__(self, epoch, minimum_period=.01, input_timeout=.1):
        if not (math.isfinite(minimum_period) and minimum_period >= .01
                and math.isfinite(input_timeout) and input_timeout > 0):
            raise ValueError('invalid command guard timing')
        self.epoch = epoch
        self.minimum_period = minimum_period
        self.input_timeout = input_timeout
        self.last_wall = None
        self.last_sequence = -1
        self.last_stamp = None
        self.fault = None

    def fail(self, reason):
        self.fault = self.fault or reason
        raise RuntimeError(self.fault)

    def accept(self, command, clock, wall_now, source_arrival):
        if self.fault:
            self.fail(self.fault)
        if not math.isfinite(wall_now):
            self.fail('INVALID_WALL_CLOCK')
        if command.epoch != self.epoch or clock.epoch != self.epoch:
            self.fail('CLOCK_RESET')
        if (not isinstance(source_arrival, (int, float)) or not math.isfinite(source_arrival)
                or not 0 <= wall_now-source_arrival <= self.input_timeout
                or not clock.fresh(wall_now, self.input_timeout) or clock.stamp is None
                or source_arrival > clock.arrival
                or not 0 <= clock.stamp-command.stamp <= .05):
            self.fail('STALE_IMU_OR_COMMAND')
        if command.sequence <= self.last_sequence:
            self.fail('COMMAND_REPLAY')
        if self.last_stamp is not None and command.stamp < self.last_stamp:
            self.fail('COMMAND_TIME_ROLLBACK')
        if self.last_wall is not None:
            elapsed = wall_now-self.last_wall
            if elapsed < 0:
                self.fail('WALL_CLOCK_ROLLBACK')
            if elapsed < self.minimum_period:
                return False  # Rate limit; no stale command replay to fill gaps.
        self.last_wall, self.last_sequence, self.last_stamp = wall_now, command.sequence, command.stamp
        return True

    def published(self, wall_now):
        """Space the next command from completion even if ROS publish blocked."""
        if (self.last_wall is None or not math.isfinite(wall_now)
                or wall_now < self.last_wall):
            self.fail('INVALID_PUBLICATION_TIME')
        self.last_wall = wall_now
