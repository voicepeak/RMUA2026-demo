"""IMU owns simulator time; wall monotonic only measures reception freshness."""
import math


class SensorClock:
    def __init__(self):
        self.epoch = 0
        self.stamp = None
        self.arrival = None

    def observe(self, stamp, arrival):
        if not all(math.isfinite(t) and t >= 0 for t in (stamp, arrival)) or stamp == 0:
            raise ValueError("invalid IMU clock")
        reset = self.stamp is not None and stamp < self.stamp
        if reset:
            self.epoch += 1
        # Duplicates must not refresh a stalled sensor's wall deadline.
        if self.stamp != stamp:
            self.stamp, self.arrival = stamp, arrival
        return reset

    def fresh(self, arrival, timeout=.1):
        return (self.arrival is not None and math.isfinite(arrival)
                and 0 <= arrival - self.arrival <= timeout)
