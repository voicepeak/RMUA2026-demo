#!/usr/bin/env python3
"""Monotonic command lease; expires without waiting for the planner lock.

The ROS adapter serializes publication and calls expire/record under its
publication lock. An expired calculation cannot restart motion when it returns.
"""
import math


class CommandDeadline:
    def __init__(self,hold=.25):
        if not math.isfinite(hold) or not 0.<hold<.35:
            raise ValueError('Command deadline must be inside the 0.35s prediction hold')
        self.hold=hold
        self.last_publication=None
        self.expired_at=None
        self.armed=False

    def expire(self,now):
        if self.armed and now-self.last_publication>=self.hold:
            self.armed=False;self.expired_at=now
            return True
        return False

    def accepts(self,started):
        return self.expired_at is None or (started is not None and started>self.expired_at)

    def record(self,now,stopped):
        self.last_publication=now;self.armed=not stopped
