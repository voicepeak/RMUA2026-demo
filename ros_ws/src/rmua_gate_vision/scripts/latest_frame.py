#!/usr/bin/env python3
"""Bounded latest-only mailbox: inference never blocks sensor reception."""
import threading

class LatestFrame:
    def __init__(self):
        self.condition=threading.Condition()
        self.value=None
        self.dropped=0

    def put(self,value):
        with self.condition:
            if self.value is not None:self.dropped+=1
            self.value=value
            self.condition.notify()

    def take(self,timeout=.2):
        with self.condition:
            self.condition.wait_for(lambda:self.value is not None,timeout=timeout)
            value,self.value=self.value,None
            return value

    def clear(self):
        with self.condition:
            self.value=None
