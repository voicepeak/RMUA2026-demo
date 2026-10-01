#!/usr/bin/env python3
"""One planner job at a time; the control loop never waits for its future."""
from concurrent.futures import ThreadPoolExecutor
import time


class AsyncPlanner:
    def __init__(self):
        self.executor=ThreadPoolExecutor(max_workers=1)
        self.future=None;self.generation=0
    def submit(self,stamp,job):
        if self.future is not None:return False
        generation=self.generation
        submitted=time.monotonic()
        def run():
            start=time.monotonic();info,plan=job()
            completed=time.monotonic()
            return dict(info,planned_stamp=stamp,planning_ms=1000.*(completed-start),
                        submitted_monotonic=submitted,started_monotonic=start,
                        completed_monotonic=completed,queue_ms=1000.*(start-submitted)),plan,generation
        self.future=self.executor.submit(run);return True
    def poll(self):
        if self.future is None or not self.future.done():return None
        future=self.future;self.future=None
        info,plan,generation=future.result()
        info=dict(info,received_monotonic=time.monotonic())
        return (info,plan) if generation==self.generation else None
    def reset(self):self.generation+=1
    def close(self):self.executor.shutdown(wait=False)
