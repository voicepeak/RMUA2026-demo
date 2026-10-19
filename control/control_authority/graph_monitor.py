"""Move blocking ROS graph RPCs off the control loop; freshness is fail-closed."""
from collections import deque
import math
import threading
import time


class GraphMonitor:
    def __init__(self, probe, interval=.1, stale_after=.3, clock=time.monotonic):
        if not (math.isfinite(interval) and math.isfinite(stale_after) and 0 < interval < stale_after):
            raise ValueError('invalid graph observation timing')
        self.probe, self.interval, self.stale_after, self.clock = probe, interval, stale_after, clock
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.last_verified = None
        self.fault = None
        self.durations = deque(maxlen=1000)
        self.count = 0
        self.thread = None

    def _once(self):
        start = self.clock()
        try:
            self.probe()
            end = self.clock()
            with self.lock:
                # The returned graph could have been sampled at request time.
                self.last_verified = start
                self.durations.append(end-start)
                self.count += 1
        except Exception as error:
            with self.lock:
                if not self.stop_event.is_set():
                    self.fault = str(error)

    def start(self):
        self._once()  # Startup verification precedes any flight command.
        self.check()
        def worker():
            while not self.stop_event.wait(self.interval):
                self._once()
        self.thread = threading.Thread(target=worker, daemon=True)
        self.thread.start()

    def check(self):
        now = self.clock()
        with self.lock:
            if self.fault:
                raise RuntimeError('ROS_GRAPH:'+self.fault)
            if self.last_verified is None or not 0 <= now-self.last_verified <= self.stale_after:
                raise RuntimeError('ROS_GRAPH_STALE')

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=.2)

    def summary(self):
        with self.lock:
            return {'successful_checks':self.count,'fault':self.fault,
                    'max_rpc_wall_s':max(self.durations,default=0.),
                    'stale_after_wall_s':self.stale_after}
