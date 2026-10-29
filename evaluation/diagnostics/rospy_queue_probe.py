"""Isolated deterministic probe of Noetic rospy per-connection queue eviction.

No ROS master, network, vehicle, or flight-control topic is used.
"""
import hashlib
import json
import platform
import threading
import time
from pathlib import Path

import rospy.impl.tcpros_pubsub as tcpros


class ControlledTransport:
    def __init__(self, block_first=False):
        self.done = False
        self.block_first = block_first
        self.started = threading.Event()
        self.release = threading.Event()
        self.changed = threading.Condition()
        self.writes = []
        self.cleanup = None

    def set_cleanup_callback(self, callback):
        self.cleanup = callback

    def write_data(self, data):
        if self.block_first and not self.started.is_set():
            self.started.set()
            if not self.release.wait(5):
                raise TimeoutError('blocked transport not released')
        with self.changed:
            self.writes.append(data.decode('ascii'))
            self.changed.notify_all()
        return True

    def wait_count(self, count):
        with self.changed:
            if not self.changed.wait_for(lambda: len(self.writes) >= count, timeout=5):
                raise TimeoutError('transport did not receive expected records')

    def close(self):
        self.done = True
        self.release.set()
        self.cleanup(self)


def run(queue_size):
    slow = ControlledTransport(block_first=True)
    fast = ControlledTransport()
    connections = [tcpros.QueuedConnection(slow, queue_size), tcpros.QueuedConnection(fast, queue_size)]
    try:
        for conn in connections:
            conn.write_data(b'A')
        if not slow.started.wait(5):
            raise TimeoutError('first slow transport write did not start')
        fast.wait_count(1)
        for value, count in ((b'B', 2), (b'C', 3)):
            for conn in connections:
                conn.write_data(value)
            fast.wait_count(count)
        slow.release.set()
        expected = ['A', 'C'] if queue_size == 1 else ['A', 'B', 'C']
        slow.wait_count(len(expected))
        if slow.writes != expected or fast.writes != ['A', 'B', 'C']:
            raise AssertionError('unexpected queue behavior')
        return {'queue_size': queue_size, 'send_order': ['A', 'B', 'C'],
                'blocked_connection_writes': slow.writes[:], 'other_connection_writes': fast.writes[:],
                'slow_first_write_blocked_until_after_C_enqueued': True,
                'python_version': platform.python_version(),
                'rospy_source': tcpros.__file__,
                'rospy_source_sha256': hashlib.sha256(Path(tcpros.__file__).read_bytes()).hexdigest(),
                'probe_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'limitations': 'Controlled fake transports; no TCP, ROS subscriber, simulator, backend or motor behavior tested.'}
    finally:
        for transport in (slow, fast):
            transport.close()
        for conn in connections:
            conn._thread.join(timeout=5)
            if conn._thread.is_alive():
                raise TimeoutError('queue worker failed to stop')


if __name__ == '__main__':
    print(json.dumps({'cases': [run(1), run(2)]}, indent=2))
