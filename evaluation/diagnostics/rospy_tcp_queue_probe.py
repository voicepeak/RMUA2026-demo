"""Controlled queue eviction over isolated ROS TCP connections; never touches flight topics."""
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

TOPIC = '/ruma_diagnostic_queue_probe'


def wait_for(predicate, description, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise TimeoutError(description)


def subscriber(name, directory, topic):
    import rospy
    from std_msgs.msg import String
    rospy.init_node(name, disable_signals=True)
    path = Path(directory) / (name + '.jsonl')
    def receive(msg):
        with path.open('a') as out:
            out.write(json.dumps(msg.data) + '\n')
            out.flush()
    listener = rospy.Subscriber(topic, String, receive, queue_size=1000)
    try:
        wait_for(lambda: (Path(directory) / 'stop').exists(), 'subscriber stop', 30)
    finally:
        listener.unregister()


def case(queue_size, directory):
    import rospy
    from std_msgs.msg import String
    topic = TOPIC + '_' + str(queue_size)
    publisher = rospy.Publisher(topic, String, queue_size=queue_size)
    children = []
    release = threading.Event()
    entered = threading.Event()
    try:
        for name in ('probe_slow', 'probe_fast'):
            children.append(subprocess.Popen([sys.executable, __file__, 'subscriber', name, directory, topic]))
        wait_for(lambda: publisher.get_num_connections() == 2, 'two TCP connections')
        connections = {c.endpoint_id: c for c in publisher.impl.connections}
        if set(connections) != {'/probe_slow', '/probe_fast'}:
            raise RuntimeError('unexpected connection endpoints: ' + repr(list(connections)))
        slow = connections['/probe_slow']._connection
        original_write = slow.write_data
        def blocked_write(data):
            if not entered.is_set():
                entered.set()
                if not release.wait(10):
                    raise TimeoutError('injected block timed out')
            return original_write(data)
        slow.write_data = blocked_write
        def rows(name):
            path = Path(directory) / (name + '.jsonl')
            return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
        publisher.publish('A')
        if not entered.wait(10):
            raise TimeoutError('slow TCP write did not enter injected block')
        wait_for(lambda: rows('probe_fast') == ['A'], 'fast A')
        publisher.publish('B')
        wait_for(lambda: rows('probe_fast') == ['A', 'B'], 'fast B')
        publisher.publish('C')
        wait_for(lambda: rows('probe_fast') == ['A', 'B', 'C'], 'fast C')
        release.set()
        expected = ['A', 'C'] if queue_size == 1 else ['A', 'B', 'C']
        wait_for(lambda: rows('probe_slow') == expected, 'slow expected records')
        return {'queue_size': queue_size, 'published': ['A', 'B', 'C'],
                'slow_subscriber': rows('probe_slow'), 'fast_subscriber': rows('probe_fast'),
                'endpoints': sorted(connections), 'connection_type': type(slow).__name__,
                'intervention': 'block first slow underlying TCP write until B and C are enqueued'}
    finally:
        release.set()
        (Path(directory) / 'stop').touch()
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(timeout=5)
        publisher.unregister()
        if any(child.returncode != 0 for child in children):
            raise RuntimeError('subscriber process exited unsuccessfully')


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'subscriber':
        subscriber(*sys.argv[2:])
        return
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    with tempfile.TemporaryDirectory(prefix='ruma_ros_tcp_probe_') as temp:
        os.environ['ROS_HOME'] = temp + '/ros_home'
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            import xmlrpc.client
            def master_ready():
                if master.poll() is not None:
                    raise RuntimeError('private ROS master exited')
                try:
                    return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0] == 1
                except (OSError, xmlrpc.client.Error):
                    return False
            wait_for(master_ready, 'private ROS master')
            import rospy
            import rospy.impl.tcpros_pubsub as tcpros
            rospy.init_node('probe_publisher', disable_signals=True)
            cases = []
            for size in (1, 2):
                directory = Path(temp) / str(size)
                directory.mkdir()
                cases.append(case(size, str(directory)))
            report = {'topic_prefix': TOPIC, 'private_ros_master': True, 'cases': cases,
                      'python_version': platform.python_version(),
                      'probe_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      'rospy_source_sha256': hashlib.sha256(Path(tcpros.__file__).read_bytes()).hexdigest(),
                      'limitations': 'Deliberate publisher-side instrumentation on one diagnostic connection; no simulator or backend delivery tested.'}
            print(json.dumps(report, indent=2))
        finally:
            os.killpg(master.pid, signal.SIGTERM)
            try:
                master.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGKILL)
                master.wait()


if __name__ == '__main__':
    main()
