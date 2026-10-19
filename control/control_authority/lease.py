"""Cooperative process lock; ROS graph checks still detect non-cooperating nodes."""
import fcntl
import json
from pathlib import Path


class ControlLease:
    def __init__(self, scene_id, mode, directory=Path('/tmp/rmua_next_control')):
        if mode not in ('PWM', 'VEL') or not scene_id:
            raise ValueError('scene id and mode required')
        self.directory = Path(directory)
        self.scene_id, self.mode = scene_id, mode
        self.handle = None

    def __enter__(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        handle = (self.directory/'publisher.lock').open('a+')
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            path = self.directory/'mode.json'
            if path.exists():
                previous = json.loads(path.read_text())
                if previous['scene_id'] == self.scene_id and previous['mode'] != self.mode:
                    raise RuntimeError('control mode already locked for this ROS scene')
            temporary = self.directory/'mode.tmp'
            temporary.write_text(json.dumps({'scene_id': self.scene_id, 'mode': self.mode}))
            temporary.replace(path)
        except BaseException:
            handle.close()
            raise
        self.handle = handle
        return self

    def __exit__(self, *_):
        self.handle.close()
        self.handle = None


def check_graph(publishers, node_name, mode='PWM', require_self=True):
    topics = {'PWM': '/airsim_node/drone_1/rotor_pwm_cmd',
              'VEL': '/airsim_node/drone_1/vel_body_cmd'}
    for name, topic in topics.items():
        nodes = set(publishers.get(topic, ()))
        expected = {node_name} if name == mode and require_self else set()
        if nodes != expected:
            raise RuntimeError('control publisher conflict: '+topic+' '+str(sorted(nodes)))
