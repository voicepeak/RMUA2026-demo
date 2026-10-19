"""Persistent, headless P0 deployment observer. Never claims mission capability."""
import json
from pathlib import Path
import signal
import time


def main():
    import rospy
    from adapters.ros_noetic.flight_sensors import FlightSensors
    config = json.loads((Path(__file__).resolve().parents[2]/'configs/competition.yaml').read_text())
    if config['runtime_control_enabled']:
        raise RuntimeError('competition flight entrypoint has not passed acceptance')
    print('RMUA development observer: competition acceptance pending; no control published.', flush=True)
    # init_node waits for the supplied ROS master. No GUI or runtime dependency install.
    rospy.init_node('rmua_next_runtime_observer', disable_signals=True)
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    sensors = FlightSensors()
    try:
        while not stopped:
            try:
                state, _ = sensors.read()
                print(json.dumps({'stamp': state.metadata.stamp, 'health': state.health.value,
                                  'position_ned': state.position_ned, 'competition_enabled': False}), flush=True)
            except (ValueError, RuntimeError) as error:
                print(json.dumps({'health': 'LOST', 'reason': str(error)}), flush=True)
            time.sleep(1.)
    finally:
        sensors.close()
        rospy.signal_shutdown('launcher stopped observer')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
