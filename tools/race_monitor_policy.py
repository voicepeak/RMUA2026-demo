"""Distinguish a normal controller handoff from a flight abort."""


def should_stop_controller_event(data):
    return (data.get('event',data.get('kind'))=='TERMINATION' and
            data.get('reason') not in ('ROS_SHUTDOWN','RACE_GOAL_CHANGED','ROUTE_END'))


def simulator_crashed(log,returncode):
    return (returncode in (-6,-7,-11,134,135,139) or
            any(signal in log for signal in ('SIGSEGV','SIGABRT','SIGBUS')))
