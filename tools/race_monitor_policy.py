"""Distinguish a normal controller handoff from a flight abort."""


def should_stop_controller_event(data):
    return (data.get('event',data.get('kind'))=='TERMINATION' and
            data.get('reason') not in ('ROS_SHUTDOWN','RACE_GOAL_CHANGED','ROUTE_END'))
