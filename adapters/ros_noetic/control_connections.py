"""Startup connection expectation: diagnostic readers cannot mask the backend."""


def expected_control_connections(graph,topic):
    publishers,subscribers,_=graph
    providers=set(dict(publishers).get('/airsim_node/drone_1/imu/imu',[]))
    readers=set(dict(subscribers).get(topic,[]))
    backend=providers&readers
    if not backend:raise RuntimeError('CONTROL_BACKEND_SUBSCRIBER_MISSING')
    return len(readers),sorted(backend)
