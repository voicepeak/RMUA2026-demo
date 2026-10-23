"""Exact bounded ROS sensor payload retention; never modifies source messages."""
import gzip
import hashlib
import json
from pathlib import Path


def save_message(directory, prefix, message, kind):
    if prefix not in ('left', 'right', 'lidar') or kind not in ('image', 'lidar'):
        raise ValueError('unsupported captured sensor')
    directory = Path(directory)
    data = bytes(message.data)
    if len(data) > 32 * 1024 * 1024:
        raise ValueError('oversized captured payload')
    stamp = message.header.stamp
    document = {'kind': kind, 'stamp_secs': stamp.secs, 'stamp_nsecs': stamp.nsecs,
                'stamp': stamp.to_sec(), 'sequence': message.header.seq,
                'frame_id': message.header.frame_id, 'width': message.width, 'height': message.height,
                'is_bigendian': message.is_bigendian, 'payload_file': prefix + '.bin.gz',
                'raw_bytes': len(data), 'raw_sha256': hashlib.sha256(data).hexdigest()}
    if kind == 'image':
        document.update(encoding=message.encoding, step=message.step)
    else:
        document.update(point_step=message.point_step, row_step=message.row_step, is_dense=message.is_dense,
                        fields=[{'name': f.name, 'offset': f.offset, 'datatype': f.datatype, 'count': f.count}
                                for f in message.fields])
    with (directory / document['payload_file']).open('xb') as raw:
        with gzip.GzipFile(filename='', mode='wb', fileobj=raw, compresslevel=1, mtime=0) as output:
            output.write(data)
    with (directory / (prefix + '.json')).open('x') as output:
        output.write(json.dumps(document, indent=2, allow_nan=False) + '\n')
    return document


def read_payload(path):
    path = Path(path)
    document = json.loads(path.read_text())
    size = document['raw_bytes']
    name = document['payload_file']
    if (not isinstance(size, int) or not 0 <= size <= 32 * 1024 * 1024
            or Path(name).name != name or name in ('.', '..')):
        raise ValueError('invalid retained sensor descriptor')
    with gzip.open(path.parent / name, 'rb') as source:
        payload = source.read(size + 1)
    if len(payload) != size or hashlib.sha256(payload).hexdigest() != document['raw_sha256']:
        raise ValueError('captured payload differs from its descriptor')
    return document, payload


def load_message(path):
    import rospy
    from sensor_msgs.msg import Image, PointCloud2, PointField

    document, payload = read_payload(path)
    if document['kind'] == 'image':
        message = Image()
        keys = ('width', 'height', 'step', 'is_bigendian', 'encoding')
    elif document['kind'] == 'lidar':
        message = PointCloud2()
        keys = ('width', 'height', 'point_step', 'row_step', 'is_bigendian', 'is_dense')
        message.fields = [PointField(**f) for f in document['fields']]
    else:
        raise ValueError('unknown captured sensor kind')
    message.header.stamp = rospy.Time(document['stamp_secs'], document['stamp_nsecs'])
    message.header.seq = document['sequence']
    message.header.frame_id = document['frame_id']
    for key in keys:
        setattr(message, key, document[key])
    message.data = payload
    if message.header.stamp.to_sec() != document['stamp']:
        raise ValueError('captured timestamp did not reconstruct exactly')
    return message
