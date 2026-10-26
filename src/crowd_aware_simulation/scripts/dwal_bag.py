"""Streaming rosbag reader; importable without ROS for numerical tests."""
import json
import math
from pathlib import Path


def locate(path):
    path = Path(path).resolve()
    bag = path/'bag' if (path/'bag').is_dir() else path
    if not (bag/'metadata.yaml').exists() and not list(bag.glob('*.db3')):
        raise ValueError(f'No bag metadata or SQLite recordings at {bag}')
    meta = next((p for p in (path/'evaluation.json', bag.parent/'evaluation.json') if p.exists()), None)
    if meta is None: raise ValueError('Missing evaluation.json; use experiment.py record for reproducible bags')
    return bag, json.loads(meta.read_text())


def events(path, topics=None):
    bag, _ = locate(path)
    if not (bag/'metadata.yaml').exists():
        yield from sqlite_events(bag, topics)
        return
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    bag, _ = locate(path)
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata.get('storage_identifier') or ('sqlite3' if list(bag.glob('*.db3')) else '')),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics is not None:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    classes = {}
    while reader.has_next():
        topic, raw, ns = reader.read_next()
        if topic not in classes:
            try: classes[topic] = get_message(types[topic])
            except Exception as error:
                raise RuntimeError(f'Cannot deserialize {topic} ({types[topic]}). Source the rebuilt ROS workspace.') from error
        yield ns*1e-9, topic, deserialize_message(raw, classes[topic])


def twist(msg):
    return msg.twist if hasattr(msg, 'twist') else msg


def sample_geometry(msg, reference_radius):
    """Return index, curvature, comparable phi, endpoint radius and checked length."""
    origin = msg.pose0
    if len(origin) < 3: return []
    result = []
    for i, path in enumerate(msg.paths):
        poses = path.poses
        if not poses or not math.isfinite(path.curvature): continue
        k = float(path.curvature)
        # Matches the upstream arc-then-tangent geometry's curv2phi convention.
        if abs(k)*reference_radius < math.sqrt(2):
            phi = math.asin(.5*k*reference_radius)
        else:
            phi = math.copysign(math.acos(1/(abs(k)*reference_radius)), k)
        length = sum(math.hypot(b.x-a.x, b.y-a.y) for a,b in zip(poses, poses[1:]))
        radius = math.hypot(poses[-1].x-origin[0], poses[-1].y-origin[1])
        if all(math.isfinite(x) for x in (length, radius, phi)):
            result.append({'index': i, 'curvature': k, 'phi': phi, 'r': radius, 'length': length})
    return result


def sqlite_rows(bag, topics=None):
    """Read a finite, consistent snapshot per SQLite shard, without modifying it."""
    import sqlite3
    import heapq
    from contextlib import ExitStack
    with ExitStack() as stack:
        streams = []
        for file in sorted(Path(bag).glob('*.db3')):
            db = sqlite3.connect(file.resolve().as_uri() + '?mode=ro', uri=True)
            stack.callback(db.close)
            db.execute('BEGIN')
            types = {i: (name, kind, fmt) for i, name, kind, fmt in
                     db.execute('SELECT id,name,type,serialization_format FROM topics')}
            selected = {i: value for i,value in types.items()
                        if topics is None or value[0] in topics}
            if any(value[2] != 'cdr' for value in selected.values()):
                raise ValueError('SQLite fallback requires CDR serialization')
            if not selected:
                continue
            placeholders = ','.join('?' for _ in selected)
            cursor = db.execute(
                'SELECT timestamp,topic_id,data FROM messages WHERE topic_id IN ('
                + placeholders + ') ORDER BY timestamp,id', tuple(selected))
            def rows(cursor, selected):
                for ns, topic_id, raw in cursor:
                    name, kind, _ = selected[topic_id]
                    yield ns, name, kind, raw
            streams.append(rows(cursor, selected))
        yield from heapq.merge(*streams, key=lambda row: row[0])


def sqlite_events(bag, topics=None):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import warnings
    warnings.warn('Bag metadata missing: reading saved SQLite messages directly. '
                  'An active recording is read as a snapshot; use plot.py live '
                  'for continuous live display. An interrupted recording may have lost its final messages.')
    classes = {}
    for ns, topic, kind, raw in sqlite_rows(bag, topics):
        if kind not in classes:
            classes[kind] = get_message(kind)
        yield ns*1e-9, topic, deserialize_message(raw, classes[kind])
