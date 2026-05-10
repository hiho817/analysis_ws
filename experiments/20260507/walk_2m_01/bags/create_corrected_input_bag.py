#!/usr/bin/env python3
"""
Create a corrected input bag from the original 0507 bag.
- Keeps only INPUT topics: /motor/state, /imu_raw, /trigger
- For /motor/state: divides velocity_r and velocity_l by 2 for all modules
- Output: corrected_input/ folder next to this script
"""

import sqlite3
import shutil
import os
import sys
import yaml

# Must be run with ROS2 environment sourced
from rclpy.serialization import deserialize_message, serialize_message
from corgi_msgs.msg import MotorStateStamped

SRC_DB = os.path.join(os.path.dirname(__file__),
                      'leg_odom20260507_161231',
                      'leg_odom20260507_161231_0.db3')
OUT_DIR = os.path.join(os.path.dirname(__file__), 'corrected_input')
OUT_DB  = os.path.join(OUT_DIR, 'corrected_input_0.db3')

# Only replay these input topics
INPUT_TOPICS = {'/motor/state', '/imu_raw', '/trigger'}

def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    src = sqlite3.connect(SRC_DB)
    dst = sqlite3.connect(OUT_DB)

    # Create tables
    dst.execute("""
        CREATE TABLE IF NOT EXISTS topics (
            id INTEGER PRIMARY KEY,
            name TEXT NOT NULL,
            type TEXT NOT NULL,
            serialization_format TEXT NOT NULL,
            offered_qos_profiles TEXT NOT NULL
        )
    """)
    dst.execute("""
        CREATE TABLE IF NOT EXISTS messages (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            topic_id INTEGER NOT NULL,
            timestamp INTEGER NOT NULL,
            data BLOB NOT NULL
        )
    """)
    dst.commit()

    # Copy topics (input only), build id remap
    src_topics = src.execute(
        "SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics"
    ).fetchall()

    id_map = {}  # old_id -> new_id
    motor_state_new_id = None
    for row in src_topics:
        old_id, name, typ, sfmt, qos = row
        if name not in INPUT_TOPICS:
            continue
        dst.execute(
            "INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) "
            "VALUES (?, ?, ?, ?)",
            (name, typ, sfmt, qos)
        )
        new_id = dst.execute("SELECT last_insert_rowid()").fetchone()[0]
        id_map[old_id] = new_id
        if name == '/motor/state':
            motor_state_new_id = new_id
        print(f"  Topic {name}: old_id={old_id} -> new_id={new_id}")
    dst.commit()

    # Copy & patch messages
    old_ids_str = ','.join(str(k) for k in id_map.keys())
    rows = src.execute(
        f"SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN ({old_ids_str})"
        f" ORDER BY timestamp"
    ).fetchall()

    print(f"\nProcessing {len(rows)} messages...")
    patched = 0
    batch = []
    for i, (topic_id, ts, data) in enumerate(rows):
        new_topic_id = id_map[topic_id]
        if topic_id == list(id_map.keys())[list(id_map.values()).index(motor_state_new_id)]:
            # Patch motor/state velocity
            msg = deserialize_message(bytes(data), MotorStateStamped)
            for module in (msg.module_a, msg.module_b, msg.module_c, msg.module_d):
                module.velocity_r /= 2.0
                module.velocity_l /= 2.0
            data = serialize_message(msg)
            patched += 1
        batch.append((new_topic_id, ts, data))
        if len(batch) >= 1000:
            dst.executemany(
                "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
                batch
            )
            dst.commit()
            batch.clear()
            print(f"  ... {i+1}/{len(rows)} done", end='\r')

    if batch:
        dst.executemany(
            "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
            batch
        )
        dst.commit()

    print(f"\nPatched {patched} /motor/state messages (velocity ÷ 2)")

    # Build metadata.yaml
    topic_meta = {}
    for new_id, in dst.execute("SELECT id FROM topics").fetchall():
        name, typ = dst.execute(
            "SELECT name, type FROM topics WHERE id=?", (new_id,)
        ).fetchone()
        count, tmin, tmax = dst.execute(
            "SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages WHERE topic_id=?",
            (new_id,)
        ).fetchone()
        topic_meta[name] = {'type': typ, 'count': count, 'tmin': tmin, 'tmax': tmax}

    tmin_all = min(v['tmin'] for v in topic_meta.values())
    tmax_all = max(v['tmax'] for v in topic_meta.values())
    total    = sum(v['count'] for v in topic_meta.values())
    bag_name = 'corrected_input_0.db3'

    metadata = {'rosbag2_bagfile_information': {
        'version': 6,
        'storage_identifier': 'sqlite3',
        'duration': {'nanoseconds': int(tmax_all - tmin_all)},
        'starting_time': {'nanoseconds_since_epoch': int(tmin_all)},
        'message_count': total,
        'topics_with_message_count': [
            {'topic_metadata': {
                'name': n,
                'type': v['type'],
                'serialization_format': 'cdr',
                'offered_qos_profiles': ''},
             'message_count': v['count']}
            for n, v in topic_meta.items()],
        'compression_format': '',
        'compression_mode': '',
        'relative_file_paths': [bag_name],
        'files': [{'path': bag_name,
                   'starting_time': {'nanoseconds_since_epoch': int(tmin_all)},
                   'duration': {'nanoseconds': int(tmax_all - tmin_all)},
                   'message_count': total}]
    }}

    meta_path = os.path.join(OUT_DIR, 'metadata.yaml')
    with open(meta_path, 'w') as f:
        yaml.dump(metadata, f, default_flow_style=False)

    src.close()
    dst.close()

    print(f"\nOutput bag: {OUT_DIR}")
    print(f"  metadata.yaml written")
    for n, v in topic_meta.items():
        print(f"  {n}: count={v['count']}")
    print("Done.")

if __name__ == '__main__':
    main()
