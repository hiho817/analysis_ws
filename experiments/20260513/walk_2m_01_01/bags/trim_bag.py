#!/usr/bin/env python3
"""
Trim odom_fusion20260512_222613_0.db3 to the trigger ON/OFF window.

Trigger ON  storage_ts : 1778595998802159807  (enable=True)
Trigger OFF storage_ts : 1778596022818128016  (enable=False)
Window (~24 s) + 2 s buffer on each side.

Output: odom_fusion20260512_222613_trimmed/odom_fusion20260512_222613_trimmed_0.db3
"""

import sqlite3
import shutil
import os
import yaml
from pathlib import Path

SRC_DB = Path(__file__).parent / "odom_fusion20260512_222613" / "odom_fusion20260512_222613_0.db3"
OUT_DIR = Path(__file__).parent / "odom_fusion20260512_222613_trimmed"
OUT_DB  = OUT_DIR / "odom_fusion20260512_222613_trimmed_0.db3"

TRIGGER_ON_TS  = 1778595998802159807
TRIGGER_OFF_TS = 1778596022818128016
BUFFER_NS = 2_000_000_000   # 2 seconds

T_MIN = TRIGGER_ON_TS  - BUFFER_NS
T_MAX = TRIGGER_OFF_TS + BUFFER_NS

print(f"Source: {SRC_DB}")
print(f"Window: {T_MIN} → {T_MAX}  ({(T_MAX-T_MIN)/1e9:.1f} s)")

OUT_DIR.mkdir(exist_ok=True)

# ── Copy schema + data ────────────────────────────────────────────────────────
src = sqlite3.connect(str(SRC_DB))
dst = sqlite3.connect(str(OUT_DB))

# Write same schema
schema_sql = src.execute("SELECT sql FROM sqlite_master WHERE type='table'").fetchall()
for (sql,) in schema_sql:
    if sql:
        dst.execute(sql)
dst.commit()

# Copy topics table verbatim
topics = src.execute("SELECT * FROM topics").fetchall()
dst.executemany("INSERT INTO topics VALUES (?,?,?,?,?)", topics)
dst.commit()
print(f"Copied {len(topics)} topics")

# Copy messages in window
cur_src = src.execute(
    "SELECT * FROM messages WHERE timestamp >= ? AND timestamp <= ?",
    (T_MIN, T_MAX)
)
batch = cur_src.fetchall()
dst.executemany("INSERT INTO messages VALUES (?,?,?,?)", batch)
dst.commit()
print(f"Copied {len(batch)} messages  ({len(batch)} / total)")

src.close()

# Quick verify: count per topic
print("\nMessages per topic in trimmed bag:")
for (tid, name, *_) in topics:
    cnt = dst.execute(
        "SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,)
    ).fetchone()[0]
    print(f"  {name}: {cnt}")

dst.close()

# ── Write metadata.yaml ───────────────────────────────────────────────────────
first_ts = batch[0][0]
last_ts  = batch[-1][0]
duration_ns = last_ts - first_ts

meta = {
    'rosbag2_bagfile_information': {
        'version': 5,
        'storage_identifier': 'sqlite3',
        'duration': {'nanoseconds': int(duration_ns)},
        'starting_time': {'nanoseconds_since_epoch': int(first_ts)},
        'message_count': len(batch),
        'compression_format': '',
        'compression_mode': '',
        'relative_file_paths': ['odom_fusion20260512_222613_trimmed_0.db3'],
        'files': [{
            'path': 'odom_fusion20260512_222613_trimmed_0.db3',
            'starting_time': {'nanoseconds_since_epoch': int(first_ts)},
            'duration': {'nanoseconds': int(duration_ns)},
            'message_count': len(batch),
        }],
        'topics_with_message_count': []
    }
}

for (tid, name, *_) in topics:
    cnt_val = sqlite3.connect(str(OUT_DB)).execute(
        "SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,)
    ).fetchone()[0]
    meta['rosbag2_bagfile_information']['topics_with_message_count'].append({
        'topic_metadata': {'name': name},
        'message_count': cnt_val,
    })

with open(OUT_DIR / "metadata.yaml", "w") as f:
    yaml.dump(meta, f, default_flow_style=False)

print(f"\nDone! Trimmed bag: {OUT_DB}  ({OUT_DB.stat().st_size / 1e6:.1f} MB)")
