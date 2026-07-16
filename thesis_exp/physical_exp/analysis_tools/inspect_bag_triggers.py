#!/usr/bin/env python3
"""Print trigger record time, message time, and enable value from ROS 2 bags."""

import argparse
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from corgi_msgs.msg import TriggerStamped


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    args = parser.parse_args()
    for bag in args.bags:
        db = next(bag.glob("*.db3"))
        connection = sqlite3.connect(db)
        cursor = connection.cursor()
        row = cursor.execute(
            "SELECT id FROM topics WHERE name='/trigger'").fetchone()
        print(f"== {bag} ==")
        if row is None:
            print("no /trigger")
            connection.close()
            continue
        records = cursor.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (row[0],),
        ).fetchall()
        t0 = records[0][0]
        for timestamp, data in records:
            msg = deserialize_message(data, TriggerStamped)
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            print(
                f"record_dt={(timestamp - t0) * 1e-9:9.3f}s "
                f"msg_time={msg_time:.9f} enable={msg.enable}"
            )
        connection.close()


if __name__ == "__main__":
    main()
