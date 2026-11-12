"""
Replay a recorded /scan through the tracker, with other settings than the node had.

Used to tune the settings at home from recordings made with the robot:

    ros2 run lidar_person_tracker replay_bag ~/walk_test3 facing_angle_rad=3.14159

It prints what the tracker would have said every half second, and how many
scans showed 0, 1, 2 ... people.
"""

import argparse
from dataclasses import fields
import sys

from lidar_person_tracker.tracker import PersonTracker, TrackerConfig

PRINT_EVERY_S = 0.5


def parse_settings(pairs):
    """Turn ['name=value', ...] into tracker settings; raise ValueError on a bad one."""
    types = {field.name: type(field.default) for field in fields(TrackerConfig)}
    settings = {}
    for pair in pairs:
        name, sep, value = pair.partition('=')
        if not sep or name not in types:
            raise ValueError(f'unknown setting {pair!r}, use one of: {", ".join(types)}')
        settings[name] = types[name](value)
    return settings


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('bag', help='folder of the recording')
    parser.add_argument('settings', nargs='*', help='name=value, for example path_distance_m=1.2')
    args = parser.parse_args(argv)
    try:
        tracker = PersonTracker(TrackerConfig(**parse_settings(args.settings)))
    except ValueError as error:
        parser.error(str(error))

    # Imported here so that parse_settings can be tested without ROS.
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import LaserScan

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    start_s, last_print_s, counts = None, None, {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/scan':
            continue
        scan = deserialize_message(data, LaserScan)
        stamp_s = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        start_s = stamp_s if start_s is None else start_s
        try:
            estimate = tracker.update(stamp_s, scan.ranges, scan.angle_min, scan.angle_increment,
                                      scan.range_min, scan.range_max)
        except ValueError as error:
            print(f'{stamp_s - start_s:6.1f}  scan ignored: {error}')
            continue
        if not estimate.ready:
            continue
        counts[estimate.people_count] = counts.get(estimate.people_count, 0) + 1
        if last_print_s is None or stamp_s - last_print_s >= PRINT_EVERY_S:
            last_print_s = stamp_s
            print(f'{stamp_s - start_s:6.1f} s  people={estimate.people_count}  '
                  f'position={estimate.position_m:6.2f} m  speed={estimate.speed_mps:6.2f} m/s')
    if start_s is None:
        sys.exit(f'no /scan messages in {args.bag}')
    print('scans with 0, 1, 2 ... people:', dict(sorted(counts.items())))


if __name__ == '__main__':
    main()
