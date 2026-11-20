"""
Replay a recorded /scan through the tracker, with other settings than the node had.

Used to tune the settings at home from recordings made with the robot:

    ros2 run lidar_person_tracker replay_bag ~/walk_test3 facing_angle_rad=3.14159

When the recording has /odom, the tracker gets it too, as on the robot, so the
driving mode can be tried with --mode driving.

It prints what the tracker would have said every half second, how many scans
showed 0, 1, 2 ... people, and how many steps it counted. With --csv it also
writes every scan to a file, for plots.
"""

import argparse
from contextlib import nullcontext
import csv
from dataclasses import fields
import sys

from lidar_person_tracker.tracker import MODES, PersonTracker, TrackerConfig

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
    parser.add_argument('--csv', metavar='FILE', help='also write every scan to this file')
    parser.add_argument('--mode', choices=MODES, default='static', help='tracker mode')
    args = parser.parse_args(argv)
    try:
        tracker = PersonTracker(TrackerConfig(**parse_settings(args.settings)))
    except ValueError as error:
        parser.error(str(error))
    tracker.set_mode(args.mode)
    try:
        csv_file = open(args.csv, 'w', newline='') if args.csv else nullcontext()
    except OSError as error:
        parser.error(f'cannot write {args.csv}: {error}')
    with csv_file:
        replay(args.bag, tracker, csv.writer(csv_file) if args.csv else None)


def replay(bag, tracker, rows):
    """Feed every /scan and /odom of the bag to the tracker and print what it says."""
    # Imported here so that parse_settings can be tested without ROS.
    from lidar_person_tracker.tracker_node import stamp_seconds, yaw_of
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import LaserScan

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    if rows:
        rows.writerow(['time_s', 'people', 'position_m', 'speed_mps', 'spread_m', 'steps',
                       'following', 'moved'])
    start_s, last_print_s, counts, steps, odometry = None, None, {}, 0, 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/odom':
            message = deserialize_message(data, Odometry)
            pose = message.pose.pose
            try:
                tracker.add_odometry(stamp_seconds(message.header), pose.position.x,
                                     pose.position.y, yaw_of(pose.orientation))
                odometry += 1
            except ValueError as error:
                print(f'odometry ignored: {error}')
            continue
        if topic != '/scan':
            continue
        scan = deserialize_message(data, LaserScan)
        stamp_s = stamp_seconds(scan.header)
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
        steps = estimate.steps
        if rows:
            rows.writerow([f'{stamp_s - start_s:.3f}', estimate.people_count,
                           f'{estimate.position_m:.3f}', f'{estimate.speed_mps:.3f}',
                           f'{estimate.spread_m:.3f}', estimate.steps, int(estimate.following),
                           int(estimate.moved)])
        if last_print_s is None or stamp_s - last_print_s >= PRINT_EVERY_S:
            last_print_s = stamp_s
            print(f'{stamp_s - start_s:6.1f} s  people={estimate.people_count}  '
                  f'position={estimate.position_m:6.2f} m  speed={estimate.speed_mps:6.2f} m/s  '
                  f'steps={estimate.steps}'
                  + ('  following' if estimate.following else '')
                  + ('  robot moved' if estimate.moved else ''))
    if start_s is None:
        sys.exit(f'no /scan messages in {bag}')
    print('scans with 0, 1, 2 ... people:', dict(sorted(counts.items())))
    print('steps:', steps)
    print('odometry messages:', odometry)


if __name__ == '__main__':
    main()
