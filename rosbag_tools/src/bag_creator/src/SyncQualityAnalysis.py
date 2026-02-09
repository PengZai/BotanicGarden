#!/usr/bin/env python3
"""
Analyze time synchronization quality of cameras and LiDARs from ROS bag file
"""

import rosbag
import numpy as np
from collections import defaultdict
from datetime import datetime
import pandas as pd
import os


def extract_timestamps(bag_file):
    """Extract timestamps from all sensor topics"""
    print(f"Reading bag file: {bag_file}")

    # Define topics to analyze
    topics = {
        'cameras': [
             '/left/image_raw/compressed',
            '/forwardRight/image_raw/compressed',
            '/right/image_raw/compressed',
            '/forwardLeft/image_raw/compressed'
        ],
        'lidars': [
            '/ouster/points',
            '/velodyne_points'
        ]
    }

    # topics = {
    #     'cameras': [
    #         '/dalsa_rgb/left/image_raw',
    #         '/dalsa_rgb/right/image_raw'
    #     ],
    #     'lidars': [
    #         '/livox/lidar',
    #         '/velodyne_points'
    #     ]
    # }

    # topics = {
    #     'cameras': [
    #         '/left/image_raw/compressed',
    #         '/right/image_raw/compressed'
    #     ],
    #     'lidars': [
    #         '/ouster/points',
    #         '/velodyne_points'
    #     ]
    # }

    # Store timestamps for each topic
    timestamps = defaultdict(list)

    # Read bag file
    bag = rosbag.Bag(bag_file)

    all_topics = topics['cameras'] + topics['lidars']

    print("Extracting timestamps...")
    for topic, msg, t in bag.read_messages(topics=all_topics):
        # Store ROS time (t) in seconds
        timestamps[topic].append(t.to_sec())

    bag.close()

    # Convert to numpy arrays
    for topic in timestamps:
        timestamps[topic] = np.array(timestamps[topic])
        print(f"{topic}: {len(timestamps[topic])} messages")

    return timestamps, topics


def export_all_timestamps(timestamps):
    """Export all timestamps to CSV files"""
    print("\nExporting all timestamps to CSV...")

    output_dir = 'sync_analysis_results'
    os.makedirs(output_dir, exist_ok=True)

    # Export individual sensor timestamps
    for topic, ts in timestamps.items():
        sensor_name = topic.split('/')[-2] if 'image' in topic else topic.split('/')[-1]

        df = pd.DataFrame({
            'frame_number': range(len(ts)),
            'timestamp_sec': ts,
            'timestamp_readable': pd.to_datetime(ts, unit='s')
        })

        filename = os.path.join(output_dir, f'{sensor_name}_timestamps.csv')
        df.to_csv(filename, index=False)
        print(f"  Saved: {filename}")

    # Export combined timestamps (all sensors in one file)
    # Find min and max timestamps across all sensors
    all_ts = np.concatenate([ts for ts in timestamps.values()])
    min_time = np.min(all_ts)
    max_time = np.max(all_ts)

    print(f"\n  Creating combined timestamps file...")
    print(f"  Time range: {min_time:.3f} to {max_time:.3f} ({max_time - min_time:.3f} seconds)")

    # Create a combined dataframe with all timestamps
    combined_data = []
    for topic, ts in timestamps.items():
        sensor_name = topic.split('/')[-2] if 'image' in topic else topic.split('/')[-1]
        for frame_num, t in enumerate(ts):
            combined_data.append({
                'sensor': sensor_name,
                'topic': topic,
                'frame_number': frame_num,
                'timestamp_sec': t,
                'time_from_start_sec': t - min_time
            })

    combined_df = pd.DataFrame(combined_data)
    combined_df = combined_df.sort_values('timestamp_sec').reset_index(drop=True)

    combined_filename = os.path.join(output_dir, 'all_timestamps_combined.csv')
    combined_df.to_csv(combined_filename, index=False)
    print(f"  Saved: {combined_filename}")

    return output_dir


def compute_sync_metrics(timestamps, topics):
    """Compute synchronization metrics"""
    print("\n" + "=" * 80)
    print("TIME SYNCHRONIZATION ANALYSIS")
    print("=" * 80)

    metrics = {}

    # 1. Analyze frame rates
    print("\n1. FRAME RATES")
    print("-" * 80)
    for topic in timestamps:
        ts = timestamps[topic]
        if len(ts) > 1:
            diffs = np.diff(ts)
            avg_rate = 1.0 / np.mean(diffs)
            std_rate = np.std(diffs) * 1000  # in milliseconds

            print(f"\n{topic}:")
            print(f"  Messages: {len(ts)}")
            print(f"  Average rate: {avg_rate:.2f} Hz")
            print(f"  Period: {np.mean(diffs) * 1000:.2f} ± {std_rate:.2f} ms")
            print(f"  Min period: {np.min(diffs) * 1000:.2f} ms")
            print(f"  Max period: {np.max(diffs) * 1000:.2f} ms")

            metrics[topic] = {
                'count': len(ts),
                'avg_rate': avg_rate,
                'avg_period_ms': np.mean(diffs) * 1000,
                'std_period_ms': std_rate,
                'min_period_ms': np.min(diffs) * 1000,
                'max_period_ms': np.max(diffs) * 1000
            }

    # 2. Analyze synchronization between cameras
    print("\n\n2. CAMERA SYNCHRONIZATION")
    print("-" * 80)
    camera_topics = topics['cameras']

    if len(camera_topics) > 1:
        # Find nearest timestamps between cameras
        base_camera = camera_topics[0]
        base_ts = timestamps[base_camera]

        print(f"\nUsing {base_camera} as reference (with proper timestamp pairing)")

        for cam in camera_topics[1:]:
            cam_ts = timestamps[cam]

            # Use proper pairing to avoid reusing timestamps
            _, _, time_diffs = find_closest_pairs(base_ts, cam_ts, max_time_diff=0.05)
            time_diffs = time_diffs * 1000  # Convert to ms

            match_rate = len(time_diffs) / len(base_ts) * 100

            print(f"\n{cam} vs {base_camera}:")
            print(f"  Matched pairs: {len(time_diffs)} / {len(base_ts)} ({match_rate:.1f}%)")
            print(f"  Mean offset (signed): {np.mean(time_diffs):+.2f} ms")
            print(f"  Mean ABS offset: {np.mean(np.abs(time_diffs)):.2f} ms")
            print(f"  Std offset: {np.std(time_diffs):.2f} ms")
            print(f"  Max ABS offset: {np.max(np.abs(time_diffs)):.2f} ms")
            print(f"  95th percentile: {np.percentile(np.abs(time_diffs), 95):.2f} ms")

    # 3. Analyze synchronization between LiDARs
    print("\n\n3. LIDAR SYNCHRONIZATION")
    print("-" * 80)
    lidar_topics = topics['lidars']

    if len(lidar_topics) == 2:
        lidar1_ts = timestamps[lidar_topics[0]]
        lidar2_ts = timestamps[lidar_topics[1]]

        # Use proper pairing
        _, _, time_diffs = find_closest_pairs(lidar1_ts, lidar2_ts, max_time_diff=0.05)
        time_diffs = time_diffs * 1000  # Convert to ms

        match_rate = len(time_diffs) / len(lidar1_ts) * 100

        print(f"\n{lidar_topics[1]} vs {lidar_topics[0]}:")
        print(f"  Matched pairs: {len(time_diffs)} / {len(lidar1_ts)} ({match_rate:.1f}%)")
        print(f"  Mean offset (signed): {np.mean(time_diffs):+.2f} ms")
        print(f"  Mean ABS offset: {np.mean(np.abs(time_diffs)):.2f} ms")
        print(f"  Std offset: {np.std(time_diffs):.2f} ms")
        print(f"  Max ABS offset: {np.max(np.abs(time_diffs)):.2f} ms")
        print(f"  95th percentile: {np.percentile(np.abs(time_diffs), 95):.2f} ms")

    # 4. Camera-LiDAR synchronization
    print("\n\n4. CAMERA-LIDAR SYNCHRONIZATION")
    print("-" * 80)

    base_camera = camera_topics[0]
    base_ts = timestamps[base_camera]

    for lidar in lidar_topics:
        lidar_ts = timestamps[lidar]

        # Use proper pairing
        _, _, time_diffs = find_closest_pairs(base_ts, lidar_ts, max_time_diff=0.05)
        time_diffs = time_diffs * 1000  # Convert to ms

        match_rate = len(time_diffs) / len(base_ts) * 100

        print(f"\n{lidar} vs {base_camera}:")
        print(f"  Matched pairs: {len(time_diffs)} / {len(base_ts)} ({match_rate:.1f}%)")
        print(f"  Mean offset (signed): {np.mean(time_diffs):+.2f} ms")
        print(f"  Mean ABS offset: {np.mean(np.abs(time_diffs)):.2f} ms")
        print(f"  Std offset: {np.std(time_diffs):.2f} ms")
        print(f"  Max ABS offset: {np.max(np.abs(time_diffs)):.2f} ms")
        print(f"  95th percentile: {np.percentile(np.abs(time_diffs), 95):.2f} ms")

    return metrics


def find_closest_pairs(ts1, ts2, max_time_diff=0.05):
    """
    Find closest timestamp pairs between two sensors without reusing timestamps.
    Uses a greedy nearest-neighbor approach with a maximum time difference threshold.

    Args:
        ts1: timestamps from first sensor (reference)
        ts2: timestamps from second sensor
        max_time_diff: maximum allowed time difference in seconds (default 50ms)

    Returns:
        matched_ts1, matched_ts2, time_diffs
    """
    matched_ts1 = []
    matched_ts2 = []
    time_diffs = []

    # Create a set of used indices to avoid reusing timestamps
    used_indices = set()

    # For each timestamp in ts1, find the closest unused timestamp in ts2
    for t1 in ts1:
        # Calculate differences to all timestamps in ts2
        diffs = np.abs(ts2 - t1)

        # Sort by difference to find closest matches
        sorted_indices = np.argsort(diffs)

        # Find the closest unused timestamp
        matched = False
        for idx in sorted_indices:
            # FIX #1: Check BOTH conditions - not used AND within threshold
            if idx not in used_indices and diffs[idx] <= max_time_diff:
                matched_ts1.append(t1)
                matched_ts2.append(ts2[idx])
                time_diffs.append(ts2[idx] - t1)
                used_indices.add(idx)
                matched = True
                break

        if not matched:
            # No match found within threshold - skip this timestamp
            pass

    return np.array(matched_ts1), np.array(matched_ts2), np.array(time_diffs)


def analyze_ouster_sync(timestamps, topics):
    """Analyze all sensors relative to Ouster LiDAR with proper timestamp pairing"""
    print("\n" + "=" * 80)
    print("OUSTER LIDAR AS REFERENCE - DETAILED COMPARISON")
    print("=" * 80)

    ouster_topic = '/ouster/points'
    # ouster_topic = '/livox/lidar'

    ouster_ts = timestamps[ouster_topic]

    # Get all other topics
    other_topics = []
    for cam in topics['cameras']:
        other_topics.append(cam)
    for lidar in topics['lidars']:
        if lidar != ouster_topic:
            other_topics.append(lidar)

    sync_data = {}

    for topic in other_topics:
        topic_ts = timestamps[topic]

        # Find closest pairs without reusing timestamps
        ouster_matched, topic_matched, time_diffs = find_closest_pairs(
            ouster_ts, topic_ts, max_time_diff=0.05  # 50ms threshold
        )

        # Convert to milliseconds
        time_diffs_ms = time_diffs * 1000

        sensor_name = topic.split('/')[-2] if 'image' in topic else topic.split('/')[-1]

        match_rate = len(time_diffs_ms) / len(ouster_ts) * 100

        print(f"\n{sensor_name} vs Ouster:")
        print(f"  Ouster frames: {len(ouster_ts)}")
        print(f"  {sensor_name} frames: {len(topic_ts)}")
        print(f"  Matched pairs: {len(time_diffs_ms)} ({match_rate:.1f}%)")
        print(f"  Mean offset (signed): {np.mean(time_diffs_ms):+.3f} ms")
        print(f"  Mean ABS offset: {np.mean(np.abs(time_diffs_ms)):.3f} ms")  # FIX #2: Added absolute offset
        print(f"  Std offset: {np.std(time_diffs_ms):.3f} ms")
        print(f"  Median offset: {np.median(time_diffs_ms):+.3f} ms")
        print(f"  Min offset: {np.min(time_diffs_ms):+.3f} ms")
        print(f"  Max offset: {np.max(time_diffs_ms):+.3f} ms")
        print(f"  Max ABS offset: {np.max(np.abs(time_diffs_ms)):.3f} ms")
        print(f"  95th percentile: {np.percentile(np.abs(time_diffs_ms), 95):.3f} ms")
        print(f"  99th percentile: {np.percentile(np.abs(time_diffs_ms), 99):.3f} ms")

        sync_data[sensor_name] = {
            'time_diffs': time_diffs_ms,
            'ouster_times': ouster_matched,
            'sensor_times': topic_matched,
            'topic': topic
        }

    return sync_data


def main():
    bag_file = 'outdoor.bag'
    # bag_file = "/media/spiderman/zhipeng_8t1/datasets/PolyTunnel_haygrove_June2025/EASY.bag"
    # Extract timestamps
    timestamps, topics = extract_timestamps(bag_file)

    # Export all timestamps to CSV
    output_dir = export_all_timestamps(timestamps)

    # Compute metrics
    metrics = compute_sync_metrics(timestamps, topics)

    # Analyze synchronization relative to Ouster
    sync_data = analyze_ouster_sync(timestamps, topics)

    print("\n" + "=" * 80)
    print(f"Analysis complete! Results saved to: {output_dir}/")
    print("=" * 80)


if __name__ == '__main__':
    main()
