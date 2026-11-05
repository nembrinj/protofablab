"""Export pose overlays and the exact step candidates used by cadence.py."""
from pathlib import Path
import argparse
import csv
import subprocess

import cv2
import imageio_ffmpeg
import numpy as np

from .cadence import detect_step_candidates
from .paths import VIDEO_PATH, MODEL_PATH, OUTPUT_DIR
from .tracking import extract_pose_tracks

def export_debug_video(video_path: Path, model_path: Path, output_dir: Path):
    tracks = extract_pose_tracks(video_path, model_path, keep_poses=True)
    fps = tracks.info.fps
    poses = tracks.poses
    frame_indices = tracks.frame_indices
    times = tracks.timestamps
    left_y, right_y = tracks.left_y, tracks.right_y
    left_visibility, right_visibility = tracks.left_visibility, tracks.right_visibility
    output_dir.mkdir(parents=True, exist_ok=True)
    candidates = detect_step_candidates(times, np.asarray(left_y), np.asarray(right_y),
                                        left_visibility=left_visibility, right_visibility=right_visibility)
    peaks = np.asarray([event.index for event in candidates], dtype=int)
    events = np.asarray(frame_indices)[peaks]
    output = output_dir / 'walk_debug.mp4'
    with (output_dir / 'step_candidates.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['candidate', 'frame', 'time_s', 'left_ankle_y', 'right_ankle_y', 'foot', 'prominence'])
        for number, peak in enumerate(peaks, 1):
            writer.writerow([number, frame_indices[peak], times[peak], left_y[peak], right_y[peak], candidates[number-1].foot, candidates[number-1].prominence])
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"Cannot reopen video: {video_path}")
    # Browser-compatible silent preview, scaled down to keep notebook playback light.
    width = 960
    height = max(2, round(tracks.info.height / tracks.info.width * width / 2) * 2)
    command = [imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error',
               '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{width}x{height}',
               '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264',
               '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)]
    try:
        with subprocess.Popen(command, stdin=subprocess.PIPE) as encoder:
            try:
                for index, pose in enumerate(poses):
                    ok, frame = capture.read()
                    if not ok:
                        raise RuntimeError('Video ended during debug rendering')
                    frame = cv2.resize(frame, (width, height))
                    if pose:
                        for a, b in [(11,12),(11,23),(12,24),(23,24),(23,25),(25,27),(24,26),(26,28),(27,29),(29,31),(28,30),(30,32)]:
                            p, q = pose[a], pose[b]
                            cv2.line(frame, (int(p.x*width), int(p.y*height)), (int(q.x*width), int(q.y*height)), (0,220,0), 2)
                        for ankle, label, color in [(27,'L',(255,180,0)),(28,'R',(0,180,255))]:
                            p = pose[ankle]
                            point = (int(p.x*width), int(p.y*height))
                            cv2.circle(frame, point, 7, color, -1)
                            cv2.putText(frame, f'{label} vis={p.visibility:.2f}', point, cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
                    count = int(np.searchsorted(events, index, side='right'))
                    flash = count > 0 and index - events[count-1] <= fps * .2
                    cv2.rectangle(frame, (0,0), (width,88), (0,0,0), -1)
                    cv2.putText(frame, f'{index/fps:.2f}s | frame {index} | candidates: {count}/{len(events)}', (12,26), cv2.FONT_HERSHEY_SIMPLEX, .65, (255,255,255), 2)
                    status = f'STEP CANDIDATE: {candidates[count-1].foot}' if flash else ('Pose detected' if pose else 'NO POSE - tracking gap')
                    cv2.putText(frame, status, (12,58), cv2.FONT_HERSHEY_SIMPLEX, .65, (0,255,255) if flash else (255,255,255), 2)
                    encoder.stdin.write(frame.tobytes())
            finally:
                encoder.stdin.close()
            if encoder.wait():
                raise RuntimeError('FFmpeg could not encode the debug video')
    finally:
        capture.release()
    print(f'Saved {output} ({len(events)} candidates, pose in {len(times)}/{len(poses)} frames)')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', nargs='?', type=Path, default=VIDEO_PATH)
    parser.add_argument('--model', type=Path, default=MODEL_PATH)
    parser.add_argument('--output-dir', type=Path, default=OUTPUT_DIR)
    args = parser.parse_args(argv)
    export_debug_video(args.video, args.model, args.output_dir)
