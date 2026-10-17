"""Replay archived LiDAR scans into an isolated slam_toolbox UDP endpoint.

Never use this on the live robot's endpoint. Output is an offline reconstruction.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import socket
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('reports', nargs='+', type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--port', type=int, default=19760)
    args = parser.parse_args()
    if len(args.reports) != 3:
        parser.error('Supply the three archived reports in A, B, C order.')
    if args.port == 9760:
        parser.error('Use an isolated endpoint, not the live default port 9760.')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for label, path in zip('ABC', args.reports):
        data = json.loads(path.read_text())
        accepted = next(h['time'] for h in data['history'] if h['start_matching_accepted'])
        csv_path = path.with_name(path.stem + '_lidar_reconstruction.csv')
        scans = collections.defaultdict(list)
        with csv_path.open() as file:
            for row in csv.DictReader(file):
                if float(row['time']) >= accepted:
                    scans[float(row['time'])].append({k: float(v) for k, v in row.items() if k != 'pose_source'})
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', 0))
        sock.setblocking(False)
        session = 'report_offline_' + label + '_' + str(time.time_ns())
        poses, final_map, sequence_inputs = [], None, {}
        start = time.monotonic()

        def receive():
            nonlocal final_map
            while True:
                try:
                    packet, _ = sock.recvfrom(65507)
                except BlockingIOError:
                    return
                f = packet.decode().split('|')
                if f[0] != 'MAP' or f[1] != session:
                    continue
                pose = dict(sequence=int(f[2]), valid=bool(int(f[3])), x=float(f[4]), y=float(f[5]),
                            yaw=float(f[6]), map_updates=int(f[7]), graph_nodes=int(f[8]), loop_edges=int(f[9]))
                pose.update(sequence_inputs.get(pose['sequence'], {}))
                poses.append(pose)
                if int(f[13]) > 1 and f[15]:
                    final_map = dict(resolution=float(f[10]), origin_x=float(f[11]), origin_y=float(f[12]),
                                     width=int(f[13]), height=int(f[14]),
                                     occupied=[int(i) for i in f[15].split(',') if i],
                                     free=[int(i) for i in f[16].split(',') if i], **pose)

        for sequence, (timestamp, rows) in enumerate(sorted(scans.items()), 1):
            deadline = start + timestamp - accepted
            while time.monotonic() < deadline:
                receive()
                time.sleep(min(.01, max(0, deadline - time.monotonic())))
            row = rows[0]
            x, y, yaw = row['pose_x'], row['pose_y'], row['pose_yaw']
            ox = sum(r['hit_x'] - math.cos(r['angle_world_rad']) * r['range_m'] for r in rows) / len(rows) - x
            oy = sum(r['hit_y'] - math.sin(r['angle_world_rad']) * r['range_m'] for r in rows) / len(rows) - y
            dx, dy = math.cos(yaw) * ox + math.sin(yaw) * oy, -math.sin(yaw) * ox + math.cos(yaw) * oy
            # This CSV contains hit endpoints only (not synthetic no-return rays).
            beams = ';'.join(f"{r['angle_world_rad']},{r['range_m']},1" for r in rows)
            sequence_inputs[sequence] = dict(time=timestamp, odom_x=x, odom_y=y, odom_yaw=yaw)
            payload = f'SCAN|{session}|{sequence}|{timestamp}|{x}|{y}|{yaw}|0.04|1.2|{dx}|{dy}|{beams}'
            sock.sendto(payload.encode(), ('127.0.0.1', args.port))
            receive()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            receive()
            time.sleep(.01)
        sock.close()
        if not final_map or not poses:
            raise RuntimeError(f'No SLAM occupancy output for run {label}')
        result = dict(label=label, source=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      csv_sha256=hashlib.sha256(csv_path.read_bytes()).hexdigest(), session=session,
                      provenance='Offline slam_toolbox replay, starting at accepted start pose; recorded scan spacing.',
                      scan_count=len(scans), poses=poses, final_map=final_map)
        (args.output_dir / f'slam_replay_{label}.json').write_text(json.dumps(result))
        print(json.dumps(dict(run=label, scans=len(scans), responses=len(poses),
                              occupied=len(final_map['occupied']), nodes=final_map['graph_nodes'])), flush=True)


if __name__ == '__main__':
    main()
