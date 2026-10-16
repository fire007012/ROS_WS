#!/usr/bin/env python3
"""Read-only snapshots for a running mission; never publishes commands or calls services."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import os
from pathlib import Path
import shutil
import subprocess


def read_command(args):
    if shutil.which(args[0]) is None:
        return args, 'command unavailable: ' + args[0]
    try:
        result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, errors='replace', timeout=4)
        return args, result.stdout.strip() or '(no output)'
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or b''
        if isinstance(output, bytes):
            output = output.decode('utf-8', errors='replace')
        return args, output + '\nTIMEOUT: no response within 4 seconds'
    except OSError as exc:
        return args, str(exc)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='mission_diagnostics.txt')
    args = parser.parse_args()
    commands = [['rosnode', 'list']]
    for topic in ['/mission/state', '/robot_state', '/path_tracker/avoidance_state',
                  '/cmd_vel_mux/selected_source', '/distance_safety/state', '/system_health',
                  '/cmd_vel_requested', '/cmd_vel_muxed', '/path_points', '/odom/pose/pose',
                  '/video_sender_master/status', '/video_stream_master/status',
                  '/video_sender_slave/status', '/video_stream_slave/status',
                  '/audio_chat/master/peer_ready', '/audio_chat/slave/peer_ready',
                  '/fine_tuning/status', '/front/range', '/left/range', '/right/range',
                  '/rear/range', '/arm_upper_left/range', '/arm_upper_right/range']:
        commands.append(['rostopic', 'echo', '-n', '1', topic])
    for node in ['/video_sender_master', '/video_stream_master',
                 '/video_sender_slave', '/video_stream_slave']:
        commands.append(['rosnode', 'info', node])
    header = ['Read-only mission snapshot ' + datetime.now().isoformat(),
              'Snapshots are concurrent, not a synchronized recording.',
              'ROS_MASTER_URI=' + os.environ.get('ROS_MASTER_URI', ''),
              'ROS_IP=' + os.environ.get('ROS_IP', ''),
              'CPU count=' + str(os.cpu_count())]
    if Path('/proc/loadavg').exists():
        header.append('loadavg=' + Path('/proc/loadavg').read_text().strip())
    with ThreadPoolExecutor(max_workers=4) as pool:
        outputs = list(pool.map(read_command, commands))
    text = '\n'.join(header) + '\n\n' + '\n\n'.join(
        '$ ' + ' '.join(command) + '\n' + output for command, output in outputs)
    path = Path(args.output).expanduser().resolve()
    path.write_text(text, encoding='utf-8')
    print('Saved read-only diagnostic report: ' + str(path))
    print('No start, stop, reset, motion or sensor commands were sent.')


if __name__ == '__main__':
    main()
