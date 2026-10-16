"""Capture the first CAN failure, with adapter retransmission disabled.

No ROS node or firmware is changed. The only new application TX is one fixed
STM32 heartbeat. Restores the original normal mode in finally.
"""
import collections
import datetime
import errno
import json
import os
import pathlib
import select
import socket
import struct
import subprocess
import time

DEVICE = 'can0'
ROOT = pathlib.Path('/home/fire/ros_ws/src/docs/can_ack_evidence_20261016')
FRAME = struct.Struct('=IB3x8s')
SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)
report = {'started': datetime.datetime.now().astimezone().isoformat(),
          'events': [], 'counts': {}}
records = []


def event(kind, **fields):
    item = dict(time=time.time(), kind=kind, **fields)
    report['events'].append(item)
    print(json.dumps(item), flush=True)


def snapshot():
    links = json.loads(subprocess.check_output([
        'ip', '-j', '-d', '-s', 'link', 'show', 'dev', DEVICE]))
    if len(links) != 1:
        raise RuntimeError('Expected one CAN interface')
    return links[0]


def ip(*args):
    command = ['ip', 'link', 'set', 'dev', DEVICE, *args]
    event('configure_begin', command=command)
    proc = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          universal_newlines=True, timeout=10)
    event('configure_end', returncode=proc.returncode, stderr=proc.stderr.strip())
    proc.check_returncode()


def capture(sock, stage, seconds):
    counts = collections.Counter()
    event('capture_begin', stage=stage)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not select.select([sock], [], [], min(0.05, max(0, deadline-time.monotonic())))[0]:
            continue
        try:
            raw, ancillary, flags, _ = sock.recvmsg(FRAME.size, 128)
        except OSError as exc:
            # A CAN socket opened before link-down receives an asynchronous
            # ENETDOWN notification even if link-up has already succeeded.
            if exc.errno in (errno.ENETDOWN, errno.EAGAIN, errno.EINTR):
                event('socket_notification', stage=stage, errno=exc.errno)
                continue
            raise
        timestamp = time.time()
        for level, kind, data in ancillary:
            if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS:
                sec, nsec = struct.unpack('@ll', data[:struct.calcsize('@ll')])
                timestamp = sec + nsec / 1e9
        if len(raw) != FRAME.size:
            counts['truncated'] += 1
            continue
        can_id, dlc, data = FRAME.unpack(raw)
        item = dict(stage=stage, time=timestamp, can_id=hex(can_id),
                    dlc=dlc, data=data.hex(), flags=flags)
        records.append(item)
        if can_id & 0x20000000:
            counts['error_reports'] += 1
            if can_id & 0x20:
                counts['hardware_ack_reports'] += 1
            if 'first_error' not in report:
                report['first_error'] = item
                event('first_error', frame=item)
        elif flags & socket.MSG_DONTROUTE:
            counts['local_echo'] += 1
        else:
            counts['external_data'] += 1
            if can_id == 0x102 and dlc == 8 and data[2:4] == b'\x09\x00':
                counts['heartbeat_application_ack'] += 1
    report['counts'][stage] = dict(counts)
    event('capture_end', stage=stage, counts=dict(counts))


def main():
    if os.geteuid() != 0:
        raise RuntimeError('Requires local sudo for CAN mode switching')
    ROOT.mkdir(exist_ok=True)
    report['before'] = snapshot()
    original = report['before']
    info = original['linkinfo']
    if (info['info_kind'] != 'can' or 'UP' not in original['flags'] or
            info['info_data']['bittiming']['bitrate'] != 500000 or
            info['info_data'].get('ctrlmode')):
        raise RuntimeError('Expected UP, normal-mode 500 kbit/s CAN; no changes made')
    report['qdisc_before'] = json.loads(subprocess.check_output([
        'tc', '-s', '-j', 'qdisc', 'show', 'dev', DEVICE]))
    # Subscribe before changing the controller: starting the capture after UP
    # loses precisely the first error this experiment needs to identify.
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER,
                        struct.pack('=I', 0x1fffffff))
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 1)
        sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
        sock.bind((DEVICE,))
        sock.setblocking(False)
        capture(sock, 'original_receive_only', 2)
        try:
            ip('down')
            ip('type', 'can', 'one-shot', 'on')
            # Drain old notifications while the controller is down, retaining
            # their timestamps and stage instead of confusing them with startup.
            capture(sock, 'down_drain', 0.2)
            report.pop('first_error', None)
            ip('up')
            report['single_shot_link'] = snapshot()
            capture(sock, 'single_shot_before_tx', 5)
            event('heartbeat_send_begin', can_id='0x100', dlc=8,
                  data='09ff000000000000')
            try:
                length = sock.send(FRAME.pack(0x100, 8, bytes.fromhex('09ff000000000000')))
                event('heartbeat_send_end', accepted_bytes=length)
            except OSError as exc:
                event('heartbeat_send_failed', error=str(exc))
            capture(sock, 'single_shot_after_tx', 5)
            report['single_shot_after'] = snapshot()
        finally:
            ip('down')
            ip('type', 'can', 'one-shot', 'off')
            ip('up')
            capture(sock, 'restored_receive_only', 2)
            report['restored'] = snapshot()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        event('failed', error=str(exc))
        raise
    finally:
        ROOT.mkdir(exist_ok=True)
        (ROOT / 'first_error.json').write_text(json.dumps(report, indent=2) + '\n')
        (ROOT / 'first_error.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in records))
        print('Evidence:', ROOT / 'first_error.json', flush=True)
