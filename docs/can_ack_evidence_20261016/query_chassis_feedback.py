"""Bounded non-motion check: heartbeats and Y42 speed/status reads only."""
import collections
import datetime
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import time


def snapshot():
    return json.loads(subprocess.check_output([
        'ip', '-json', '-details', '-statistics', 'link', 'show', 'dev', 'can0']))[0]


def main():
    started = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
    root = Path(__file__).resolve().parent
    stem = 'power_retry_feedback_' + started.strftime('%Y%m%d_%H%M%S_%f')
    before = snapshot()
    info = before['linkinfo']['info_data']
    if ('UP' not in before['flags'] or info['state'] != 'ERROR-ACTIVE'
            or info['bittiming']['bitrate'] != 500000):
        raise SystemExit('Link not ready; no queries sent')
    frame = struct.Struct('=IB3x8s')
    counts = collections.Counter()
    readings = {}
    start = time.monotonic()
    next_heartbeat = next_read = start
    slot = 0
    with (root / (stem + '.jsonl')).open('w') as log, socket.socket(
            socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER,
                        struct.pack('=I', 0x1fffffff))
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 1)
        sock.bind(('can0',))
        sock.setblocking(False)

        def send(cid, payload, label):
            sock.send(frame.pack(cid, len(payload), payload))
            counts[label] += 1
            log.write(json.dumps({'time': time.time(), 'submitted': hex(cid),
                                  'data': payload.hex()}) + '\n')

        while time.monotonic() - start < 6:
            now = time.monotonic()
            if now >= next_heartbeat:
                send(0x100, bytes.fromhex('09ff000000000000'), 'heartbeat_submitted')
                next_heartbeat = now + 0.1
            if now >= next_read:
                address = (slot // 2) % 4 + 1
                function = 0x35 if slot % 2 == 0 else 0x3a
                send(0x80000000 | address << 8, bytes([function, 0x6b]),
                     'query_%d_%02x_submitted' % (address, function))
                slot += 1
                next_read = now + 0.05
            if not select.select([sock], [], [], 0.01)[0]:
                continue
            raw, _, flags, _ = sock.recvmsg(frame.size)
            if len(raw) != frame.size:
                continue
            cid, dlc, data = frame.unpack(raw)
            data = data[:min(dlc, 8)]
            log.write(json.dumps({'time': time.time(), 'can_id': hex(cid),
                                  'dlc': dlc, 'data': data.hex(), 'flags': flags}) + '\n')
            if cid & 0x20000000:
                counts['error_frames'] += 1
                break
            if flags & socket.MSG_DONTROUTE:
                counts['local_echo'] += 1
                continue
            counts['external_' + hex(cid)] += 1
            if cid == 0x102 and dlc == 8 and data[2] == 9:
                counts['heartbeat_ack_ok' if data[3] == 0 else 'heartbeat_ack_rejected'] += 1
            if cid & 0x80000000 and not cid & 0x40000000:
                address = (cid & 0x1fffffff) >> 8
                if cid & 0xff or address not in (1, 2, 3, 4):
                    continue
                item = readings.setdefault(str(address), {})
                if dlc == 5 and data[0] == 0x35 and data[1] <= 1 and data[4] == 0x6b:
                    item['rpm'] = int.from_bytes(data[2:4], 'big') / 10 * (-1 if data[1] else 1)
                    item['speed_received_at'] = time.time()
                    counts['speed_reply_%d' % address] += 1
                if dlc == 3 and data[0] == 0x3a and data[2] == 0x6b and data[1] not in (0xe2, 0xee):
                    item['flags'] = data[1]
                    item['status_received_at'] = time.time()
                    counts['status_reply_%d' % address] += 1
    report = {'started': started.isoformat(), 'before': before, 'after': snapshot(),
              'counts': dict(counts), 'driver_readings': readings,
              'motion_commands_sent': 0, 'elapsed_seconds': time.monotonic() - start,
              'raw_log': stem + '.jsonl'}
    (root / (stem + '.json')).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'report': str(root / (stem + '.json')),
                      'counts': dict(counts), 'driver_readings': readings,
                      'final_state': report['after']['linkinfo']['info_data']['state']}, indent=2))


if __name__ == '__main__':
    main()
