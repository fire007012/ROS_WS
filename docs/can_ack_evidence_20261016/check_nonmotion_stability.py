"""Bounded STM32 heartbeat/read test; sends no motor-state-changing command."""
import argparse
import collections
import datetime
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parent
FRAME = struct.Struct('=IB3x8s')


def snapshot():
    return json.loads(subprocess.check_output([
        'ip', '-json', '-details', '-statistics', 'link', 'show', 'dev', 'can0']))[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 180:
        parser.error('seconds must be within 1..180')
    started = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
    stem = 'nonmotion_stability_' + started.strftime('%Y%m%d_%H%M%S')
    before = snapshot()
    if before['linkinfo']['info_data']['state'] != 'ERROR-ACTIVE' or 'UP' not in before['flags']:
        raise SystemExit('CAN link not ready')
    counts = collections.Counter()
    stats = {}
    first_stats = {}
    readings = {}
    fault = ''
    start = time.monotonic()
    next_heartbeat = next_read = next_progress = last_tx = 0
    slot = 0
    with (ROOT / (stem + '.jsonl')).open('w') as log, socket.socket(
            socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack('=I', 0x1fffffff))
        sock.bind(('can0',))
        sock.setblocking(False)
        while time.monotonic() - start < args.seconds:
            now = time.monotonic()
            if now - last_tx >= 0.004:
                cid = None
                if now >= next_heartbeat:
                    cid, payload = 0x100, bytes.fromhex('09ff000000000000')
                    next_heartbeat = now + 0.1
                    counts['heartbeat_submitted'] += 1
                elif now >= next_read:
                    addr = (slot // 2) % 4 + 1
                    func = 0x35 if slot % 2 == 0 else 0x3a
                    cid, payload = 0x80000000 | addr << 8, bytes([func, 0x6b])
                    slot += 1
                    next_read = now + 0.05
                    counts['query_submitted'] += 1
                if cid is not None:
                    sock.send(FRAME.pack(cid, len(payload), payload))
                    log.write(json.dumps({'time': time.time(), 'submitted': hex(cid), 'data': payload.hex()})+'\n')
                    last_tx = now
            if select.select([sock], [], [], 0.002)[0]:
                raw, _, flags, _ = sock.recvmsg(FRAME.size)
                if len(raw) != FRAME.size:
                    continue
                cid, dlc, data = FRAME.unpack(raw)
                data = data[:min(dlc, 8)]
                log.write(json.dumps({'time': time.time(), 'can_id': hex(cid), 'dlc': dlc,
                                      'data': data.hex(), 'flags': flags})+'\n')
                if cid & 0x20000000:
                    if (cid & 0x1fffffff) == 4 and dlc == 8 and data[1] in (0, 0x40):
                        counts['active_notifications'] += 1
                    else:
                        counts['can_fault_reports'] += 1
                        fault = 'CAN error ' + hex(cid) + ' ' + data.hex()
                elif not flags & socket.MSG_DONTROUTE:
                    counts['external'] += 1
                    if cid == 0x102 and dlc == 8 and data[2] == 9:
                        counts['heartbeat_ack_ok' if data[3] == 0 else 'heartbeat_ack_rejected'] += 1
                    if cid == 0x103 and dlc == 8:
                        key = data[0]
                        first_stats.setdefault(str(key), data.hex())
                        previous = stats.get(str(key))
                        indices = (3, 4) if key == 1 else ((3, 4, 7) if key == 2 else tuple(range(1, 7)))
                        if previous and any(data[i] != bytes.fromhex(previous)[i] for i in indices):
                            fault = 'STM32 fault counter changed: ' + data.hex()
                        stats[str(key)] = data.hex()
                    if cid & 0x80000000 and not cid & 0x40000000:
                        addr = (cid & 0x1fffffff) >> 8
                        if addr in (1, 2, 3, 4) and data:
                            if dlc == 5 and data[0] == 0x35 and data[1] <= 1 and data[-1] == 0x6b:
                                readings.setdefault(str(addr), {})['rpm'] = int.from_bytes(data[2:4], 'big')/10*(-1 if data[1] else 1)
                                counts['speed_%d' % addr] += 1
                            if dlc == 3 and data[0] == 0x3a and data[-1] == 0x6b:
                                readings.setdefault(str(addr), {})['flags'] = data[1]
                                counts['status_%d' % addr] += 1
            if fault:
                break
            if now >= next_progress:
                print(json.dumps({'elapsed': now-start, 'counts': dict(counts)}), flush=True)
                next_progress = now + 15
    report = {'started': started.isoformat(), 'elapsed': time.monotonic()-start,
              'requested_seconds': args.seconds, 'fault': fault, 'counts': dict(counts),
              'before': before, 'after': snapshot(), 'first_stats': first_stats,
              'last_stats': stats, 'readings': readings, 'motion_commands_sent': 0}
    path = ROOT / (stem + '.json')
    path.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'report': str(path), **report}, indent=2), flush=True)
    return bool(fault)


if __name__ == '__main__':
    raise SystemExit(main())
