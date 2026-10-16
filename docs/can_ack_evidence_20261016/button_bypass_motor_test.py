"""Bounded chassis test using existing Y42 codecs, without the start button.

Run explicitly with --run. No driver settings or firmware are changed.
Only motor addresses 1..4 receive native commands; heartbeat stays active.
"""
import argparse
import collections
import datetime
import json
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1] / 'can_motor_interface' / 'scripts'))
from y42_direct_node import Feedback, FRAME, STOP, parse_reply, parse_stm32_stop, speed_payload


def snapshot():
    return json.loads(subprocess.check_output([
        'ip', '-json', '-details', '-statistics', 'link', 'show', 'dev', 'can0']))[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true', required=True)
    parser.add_argument('--route', choices=('native', 'standard'), default='standard')
    parser.add_argument('--pattern', choices=('suite', 'forward', 'left'), default='suite')
    parser.add_argument('--duration', type=float, default=1.0)
    args = parser.parse_args()
    if not 0.2 <= args.duration <= 3.0:
        parser.error('duration must be within 0.2..3.0 seconds')
    directional_cases = {'forward': [10, 0, -10, 0], 'left': [0, 10, 0, -10]}
    test_cases = ([(args.pattern, directional_cases[args.pattern])]
                  if args.pattern in directional_cases else [
        ('address_2_positive', [10, 0, 0, 0]),
        ('address_1_positive', [0, 10, 0, 0]),
        ('address_4_positive', [0, 0, 10, 0]),
        ('address_3_positive', [0, 0, 0, 10]),
        ('all_positive', [10, 10, 10, 10]),
        ('all_negative', [-10, -10, -10, -10])])
    started = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
    stem = 'button_bypass_motor_' + started.strftime('%Y%m%d_%H%M%S')
    before = snapshot()
    info = before['linkinfo']['info_data']
    modes = info.get('ctrlmode', [])
    if ('UP' not in before['flags'] or info['state'] != 'ERROR-ACTIVE'
            or info['bittiming']['bitrate'] != 500000
            or any(x in modes for x in ('LOOPBACK', 'LISTEN-ONLY'))):
        raise SystemExit('Link not ready; no commands sent')
    ids = [2, 1, 4, 3]
    feedback = Feedback(ids)
    counts = collections.Counter()
    stats = {}
    stats_seen = {}
    last_heartbeat = 0
    ack_seen = {}
    first_motion = {}
    stages = []
    phase = 'preflight'
    fault = ''
    interrupted = False
    target = dict.fromkeys(ids, 0)
    init_zero = False
    last_control = last_query = last_heartbeat_tx = last_tx = 0
    control_slot = query_slot = 0
    start = time.monotonic()
    log = (ROOT / (stem + '.jsonl')).open('w')
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER,
                    struct.pack('=I', 0x1fffffff))
    sock.bind(('can0',))
    sock.setblocking(False)

    def interrupt(*_):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    def record(item):
        log.write(json.dumps({'time': time.time(), 'phase': phase, **item}) + '\n')

    def send(cid, payload):
        nonlocal last_tx
        if sock.send(FRAME.pack(cid, len(payload), payload)) != FRAME.size:
            raise RuntimeError('Short CAN write')
        last_tx = time.monotonic()
        counts['submitted'] += 1
        record({'submitted': hex(cid), 'data': payload.hex()})

    def receive():
        nonlocal last_heartbeat
        for _ in range(256):
            try:
                raw, _, flags, _ = sock.recvmsg(FRAME.size)
            except BlockingIOError:
                break
            if len(raw) != FRAME.size:
                continue
            cid, dlc, data = FRAME.unpack(raw)
            data = data[:min(dlc, 8)]
            record({'can_id': hex(cid), 'data': data.hex(), 'dlc': dlc, 'flags': flags})
            now = time.monotonic()
            if cid & 0x20000000:
                if (cid & 0x1fffffff) == 4 and dlc == 8 and data[1] in (0, 0x40):
                    counts['active_notifications'] += 1
                    continue
                counts['can_errors'] += 1
                raise RuntimeError('CAN error frame ' + hex(cid) + ' ' + data.hex())
            if flags & socket.MSG_DONTROUTE:
                continue
            counts['external'] += 1
            why = parse_stm32_stop(cid, data)
            if why:
                raise RuntimeError(why)
            reply = parse_reply(cid, data)
            feedback.update(reply, now)
            if reply and reply[1] == 'speed':
                if stages and phase == stages[-1]['name']:
                    item = stages[-1]
                    address, _, rpm = reply
                    if abs(rpm) > abs(item['peaks'][str(address)]):
                        item['peaks'][str(address)] = rpm
                    if abs(rpm) > 25:
                        raise RuntimeError('Unexpected overspeed')
                    if target[address] and rpm * target[address] < -20:
                        raise RuntimeError('Motor direction mismatch')
                    if not target[address] and abs(rpm) > 1.5:
                        raise RuntimeError('Inactive motor moved')
            if cid & 0x80000000 and not cid & 0x40000000:
                address = (cid & 0x1fffffff) >> 8
                if address in ids and data and data[0] == 0xf6:
                    if data != bytes.fromhex('f6026b'):
                        raise RuntimeError('Native F6 rejected: ' + data.hex())
                    ack_seen[address] = now
                    counts['motion_ack_%d' % address] += 1
                if address in ids and len(data) >= 2 and data[1] in (0xe2, 0xee):
                    raise RuntimeError('Native motor error ' + data.hex())
            if cid == 0x102 and dlc == 8:
                if data[3] != 0:
                    raise RuntimeError('STM32 command rejected: ' + data.hex())
                if data[2] == 9:
                    last_heartbeat = now
                    counts['heartbeat_ack'] += 1
            if cid == 0x103 and dlc == 8 and data[0] in (1, 2, 3):
                kind = data[0]
                values = struct.unpack('<3H', data[1:7])
                # Match link_health.h FirmwareStats wire offsets exactly.
                # Type 1: RX count / queue drops / aggregate response timeouts.
                # Type 2: successes / execution failures / sequence.
                # Type 3: CAN1 TX / CAN2 TX / stop failures.
                indices = (1,) if kind in (1, 2) else (0, 1, 2)
                if kind == 2 and data[7] != 0:
                    raise RuntimeError('STM32 last execution failed: ' + data.hex())
                if kind in stats and any(values[i] != stats[kind][i] for i in indices):
                    raise RuntimeError('STM32 fault counter changed: type=%d %s -> %s' % (
                        kind, stats[kind], values))
                stats[kind] = values
                stats_seen[kind] = now

    def tick(strict=False, stopping=False):
        nonlocal last_heartbeat_tx, last_query, last_control, query_slot, control_slot
        try:
            receive()
        except RuntimeError as exc:
            if not stopping:
                raise
            record({'stop_observation_fault': str(exc)})
        now = time.monotonic()
        if interrupted and not stopping:
            raise RuntimeError('Test interrupted')
        if strict:
            why = feedback.problem(now, 1.0, strict_warning=not bool(first_motion))
            if why:
                raise RuntimeError(why)
            if now - last_heartbeat > 0.5:
                raise RuntimeError('Heartbeat application ACK stale')
            if any(now - stats_seen.get(k, 0) > 1 for k in (1, 2, 3)):
                raise RuntimeError('STM32 statistics stale')
            for addr, sent_at in first_motion.items():
                if now - max(sent_at, ack_seen.get(addr, 0)) > 0.5:
                    raise RuntimeError('Native F6 ACK timeout for address %d' % addr)
        if now - last_tx >= 0.004:
            if now - last_heartbeat_tx >= 0.1:
                send(0x100, bytes.fromhex('09ff000000000000'))
                last_heartbeat_tx = now
            elif now - last_control >= (0.0125 if any(target.values()) or init_zero else 0.0625):
                addr = ids[control_slot % 4]
                control_slot += 1
                payload = speed_payload(target[addr], 50, 10) if target[addr] or init_zero else STOP
                if payload[0] == 0xf6 and args.route == 'standard':
                    # Immediate standard speed command: cache/sync flag is zero.
                    send(0x100, struct.pack('<BBiBB', 1, addr - 1, target[addr], 50, 0))
                else:
                    send(0x80000000 | addr << 8, payload)
                if payload[0] == 0xf6:
                    first_motion.setdefault(addr, now)
                last_control = now
            elif now - last_query >= 0.05:
                addr = ids[(query_slot // 2) % 4]
                func = 0x35 if query_slot % 2 == 0 else 0x3a
                query_slot += 1
                send(0x80000000 | addr << 8, bytes([func, 0x6b]))
                last_query = now
        select.select([sock], [], [], 0.002)

    def hold(duration, strict=True):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            tick(strict)

    def stop_and_verify():
        nonlocal phase, init_zero
        phase = 'verify_stop'
        init_zero = False
        target.update(dict.fromkeys(ids, 0))
        first_motion.clear()
        stop_start = time.monotonic()
        stationary_since = None
        while time.monotonic() - stop_start < 3:
            tick(stopping=True)
            now = time.monotonic()
            fresh = all(feedback.seen.get((a, 'speed'), 0) > stop_start
                        and now - feedback.seen.get((a, 'speed'), 0) < 1
                        and abs(feedback.speed[a]) <= 1 for a in ids)
            stationary_since = (stationary_since or now) if fresh else None
            if stationary_since and now - stationary_since >= 0.5:
                return dict(feedback.speed)
        raise RuntimeError('Could not verify four-wheel stop')

    stopped = None
    try:
        hold(3, strict=False)
        tick(strict=True)
        if any(abs(v) > 1 for v in feedback.speed.values()):
            raise RuntimeError('Wheels not stationary before test')
        phase = 'zero_initialization'
        init_zero = True
        hold(1)
        init_zero = False
        for name, vector in test_cases:
            phase = name
            first_motion.clear()
            target.update(dict(zip(ids, vector)))
            stage = {'name': name, 'target_by_address': dict(target),
                     'peaks': {str(a): 0 for a in ids}}
            stages.append(stage)
            record({'stage_started': stage})
            hold(args.duration)
            stage['stop_rpm'] = stop_and_verify()
            stage['passed'] = all(stage['peaks'][str(a)] * rpm > 30
                                  for a, rpm in zip(ids, vector) if rpm)
            print(json.dumps(stage), flush=True)
            if not stage['passed']:
                raise RuntimeError('Target wheel did not respond')
    except Exception as exc:
        fault = str(exc)
        print('STOP: ' + fault, flush=True)
    finally:
        try:
            stopped = stop_and_verify()
        except Exception as exc:
            fault = fault or str(exc)
            # Bounded final best-effort stop; never resume motion after a fault.
            target.update(dict.fromkeys(ids, 0))
            for _ in range(2):
                for addr in ids:
                    try:
                        send(0x80000000 | addr << 8, STOP)
                    except OSError:
                        pass
                    time.sleep(0.02)
        sock.close()
        log.close()
        report = {'started': started.isoformat(), 'elapsed': time.monotonic() - start,
                  'button_bypass_requested': True, 'route': args.route,
                  'pattern': args.pattern, 'motion_duration': args.duration,
                  'motor_order': ids, 'limit_rpm': 10, 'before': before, 'after': snapshot(),
                  'stages': stages, 'fault': fault, 'final_stop_rpm': stopped,
                  'counts': dict(counts), 'last_statistics': stats, 'raw_log': stem + '.jsonl'}
        path = ROOT / (stem + '.json')
        path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({'report': str(path), 'fault': fault, 'final_stop_rpm': stopped,
                          'passed': not fault and len(stages) == len(test_cases)}, indent=2))
    return bool(fault)


if __name__ == '__main__':
    raise SystemExit(main())
