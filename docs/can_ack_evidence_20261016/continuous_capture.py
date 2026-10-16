"""Receive-only CAN capture across USB reconnects; stop with SIGTERM/Ctrl-C."""
import collections
import datetime
import errno
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import time


ROOT = Path(__file__).resolve().parent
FRAME = struct.Struct('=IB3x8s')
TZ = datetime.timezone(datetime.timedelta(hours=8))


def main():
    stamp = datetime.datetime.now(TZ).strftime('%Y%m%d_%H%M%S_%f')
    base = ROOT / ('continuous_' + stamp)
    counts = collections.Counter()
    stop = False

    def finish(signum, unused):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, finish)
    signal.signal(signal.SIGINT, finish)
    start = time.time()
    last_state = None
    interfaces = []
    next_snapshot = next_status = 0
    with base.with_suffix('.jsonl').open('x', buffering=1) as log:
        def record(kind, **fields):
            log.write(json.dumps(dict(time=time.time(), kind=kind, **fields)) + '\n')

        def status(running):
            report = dict(pid=os.getpid(), running=running, started=start,
                          updated=time.time(), receive_only=True,
                          counts=dict(counts), interfaces=interfaces,
                          frames_file=str(base.with_suffix('.jsonl')))
            temporary = base.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(report, indent=2) + '\n')
            temporary.replace(base.with_suffix('.json'))

        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER,
                            struct.pack('=I', 0x1fffffff))
            timestamp_option = getattr(socket, 'SO_TIMESTAMPNS', 35)
            sock.setsockopt(socket.SOL_SOCKET, timestamp_option, 1)
            # ifindex=0 receives from all CAN interfaces, including new USB
            # instances. No rebind gap when can0 disappears or changes index.
            sock.bind(('',))
            sock.setblocking(False)
            record('start', pid=os.getpid(), receive_only=True)
            status(True)
            print('READY pid=%d log=%s' % (os.getpid(), base.with_suffix('.jsonl')),
                  flush=True)
            try:
                while not stop:
                    now = time.monotonic()
                    if now >= next_snapshot:
                        try:
                            proc = subprocess.run(
                                ['ip', '-j', '-d', '-s', 'link', 'show', 'type', 'can'],
                                capture_output=True, text=True, timeout=2, check=True)
                            interfaces = json.loads(proc.stdout)
                            state = [(i['ifindex'], i['ifname'], i.get('flags'),
                                      i.get('linkinfo', {}).get('info_data', {}))
                                     for i in interfaces]
                            if state != last_state:
                                record('link_change', interfaces=interfaces)
                                print('LINK ' + json.dumps(state), flush=True)
                                last_state = state
                            else:
                                record('link_snapshot', interfaces=interfaces)
                        except (OSError, ValueError, subprocess.SubprocessError) as exc:
                            record('snapshot_error', error=str(exc))
                        next_snapshot = time.monotonic() + 0.5
                    if now >= next_status:
                        status(True)
                        print('COUNTS ' + json.dumps(dict(counts)), flush=True)
                        next_status = time.monotonic() + 5
                    if not select.select([sock], [], [], 0.1)[0]:
                        continue
                    try:
                        raw, ancillary, flags, address = sock.recvmsg(FRAME.size, 128)
                    except OSError as exc:
                        if exc.errno in (errno.ENETDOWN, errno.ENODEV, errno.EAGAIN,
                                         errno.EINTR, errno.ENOBUFS):
                            counts['socket_notifications'] += 1
                            record('socket_notification', errno=exc.errno)
                            time.sleep(0.05)
                            continue
                        raise
                    if len(raw) != FRAME.size:
                        counts['truncated_frames'] += 1
                        record('truncated_frame', raw=raw.hex(), flags=flags)
                        continue
                    cid, dlc, data = FRAME.unpack(raw)
                    kernel_time = None
                    for level, kind, payload in ancillary:
                        if level == socket.SOL_SOCKET and kind == timestamp_option:
                            sec, nsec = struct.unpack('@ll', payload[:struct.calcsize('@ll')])
                            kernel_time = sec + nsec / 1e9
                    record('frame', kernel_time=kernel_time, interface=address[0],
                           can_id=hex(cid), dlc=dlc, data=data.hex(), flags=flags,
                           truncated=bool(flags & socket.MSG_TRUNC))
                    if cid & 0x20000000:
                        counts['error_reports'] += 1
                        if cid & 0x20:
                            counts['ack_reports'] += 1
                        if cid & 0x08:
                            counts['protocol_reports'] += 1
                        if cid & 0x40:
                            counts['bus_off_reports'] += 1
                        if cid & 0x04 and dlc >= 2 and data[1] & 0x30:
                            counts['passive_reports'] += 1
                    elif flags & socket.MSG_DONTROUTE:
                        counts['local_tx_echoes'] += 1
                    else:
                        counts['external_frames'] += 1
                        counts['external_' + hex(cid)] += 1
            finally:
                record('stop', counts=dict(counts))
                status(False)
                print('STOPPED ' + str(base.with_suffix('.json')), flush=True)


if __name__ == '__main__':
    main()
