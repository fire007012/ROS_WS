"""First-transmission experiment after CANable power cycling; no firmware writes."""
import json
import os
import socket
import struct
import sys
import time

import can_ack_first_error as trace


def main():
    if os.geteuid() != 0:
        raise RuntimeError('Local sudo is required to bring CAN up')
    original = trace.snapshot()
    trace.report['fresh_before'] = original
    stats = original['stats64']
    if ('UP' in original['flags'] or stats['tx']['packets'] or stats['rx']['packets']):
        raise RuntimeError('Expected freshly replugged CANable: DOWN and RX/TX=0. '
                           'Unplug/replug CANable only, leave ROS stopped, then rerun.')
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER,
                        struct.pack('=I', 0x1fffffff))
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 1)
        sock.setsockopt(socket.SOL_SOCKET, trace.SO_TIMESTAMPNS, 1)
        # Keep the subscription alive before starting the CAN peripheral.
        sock.bind((trace.DEVICE,))
        sock.setblocking(False)
        trace.ip('type', 'can', 'bitrate', '500000', 'sample-point', '0.875', 'sjw', '1')
        trace.ip('up')
        trace.capture(sock, 'fresh_before_first_tx', 5)
        trace.report['before_first_tx'] = trace.snapshot()
        baseline = trace.report['counts']['fresh_before_first_tx']
        if baseline.get('error_reports', 0):
            trace.event('no_probe_needed', reason='Controller errors precede any host application TX')
        else:
            trace.event('heartbeat_send_begin', can_id='0x100', dlc=8,
                        data='09ff000000000000')
            try:
                count = sock.send(trace.FRAME.pack(0x100, 8, bytes.fromhex('09ff000000000000')))
                trace.event('heartbeat_send_end', accepted_bytes=count)
            except OSError as exc:
                trace.event('heartbeat_send_failed', error=str(exc))
            trace.capture(sock, 'fresh_after_first_tx', 5)
        trace.report['after'] = trace.snapshot()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        trace.event('failed', error=str(exc))
        raise
    finally:
        trace.ROOT.mkdir(exist_ok=True)
        (trace.ROOT / 'fresh_start.json').write_text(json.dumps(trace.report, indent=2) + '\n')
        (trace.ROOT / 'fresh_start.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in trace.records))
        print('Evidence:', trace.ROOT / 'fresh_start.json', flush=True)
