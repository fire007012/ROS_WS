import collections
import datetime
import json
import pathlib
import select
import socket
import struct
import subprocess
import sys
import time

device = sys.argv[1] if len(sys.argv) > 1 else 'can0'
label = sys.argv[2] if len(sys.argv) > 2 else 'baseline'
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0
probe = '--probe' in sys.argv
root = pathlib.Path('/home/fire/ros_ws/src/docs/can_ack_evidence_20261016')
root.mkdir(exist_ok=True)
frame = struct.Struct('=IB3x8s')
def snapshot():
    return json.loads(subprocess.check_output([
        'ip', '-json', '-details', '-statistics', 'link', 'show', 'dev', device]))[0]

report = {'started': datetime.datetime.now().astimezone().isoformat(),
          'device': device, 'probe': probe, 'before': snapshot()}
counts = collections.Counter()
records = []
with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack('=I', 0x1fffffff))
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 1)
    sock.bind((device,))
    sock.setblocking(False)
    start = time.monotonic()
    sent = False
    while time.monotonic() - start < duration:
        if probe and not sent and time.monotonic() - start >= 1:
            # Firmware command 0x09 changes no motor state. Never send motion.
            try:
                sock.send(frame.pack(0x100, 8, bytes.fromhex('09 ff 00 00 00 00 00 00')))
                counts['heartbeat_write_accepted'] += 1
            except OSError as error:
                counts['heartbeat_write_failed'] += 1
                report['send_error'] = str(error)
            sent = True
        if not select.select([sock], [], [], 0.05)[0]:
            continue
        raw, _, flags, _ = sock.recvmsg(frame.size)
        if len(raw) != frame.size:
            continue
        cid, dlc, data = frame.unpack(raw)
        records.append({'time': time.time(), 'can_id': hex(cid), 'dlc': dlc,
                        'data': data.hex(), 'flags': flags})
        if cid & 0x20000000:
            counts['error_frames'] += 1
            if cid & 0x20: counts['hardware_ack_errors'] += 1
            if cid & 0x40: counts['bus_off_errors'] += 1
            if cid & 0x8: counts['protocol_errors'] += 1
            if cid & 0x4 and data[1] & 0x30: counts['passive_reports'] += 1
        elif flags & socket.MSG_DONTROUTE:
            counts['local_echo'] += 1
            if flags & socket.MSG_CONFIRM: counts['tx_confirmations'] += 1
        else:
            counts['external_frames'] += 1
            counts['rx_' + hex(cid)] += 1
            if cid == 0x102 and dlc == 8 and data[2] == 9:
                counts['heartbeat_ack_ok' if data[3] == 0 else 'heartbeat_ack_rejected'] += 1
report['counts'] = dict(counts)
report['after'] = snapshot()
report['elapsed_seconds'] = time.monotonic() - start
(root / (label + '.json')).write_text(json.dumps(report, indent=2) + '\n')
(root / (label + '.jsonl')).write_text(''.join(json.dumps(r) + '\n' for r in records))
print(json.dumps(report, indent=2))
