#!/usr/bin/env python3
"""Read-only SocketCAN checks. Never sends a CAN frame or changes the link."""
import argparse
import json
import select
import socket
import struct
import subprocess
import time

FRAME = struct.Struct('=IB3x8s')
EFF = 0x80000000
RTR = 0x40000000
ERR = 0x20000000


def link_problems(link, bitrate):
    problems = []
    info = link.get('linkinfo', {})
    data = info.get('info_data', {})
    if info.get('info_kind') != 'can':
        problems.append('not a physical SocketCAN interface (expected link kind can)')
    if 'UP' not in link.get('flags', []):
        problems.append('interface is DOWN')
    if data.get('state') != 'ERROR-ACTIVE':
        problems.append('CAN controller state: ' + str(data.get('state', 'unknown')))
    actual = data.get('bittiming', {}).get('bitrate')
    if actual != bitrate:
        problems.append('bitrate is %s, expected %s; must match STM32 CAN1' % (actual, bitrate))
    modes = data.get('ctrlmode', [])
    listen_only = (modes.get('listen-only', False) if isinstance(modes, dict) else 'LISTEN-ONLY' in modes)
    loopback = (modes.get('loopback', False) if isinstance(modes, dict) else 'LOOPBACK' in modes)
    if listen_only:
        problems.append('listen-only mode cannot send motor commands/ACKs')
    if loopback:
        problems.append('controller loopback is enabled; local echo does not prove bus delivery')
    return problems


def is_stm32_reply(can_id, dlc, msg_flags, rx_id, extended):
    # A candump-visible frame can be a local TX echo from another socket.
    # SocketCAN marks locally created frames MSG_DONTROUTE; exclude them.
    return (not msg_flags & socket.MSG_DONTROUTE and not can_id & (RTR | ERR)
            and 0 < dlc <= 8 and bool(can_id & EFF) == extended
            and can_id & (0x1FFFFFFF if extended else 0x7FF) == rx_id)


def describe_can_error(can_id, data):
    reasons = []
    if can_id & 0x20:
        reasons.append('hardware ACK missing')
    if can_id & 0x40:
        reasons.append('BUS-OFF')
    if can_id & 0x04 and len(data) >= 2:
        if data[1] & 0x30:
            reasons.append('ERROR-PASSIVE')
        if data[1] & 0x0C:
            reasons.append('ERROR-WARNING')
    if can_id & 0x08 and len(data) >= 4:
        reasons.append('CAN protocol violation (type=0x%02X location=0x%02X)' % (data[2], data[3]))
    return ', '.join(reasons) or 'CAN error class=0x%X' % (can_id & 0x1FFFFFFF)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='can0')
    parser.add_argument('--bitrate', type=int, default=500000,
                        help='STM32 CAN1 bitrate (current firmware with 8 MHz HSE: 500000)')
    parser.add_argument('--rx-id', type=lambda v: int(v, 0), default=0x101)
    parser.add_argument('--frame-format', choices=['standard', 'extended'], default='standard')
    parser.add_argument('--listen-seconds', type=float, default=5.0)
    args = parser.parse_args()
    if not 0 < args.listen_seconds <= 60 or args.bitrate <= 0:
        parser.error('listen-seconds must be within 0..60 and bitrate must be positive')
    if not 0 <= args.rx_id <= (0x1FFFFFFF if args.frame_format == 'extended' else 0x7FF):
        parser.error('rx-id exceeds frame-format range')
    try:
        result = subprocess.run(['ip', '-json', '-details', '-statistics', 'link', 'show',
                                 'dev', args.device], capture_output=True, text=True, check=True)
        links = json.loads(result.stdout)
        if not links:
            raise ValueError('interface not found')
        link = links[0]
        print(json.dumps(link, indent=2))
        problems = link_problems(link, args.bitrate)
        if problems:
            for problem in problems:
                print('FAIL:', problem)
            return 2
        print('Link configuration OK. Passively listening; no CAN commands will be sent.')
        replies = local = other = errors = state_notifications = 0
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
            # CAN_RAW excludes error frames by default. Data RX can continue
            # during ERROR-PASSIVE and must never hide missing hardware ACKs.
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER,
                            struct.pack('=I', 0x1FFFFFFF))
            sock.bind((args.device,))
            sock.setblocking(False)
            deadline = time.monotonic() + args.listen_seconds
            while time.monotonic() < deadline:
                if not select.select([sock], [], [], max(0, deadline - time.monotonic()))[0]:
                    break
                raw, _, flags, _ = sock.recvmsg(FRAME.size)
                if len(raw) != FRAME.size:
                    continue
                cid, dlc, data = FRAME.unpack(raw)
                if cid & ERR:
                    # Older candleLight firmware reports ESR changes using
                    # CAN_ERR_CRTL even when returning to error-active. This
                    # notification alone is not an ACK/protocol/controller fault.
                    if (cid & 0x1FFFFFFF) == 0x04 and dlc == 8 and data[1] in (0, 0x40):
                        state_notifications += 1
                        continue
                    errors += 1
                    if errors <= 10:
                        print('FAIL: ' + describe_can_error(cid, data[:min(dlc, 8)]))
                    continue
                if flags & socket.MSG_DONTROUTE:
                    local += 1
                elif is_stm32_reply(cid, dlc, flags, args.rx_id, args.frame_format == 'extended'):
                    replies += 1
                else:
                    other += 1
                if replies + local + other <= 20:
                    print('%s %s ID=0x%X DLC=%d %s' % (
                        'local-TX' if flags & socket.MSG_DONTROUTE else 'bus-RX',
                        'extended' if cid & EFF else 'standard', cid & 0x1FFFFFFF,
                        dlc, data[:min(dlc, 8)].hex(' ')))
        print('Expected STM32 bus RX=%d; local TX echoes=%d; other bus RX=%d; CAN error frames=%d' % (
            replies, local, other, errors))
        print('Controller error-active notifications=%d' % state_notifications)
        final_links = json.loads(subprocess.run(
            ['ip', '-json', '-details', '-statistics', 'link', 'show', 'dev', args.device],
            capture_output=True, text=True, check=True).stdout)
        if not final_links:
            raise ValueError('interface disappeared during capture')
        final_problems = link_problems(final_links[0], args.bitrate)
        if errors or final_problems:
            for problem in final_problems:
                print('FAIL:', problem)
            print('FAIL: CAN link fault observed; matching data RX does not prove a healthy link.')
            return 4
        if not replies:
            print('No matching external reply observed. If the ROS node was running, check '
                  'ID/IDE/bitrate, STM32 CAN1 filter/FIFO interrupt, transceivers, H/L/GND and termination. '
                  'A local TX echo or successful socket write is not proof of STM32 reception.')
            return 3
        print('Matching bus replies observed; this confirms reception on Linux, not motor motion.')
        return 0
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('FAIL:', error)
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stderr.strip())
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
