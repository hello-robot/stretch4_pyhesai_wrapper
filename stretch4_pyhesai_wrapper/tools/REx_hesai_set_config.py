#!/usr/bin/env python3
"""
REx_hesai_set_config.py

Interactive tool to configure Hesai LiDAR settings with warnings and verification.
"""

from __future__ import annotations
import argparse
import sys

from stretch4_pyhesai_wrapper.ptc_client import (
    LEFT_LIDAR_IP,
    RIGHT_LIDAR_IP,
    ptc_reachable,
    get_return_mode,
    set_return_mode,
    get_spin_rate,
    set_spin_speed,
    get_ptp_lock_offset_us,
    set_ptp_lock_offset_us,
    get_point_cloud_config,
    set_filter_type,
    get_point_cloud_mode,
    set_point_cloud_mode,
    RETURN_MODE_NAMES,
    FILTER_NAMES,
    ULTRA_PRECISE_NAMES,
    POINT_CLOUD_MODE_NAMES,
    SPIN_RATE_RPM_600,
    SPIN_RATE_RPM_1200,
    PTP_LOCK_OFFSET_MIN_US,
    PTP_LOCK_OFFSET_MAX_US,
    PTP_LOCK_OFFSET_US,
    HesaiPtcError,
)

def _ok(msg: str) -> None:
    print(msg)

def _fail(msg: str) -> None:
    print(msg)

def test_set_return_mode(ip: str, timeout: float) -> None:
    try:
        before = get_return_mode(ip, timeout=timeout)
        _ok('GET return_mode: {} ({})'.format(before, RETURN_MODE_NAMES.get(before, 'unknown')))
    except HesaiPtcError as exc:
        _fail('GET return_mode: FAILED ({})'.format(exc))
        return

    line = input('SET return mode [0-5]: ').strip()
    try:
        mode = int(line)
    except ValueError:
        _fail('Invalid input.')
        return

    try:
        set_return_mode(ip, mode, timeout=timeout)
        after = get_return_mode(ip, timeout=timeout)
        match = 'MATCH' if after == mode else 'MISMATCH'
        _ok(
            'SET return_mode: OK\n'
            'GET readback:    {} ({}) {}'.format(
                after, RETURN_MODE_NAMES.get(after, 'unknown'), match
            )
        )
    except HesaiPtcError as exc:
        _fail('SET return_mode: FAILED ({})'.format(exc))

def test_set_spin_speed(ip: str, timeout: float) -> None:
    try:
        before = get_spin_rate(ip, timeout=timeout)
        _ok('GET spin_rate: {} RPM'.format(before))
    except HesaiPtcError as exc:
        _fail('GET spin_rate: FAILED ({})'.format(exc))
        return

    line = input(
        'SET spin speed RPM [{} or {}]: '.format(
            SPIN_RATE_RPM_600, SPIN_RATE_RPM_1200
        )
    ).strip()
    if not line:
        _fail('No input.')
        return
    try:
        rpm = int(line)
    except ValueError:
        _fail('Invalid input.')
        return

    try:
        set_spin_speed(ip, rpm, timeout=timeout)
        after = get_spin_rate(ip, timeout=timeout)
        match = 'MATCH' if after == rpm else 'MISMATCH'
        _ok('SET spin_speed: OK\nGET readback:   {} RPM {}'.format(after, match))
    except HesaiPtcError as exc:
        _fail('SET spin_speed: FAILED ({})'.format(exc))

def test_set_ptp_lock_offset(ip: str, timeout: float) -> None:
    try:
        before = get_ptp_lock_offset_us(ip, timeout=timeout)
        _ok('GET ptp_lock_offset: {} us'.format(before))
    except HesaiPtcError as exc:
        _fail('GET ptp_lock_offset: FAILED ({})'.format(exc))
        return

    line = input(
        'SET PTP lock offset us [{}-{}, default {}]: '.format(
            PTP_LOCK_OFFSET_MIN_US, PTP_LOCK_OFFSET_MAX_US, PTP_LOCK_OFFSET_US
        )
    ).strip()
    offset = PTP_LOCK_OFFSET_US if not line else int(line)

    try:
        written = set_ptp_lock_offset_us(ip, offset, timeout=timeout)
        after = get_ptp_lock_offset_us(ip, timeout=timeout)
        match = 'MATCH' if after == written else 'MISMATCH'
        _ok(
            'SET ptp_lock_offset: OK (wrote {} us)\n'
            'GET readback:        {} us {}'.format(written, after, match)
        )
    except (HesaiPtcError, ValueError) as exc:
        _fail('SET ptp_lock_offset: FAILED ({})'.format(exc))

def test_set_filter_type(ip: str, timeout: float) -> None:
    try:
        ultra_before, filter_before = get_point_cloud_config(ip, timeout=timeout)
        _ok(
            'GET point_cloud: ultra={} ({}) filter={} ({})'.format(
                ultra_before,
                ULTRA_PRECISE_NAMES.get(ultra_before, 'unknown'),
                filter_before,
                FILTER_NAMES.get(filter_before, 'unknown'),
            )
        )
    except HesaiPtcError as exc:
        _fail('GET point_cloud: FAILED ({})'.format(exc))
        return

    line = input('SET filter [0-3 disabled/medium/strong/strongest]: ').strip()
    try:
        filt = int(line)
    except ValueError:
        _fail('Invalid input.')
        return
    if filt not in FILTER_NAMES:
        _fail('Filter must be 0, 1, 2, or 3.')
        return

    try:
        set_filter_type(ip, filt, timeout=timeout)
        ultra_after, filter_after = get_point_cloud_config(ip, timeout=timeout)
        ultra_match = 'UNCHANGED' if ultra_after == ultra_before else 'CHANGED'
        filter_match = 'MATCH' if filter_after == filt else 'MISMATCH'
        _ok(
            'SET filter: OK\n'
            'GET readback: ultra={} ({}) {}\n'
            '              filter={} ({}) {}'.format(
                ultra_after,
                ULTRA_PRECISE_NAMES.get(ultra_after, 'unknown'),
                ultra_match,
                filter_after,
                FILTER_NAMES.get(filter_after, 'unknown'),
                filter_match,
            )
        )
    except HesaiPtcError as exc:
        _fail('SET filter: FAILED ({})'.format(exc))


def test_set_point_cloud_mode(ip: str, timeout: float) -> None:
    try:
        before = get_point_cloud_mode(ip, timeout=timeout)
        _ok(
            'GET point_cloud_mode: {} ({})'.format(
                before, POINT_CLOUD_MODE_NAMES.get(before, 'unknown')
            )
        )
    except HesaiPtcError as exc:
        _fail('GET point_cloud_mode: FAILED ({})'.format(exc))
        return

    line = input(
        'SET point_cloud_mode [0 general / 1 mapping / 2 mapping_ground]: '
    ).strip()
    try:
        mode = int(line)
    except ValueError:
        _fail('Invalid input.')
        return
    if mode not in POINT_CLOUD_MODE_NAMES:
        _fail('Mode must be 0, 1, or 2.')
        return

    try:
        set_point_cloud_mode(ip, mode, timeout=timeout)
        after = get_point_cloud_mode(ip, timeout=timeout)
        match = 'MATCH' if after == mode else 'MISMATCH'
        _ok(
            'SET point_cloud_mode: OK\n'
            'GET readback:         {} ({}) {}'.format(
                after, POINT_CLOUD_MODE_NAMES.get(after, 'unknown'), match
            )
        )
    except HesaiPtcError as exc:
        _fail('SET point_cloud_mode: FAILED ({})'.format(exc))


def _print_menu() -> None:
    print(
        '\n  SET (GET → SET → GET, wrapper verifies readback)\n'
        '   10  set_return_mode\n'
        '   11  set_spin_speed (600 or 1200 RPM)\n'
        '   12  set_ptp_lock_offset (1-1000 us)\n'
        '   13  set_filter_type (0-3; ultra_precise unchanged; 3 needs new FW)\n'
        '   14  set_point_cloud_mode (0-2; needs new FW)\n'
        '    0  quit\n'
        'Choice: ',
        end='',
        flush=True,
    )

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Configure Hesai LiDAR settings interactively with warnings'
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--left', action='store_true', help='Configure Left Lidar')
    group.add_argument('--right', action='store_true', help='Configure Right Lidar')
    parser.add_argument('ip', nargs='?', help='Custom Lidar IP')
    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='PTC timeout in seconds (default 5)',
    )
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    # Determine which Lidar IP
    if args.left:
        ip = LEFT_LIDAR_IP
        side = "LEFT"
    elif args.right:
        ip = RIGHT_LIDAR_IP
        side = "RIGHT"
    elif args.ip:
        ip = args.ip
        side = "CUSTOM"
    else:
        print("Error: Must specify --left, --right, or custom Lidar IP.", file=sys.stderr)
        return 1

    # Print the warning
    print("=" * 80)
    print(" WARNING: You are about to modify the Hesai LiDAR hardware settings.")
    print(" CHANGING THESE SETTINGS MAY BREAK THE NORMAL OPERATION OF YOUR ROBOT!")
    print(" Please do not change these settings unless you know exactly what you are doing.")
    print("=" * 80)
    
    proceed = input(f"Proceed with configuring {side} LiDAR ({ip})? [y/N]: ").strip().lower()
    if proceed != 'y':
        print("Aborting.")
        return 0

    if not ptc_reachable(ip, timeout=args.timeout):
        print(f"Error: Lidar at {ip} is unreachable.", file=sys.stderr)
        return 1

    handlers = {
        10: lambda: test_set_return_mode(ip, args.timeout),
        11: lambda: test_set_spin_speed(ip, args.timeout),
        12: lambda: test_set_ptp_lock_offset(ip, args.timeout),
        13: lambda: test_set_filter_type(ip, args.timeout),
        14: lambda: test_set_point_cloud_mode(ip, args.timeout),
    }

    while True:
        _print_menu()
        line = input().strip()
        try:
            choice = int(line)
        except ValueError:
            print('Unknown choice.')
            continue
        if choice == 0:
            print('Bye.')
            return 0
        handler = handlers.get(choice)
        if handler is None:
            print('Unknown choice.')
            continue
        try:
            handler()
        except HesaiPtcError as exc:
            _fail('ERROR: {}'.format(exc))

if __name__ == '__main__':
    sys.exit(main())
