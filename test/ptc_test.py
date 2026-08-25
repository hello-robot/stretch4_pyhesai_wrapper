#!/usr/bin/env python3
"""
Interactive bench test for stretch4_pyhesai_wrapper.ptc_client.

Uses only the public wrapper API (not raw pybind). Same GET → SET → GET pattern
as HesaiLidar_SDK_2.0/tool_ptc/ptc_cli for the settings the wrapper exposes.

Usage:
    stretch_ptc_test 192.168.1.202
    stretch_ptc_test --left
    stretch_ptc_test --right
"""

from __future__ import annotations

import argparse
import sys

from stretch4_pyhesai_wrapper.ptc_client import (
    FILTER_NAMES,
    HesaiPtcError,
    LEFT_LIDAR_IP,
    POINT_CLOUD_MODE_NAMES,
    PTP_LOCK_OFFSET_MAX_US,
    PTP_LOCK_OFFSET_MIN_US,
    PTP_LOCK_OFFSET_US,
    PTP_STATUS_FREE_RUN,
    RETURN_MODE_NAMES,
    RIGHT_LIDAR_IP,
    SPIN_RATE_RPM_600,
    SPIN_RATE_RPM_1200,
    ULTRA_PRECISE_NAMES,
    get_lidar_ptp_status,
    get_point_cloud_config,
    get_point_cloud_mode,
    get_ptp_diagnostics,
    get_ptp_lock_offset_us,
    get_return_mode,
    get_spin_rate,
    is_new_firmware_supported,
    ptc_reachable,
    set_filter_type,
    set_point_cloud_mode,
    set_ptp_lock_offset_us,
    set_return_mode,
    set_spin_speed,
)


def _ok(msg: str) -> None:
    print(msg)


def _fail(msg: str) -> None:
    print(msg)


def read_all(ip: str, timeout: float) -> None:
    print('\n=== read_all ===')
    if not ptc_reachable(ip, timeout=timeout):
        _fail('ptc_reachable: FAIL')
        return
    _ok('ptc_reachable: OK')

    try:
        mode = get_return_mode(ip, timeout=timeout)
        _ok(
            'return mode:    {} ({})'.format(
                mode, RETURN_MODE_NAMES.get(mode, 'unknown')
            )
        )
    except HesaiPtcError as exc:
        _fail('return mode:    FAILED ({})'.format(exc))

    try:
        rpm = get_spin_rate(ip, timeout=timeout)
        _ok('spin rate:      {} RPM'.format(rpm))
    except HesaiPtcError as exc:
        _fail('spin rate:      FAILED ({})'.format(exc))

    try:
        offset = get_ptp_lock_offset_us(ip, timeout=timeout)
        _ok('ptp lock offset:{} us'.format(offset))
    except HesaiPtcError as exc:
        _fail('ptp lock offset:FAILED ({})'.format(exc))

    try:
        ultra, filt = get_point_cloud_config(ip, timeout=timeout)
        _ok(
            'point cloud:    ultra={} ({}) filter={} ({})'.format(
                ultra,
                ULTRA_PRECISE_NAMES.get(ultra, 'unknown'),
                filt,
                FILTER_NAMES.get(filt, 'unknown'),
            )
        )
    except HesaiPtcError as exc:
        _fail('point cloud:    FAILED ({})'.format(exc))

    try:
        supported = is_new_firmware_supported(ip, timeout=timeout)
        _ok('new FW features: {}'.format('supported' if supported else 'not supported'))
    except HesaiPtcError as exc:
        _fail('new FW features: FAILED ({})'.format(exc))

    try:
        pcm = get_point_cloud_mode(ip, timeout=timeout)
        _ok(
            'point cloud mode: {} ({})'.format(
                pcm, POINT_CLOUD_MODE_NAMES.get(pcm, 'unknown')
            )
        )
    except HesaiPtcError as exc:
        _ok('point cloud mode: unavailable ({})'.format(exc))

    try:
        status = get_lidar_ptp_status(ip, timeout=timeout)
        _ok(
            'ptp_status:     {} ({})'.format(
                status['ptp_status'], status['ptp_status_name']
            )
        )
    except HesaiPtcError as exc:
        _fail('ptp_status:     FAILED ({})'.format(exc))
        return

    if status['ptp_status'] == PTP_STATUS_FREE_RUN:
        _ok('ptp diagnostics: skipped (PTP Free run)')
        return

    try:
        diag = get_ptp_diagnostics(ip, timeout=timeout)
        _ok('ptp offset:     {:.1f} us'.format(diag['offset_us']))
    except HesaiPtcError as exc:
        _fail('ptp diagnostics: unavailable ({})'.format(exc))


def test_set_return_mode(ip: str, timeout: float) -> None:
    try:
        before = get_return_mode(ip, timeout=timeout)
        _ok(
            'GET return_mode: {} ({})'.format(
                before, RETURN_MODE_NAMES.get(before, 'unknown')
            )
        )
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


def _print_menu(ip: str) -> None:
    print(
        '\n--- stretch4_pyhesai_wrapper PTC test ({}) ---\n'
        '  GET\n'
        '    1  read_all\n'
        '    2  get_return_mode\n'
        '    3  get_spin_rate\n'
        '    4  get_ptp_lock_offset\n'
        '    5  get_point_cloud_config (ultra_precise + filter)\n'
        '    6  get_ptp_status\n'
        '    7  get_ptp_diagnostics\n'
        '    8  get_point_cloud_mode (needs new FW)\n'
        '  SET (GET → SET → GET, wrapper verifies readback)\n'
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
        description='Bench test stretch4_pyhesai_wrapper.ptc_client against a JT128 lidar',
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--left', action='store_true')
    group.add_argument('--right', action='store_true')
    parser.add_argument('ip', nargs='?', help='Lidar IP')
    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='PTC timeout in seconds (default 5)',
    )
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if args.left:
        ip = LEFT_LIDAR_IP
    elif args.right:
        ip = RIGHT_LIDAR_IP
    elif args.ip:
        ip = args.ip
    else:
        print(
            'Usage: stretch_ptc_test <ip> | --left | --right',
            file=sys.stderr,
        )
        return 1

    print('Testing stretch4_pyhesai_wrapper.ptc_client at {}:{}'.format(ip, 9347))
    if not ptc_reachable(ip, timeout=args.timeout):
        print('PTC not reachable at {}:9347'.format(ip), file=sys.stderr)
        return 1
    print('PTC reachable.')

    handlers = {
        1: lambda: read_all(ip, args.timeout),
        2: lambda: _ok('return_mode: {}'.format(get_return_mode(ip, args.timeout))),
        3: lambda: _ok('spin_rate: {} RPM'.format(get_spin_rate(ip, args.timeout))),
        4: lambda: _ok(
            'ptp_lock_offset: {} us'.format(get_ptp_lock_offset_us(ip, args.timeout))
        ),
        5: lambda: _ok('point_cloud_config: {}'.format(
            get_point_cloud_config(ip, args.timeout)
        )),
        6: lambda: _ok('ptp_status: {}'.format(get_lidar_ptp_status(ip, args.timeout))),
        7: lambda: _ok('ptp_diagnostics: {}'.format(get_ptp_diagnostics(ip, args.timeout))),
        8: lambda: _ok('point_cloud_mode: {}'.format(get_point_cloud_mode(ip, args.timeout))),
        10: lambda: test_set_return_mode(ip, args.timeout),
        11: lambda: test_set_spin_speed(ip, args.timeout),
        12: lambda: test_set_ptp_lock_offset(ip, args.timeout),
        13: lambda: test_set_filter_type(ip, args.timeout),
        14: lambda: test_set_point_cloud_mode(ip, args.timeout),
    }

    while True:
        _print_menu(ip)
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
