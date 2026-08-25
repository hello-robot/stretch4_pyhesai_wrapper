#!/usr/bin/env python3
"""
REx_hesai_upgrade_firmware.py

Upload a Hesai-provided JT128 firmware patch via PTC Upgrade Safe Image (0x83)
and print transfer progress.
"""

from __future__ import annotations

import argparse
import sys
import time

from stretch4_pyhesai_wrapper.ptc_client import (
    LEFT_LIDAR_IP,
    RIGHT_LIDAR_IP,
    PTC_PORT,
    HesaiPtcError,
    get_inventory_info,
    ptc_reachable,
    reboot_lidar,
    upgrade_lidar_firmware,
)


def _print_versions(label: str, ip: str, timeout: float) -> None:
    inv = get_inventory_info(ip, timeout=timeout)
    print(f"  [{label}]")
    print(f"  Hardware Version:     {inv['hardware_version']}")
    print(f"  Software/Firmware:    {inv['software_version']}")
    print(f"  FPGA Version:         {inv['fpga_version']}")
    if inv.get('build_id'):
        print(f"  Build/Signature ID:   {inv['build_id']}")


def _wait_for_reboot(ip: str, timeout: float, wait_s: float, poll_s: float) -> bool:
    print(f"Waiting up to {wait_s:.0f}s for lidar reboot / PTC to return...")
    deadline = time.time() + wait_s
    # Brief pause so the unit can drop the old connection.
    time.sleep(min(5.0, wait_s))
    while time.time() < deadline:
        if ptc_reachable(ip, timeout=timeout):
            return True
        time.sleep(poll_s)
    return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Upgrade JT128 firmware via PTC 0x83 with progress output'
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--left', action='store_true', help='Upgrade left lidar')
    group.add_argument('--right', action='store_true', help='Upgrade right lidar')
    parser.add_argument('ip', nargs='?', help='Custom lidar IP')
    parser.add_argument(
        '--firmware',
        required=True,
        help='Path to Hesai-provided firmware upgrade file',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=30.0,
        help='PTC connect timeout in seconds (default 30)',
    )
    parser.add_argument(
        '--reboot-wait',
        type=float,
        default=120.0,
        help='Seconds to wait for lidar to come back after transfer (default 120)',
    )
    parser.add_argument(
        '-y',
        '--yes',
        action='store_true',
        help='Skip confirmation prompt',
    )
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if args.left:
        ip = LEFT_LIDAR_IP
        side = 'LEFT'
    elif args.right:
        ip = RIGHT_LIDAR_IP
        side = 'RIGHT'
    elif args.ip:
        ip = args.ip
        side = 'CUSTOM'
    else:
        print('Error: must specify --left, --right, or lidar IP.', file=sys.stderr)
        return 1

    print('=' * 70)
    print(f'  Hesai JT128 firmware upgrade — {side} ({ip}:{PTC_PORT})')
    print(f'  Firmware file: {args.firmware}')
    print('=' * 70)
    print('WARNING: Do not power off the lidar during upgrade.')
    print('The lidar will reboot after a successful transfer.')
    print('=' * 70)

    if not args.yes:
        proceed = input('Proceed with upgrade? [y/N]: ').strip().lower()
        if proceed != 'y':
            print('Aborting.')
            return 0

    if not ptc_reachable(ip, timeout=args.timeout):
        print(f'Error: lidar at {ip} is unreachable.', file=sys.stderr)
        return 1

    try:
        print('\nVersions before upgrade:')
        _print_versions('before', ip, args.timeout)
    except HesaiPtcError as exc:
        print(f'Failed to read inventory before upgrade: {exc}', file=sys.stderr)
        return 1

    last_pct = [-1.0]

    def on_progress(percent: float) -> None:
        # Throttle identical redraws; always show whole-percent changes.
        rounded = round(percent, 1)
        if rounded == last_pct[0]:
            return
        last_pct[0] = rounded
        print(f'\r  Progress: {rounded:6.1f}%', end='', flush=True)

    print('\nUploading firmware...')
    try:
        upgrade_lidar_firmware(
            ip,
            args.firmware,
            progress_callback=on_progress,
            timeout=args.timeout,
        )
    except HesaiPtcError as exc:
        print(f'\nUpgrade failed: {exc}', file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f'\nUpgrade failed: {exc}', file=sys.stderr)
        return 1

    if last_pct[0] < 100.0:
        print('\r  Progress: 100.0%', end='', flush=True)
    print('\nTransfer complete. Sending reboot command...')

    try:
        reboot_lidar(ip, timeout=args.timeout)
    except HesaiPtcError as exc:
        print(f'  (reboot command reply not received, likely rebooting: {exc})')

    if not _wait_for_reboot(ip, args.timeout, args.reboot_wait, poll_s=2.0):
        print(
            'Lidar did not become reachable again within the wait window.\n'
            'Check power/network, then re-run REx_hesai_show_config.',
            file=sys.stderr,
        )
        return 2

    try:
        print('\nVersions after upgrade:')
        _print_versions('after', ip, args.timeout)
    except HesaiPtcError as exc:
        print(f'PTC is up but inventory read failed: {exc}', file=sys.stderr)
        return 2

    print('\nDone.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
