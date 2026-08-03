#!/usr/bin/env python3
"""
REx_hesai_show_info.py

A tool to print configuration, firmware, and status of Stretch's Hesai lidars.
"""

from __future__ import annotations
import argparse
import sys

from pyhesai_wrapper.ptc_client import (
    LEFT_LIDAR_IP,
    RIGHT_LIDAR_IP,
    PTC_PORT,
    ptc_reachable,
    get_inventory_info,
    get_return_mode,
    get_spin_rate,
    get_ptp_lock_offset_us,
    get_point_cloud_config,
    get_lidar_ptp_status,
    get_ptp_diagnostics,
    RETURN_MODE_NAMES,
    FILTER_NAMES,
    ULTRA_PRECISE_NAMES,
    PTP_STATUS_NAMES,
    PTP_STATUS_FREE_RUN,
    HesaiPtcError,
)

def show_lidar_info(side: str, ip: str, timeout: float = 2.0) -> None:
    print("=" * 70)
    print(f"             HESAI {side.upper()} LIDAR CONFIGURATION & STATUS")
    print(f"             Address: {ip}:{PTC_PORT}")
    print("=" * 70)

    if not ptc_reachable(ip, timeout=timeout):
        print("  Status:               OFFLINE / UNREACHABLE")
        print("=" * 70)
        print()
        return

    # 1. Fetch Inventory Info
    try:
        inv = get_inventory_info(ip, timeout=timeout)
        print("  INVENTORY INFO")
        print("  " + "-" * 66)
        print(f"  Model:                {inv['model']}")
        print(f"  Serial Number:        {inv['serial_number']}")
        print(f"  MAC Address:          {inv['mac_address']}")
        print(f"  Calibration/Mfg Date: {inv['calibration_date']}")
        print(f"  Hardware Version:     {inv['hardware_version']}")
        print(f"  Software/Firmware:    {inv['software_version']}")
        print(f"  FPGA Version:         {inv['fpga_version']}")
        if inv['build_id']:
            print(f"  Build/Signature ID:   {inv['build_id']}")
    except Exception as e:
        print(f"  INVENTORY INFO:       FAILED to query ({e})")

    print("  " + "-" * 66)
    print("  CURRENT SETTINGS & STATUS")
    print("  " + "-" * 66)

    # 2. Return Mode
    try:
        mode = get_return_mode(ip, timeout=timeout)
        mode_name = RETURN_MODE_NAMES.get(mode, 'unknown')
        print(f"  Return Mode:          {mode} ({mode_name})")
    except Exception as e:
        print(f"  Return Mode:          FAILED ({e})")

    # 3. Spin Rate
    try:
        rpm = get_spin_rate(ip, timeout=timeout)
        print(f"  Spin Rate:            {rpm} RPM")
    except Exception as e:
        print(f"  Spin Rate:            FAILED ({e})")

    # 4. PTP Lock Offset
    try:
        offset = get_ptp_lock_offset_us(ip, timeout=timeout)
        print(f"  PTP Lock Offset:      {offset} us")
    except Exception as e:
        print(f"  PTP Lock Offset:      FAILED ({e})")

    # 5. Point Cloud Config
    try:
        ultra, filt = get_point_cloud_config(ip, timeout=timeout)
        ultra_name = ULTRA_PRECISE_NAMES.get(ultra, 'unknown')
        filter_name = FILTER_NAMES.get(filt, 'unknown')
        print(f"  Ultra Precise Mode:   {ultra} ({ultra_name})")
        print(f"  Noise Filter Type:    {filt} ({filter_name})")
    except Exception as e:
        print(f"  Point Cloud Config:   FAILED ({e})")

    # 6. PTP Status
    ptp_status_val = None
    try:
        ptp = get_lidar_ptp_status(ip, timeout=timeout)
        ptp_status_val = ptp['ptp_status']
        print(f"  PTP Status:           {ptp_status_val} ({ptp['ptp_status_name']})")
    except Exception as e:
        print(f"  PTP Status:           FAILED ({e})")

    # 7. PTP Diagnostics (only if PTP enabled / not free run)
    if ptp_status_val is not None and ptp_status_val != PTP_STATUS_FREE_RUN:
        try:
            diag = get_ptp_diagnostics(ip, timeout=timeout)
            print(f"  PTP Master Offset:    {diag['offset_us']:.1f} us ({diag['offset_ns']} ns)")
        except Exception as e:
            print(f"  PTP Master Offset:    UNAVAILABLE ({e})")

    print("=" * 70)
    print()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Print Hesai LiDAR configuration, firmware version, and status"
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--left", action="store_true", help="Show left LiDAR information only")
    group.add_argument("--right", action="store_true", help="Show right LiDAR information only")
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="PTC query timeout in seconds (default 2.0)",
    )
    
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    # Determine which lidars to show
    if args.left:
        show_lidar_info("left", LEFT_LIDAR_IP, timeout=args.timeout)
    elif args.right:
        show_lidar_info("right", RIGHT_LIDAR_IP, timeout=args.timeout)
    else:
        # Show both by default
        show_lidar_info("left", LEFT_LIDAR_IP, timeout=args.timeout)
        show_lidar_info("right", RIGHT_LIDAR_IP, timeout=args.timeout)

    return 0


if __name__ == "__main__":
    sys.exit(main())
