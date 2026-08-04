#!/usr/bin/env python3
"""
JT128 Hesai PTC (TCP) client backed by the Hesai SDK PtcClient via pybind11.

Provides module-level get/set helpers used by stretch_lidar_check and FAB tests.
"""

import argparse
import os
import struct
import sys
import yaml

from pyhesai_wrapper import pyhesai_wrapper_cpp as _cpp

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")
with open(CONFIG_PATH, "r") as f:
    _config_str = os.path.expandvars(f.read())
    CONFIG = yaml.safe_load(_config_str)

LEFT_LIDAR_IP = CONFIG['left_lidar']['ip']
RIGHT_LIDAR_IP = CONFIG['right_lidar']['ip']
PTC_PORT = CONFIG['left_lidar'].get('ptc_port', 9347)

PTP_DIAGNOSTICS_SUBCOMMAND = 1
PTP_DIAGNOSTICS_PAYLOAD_LEN = 24



RETURN_MODE_LAST_AND_STRONGEST = 2
PTP_LOCK_OFFSET_US = 350
PTP_LOCK_OFFSET_MIN_US = 1
PTP_LOCK_OFFSET_MAX_US = 1000

RETURN_MODE_NAMES = {
    0: 'last',
    1: 'strongest',
    2: 'last_and_strongest',
    3: 'first',
    4: 'last_and_first',
    5: 'first_and_strongest',
}

PTP_STATUS_FREE_RUN = 0
PTP_STATUS_TRACKING = 1
PTP_STATUS_LOCKED = 2
PTP_STATUS_FROZEN = 3

PTP_STATUS_NAMES = {
    PTP_STATUS_FREE_RUN: 'free_run',
    PTP_STATUS_TRACKING: 'tracking',
    PTP_STATUS_LOCKED: 'locked',
    PTP_STATUS_FROZEN: 'frozen',
}

ACCEPTABLE_PTP_STATUSES = {PTP_STATUS_TRACKING, PTP_STATUS_LOCKED}
FAIL_PTP_STATUSES = {PTP_STATUS_FREE_RUN, PTP_STATUS_FROZEN}

POINT_CLOUD_KEEP_CURRENT = 0xFF

FILTER_DISABLED = 0
FILTER_MEDIUM = 1
FILTER_STRONG = 2

FILTER_NAMES = {
    FILTER_DISABLED: 'disabled',
    FILTER_MEDIUM: 'medium',
    FILTER_STRONG: 'strong',
}

ULTRA_PRECISE_NAMES = {
    0: 'low',
    1: 'medium',
    2: 'strong',
    3: 'off',
}


class HesaiPtcError(Exception):
    """Raised when a PTC command fails."""


def _wrap_cpp_error(exc):
    """Convert C++ runtime_error from pybind into HesaiPtcError."""
    return HesaiPtcError(str(exc))


def _session(ip, timeout, ptc_port):
    """Open a PTC session or raise HesaiPtcError."""
    client = _cpp.PtcClient(ip, ptc_port)
    try:
        client.wait_until_open(timeout)
    except RuntimeError as exc:
        raise _wrap_cpp_error(exc) from exc
    return client


def _run(fn, ip, timeout=2.0, ptc_port=PTC_PORT):
    """Run a callable with a connected PTC session."""
    try:
        client = _session(ip, timeout, ptc_port)
        return fn(client)
    except RuntimeError as exc:
        raise _wrap_cpp_error(exc) from exc


def ptc_reachable(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Return True if PTC TCP port accepts a connection."""
    return _cpp.ptc_reachable(ip, ptc_port, timeout)


def get_return_mode(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Read lidar return mode via SDK GetReturnMode."""
    return _run(lambda c: c.get_return_mode(), ip, timeout, ptc_port)


def set_return_mode(ip, mode, timeout=2.0, ptc_port=PTC_PORT):
    """Set return mode and verify readback."""
    def _set(client):
        client.set_return_mode(int(mode))
        readback = client.get_return_mode()
        if readback != int(mode):
            raise HesaiPtcError(
                'SetReturnMode readback mismatch: expected {}, got {}'.format(
                    mode, readback
                )
            )

    _run(_set, ip, timeout, ptc_port)


def get_ptp_lock_offset_us(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Read PTP lock offset threshold in microseconds."""
    return _run(lambda c: c.get_ptp_lock_offset_us(), ip, timeout, ptc_port)


def set_ptp_lock_offset_us(ip, offset_us=PTP_LOCK_OFFSET_US, timeout=2.0, ptc_port=PTC_PORT):
    """
    Set PTP lock offset threshold in microseconds (clamped to 1-1000 µs).

    Returns the clamped value written. Raises HesaiPtcError on readback mismatch.
    """
    us = max(
        PTP_LOCK_OFFSET_MIN_US,
        min(PTP_LOCK_OFFSET_MAX_US, int(offset_us)),
    )

    def _set(client):
        client.set_ptp_lock_offset_us(us)
        readback = client.get_ptp_lock_offset_us()
        if readback != us:
            raise HesaiPtcError(
                'SetPTPLockOffset readback mismatch: expected {} µs, got {} µs'.format(
                    us, readback
                )
            )
        return us

    return _run(_set, ip, timeout, ptc_port)


def get_lidar_ptp_status(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Read PTP status (0=free_run, 1=tracking, 2=locked, 3=frozen)."""
    status = _run(lambda c: c.get_lidar_ptp_status(), ip, timeout, ptc_port)
    return {
        'ptp_status': status,
        'ptp_status_name': PTP_STATUS_NAMES.get(status, 'unknown'),
    }


def get_ptp_diagnostics(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Query PTP master offset and lidar PTP status."""
    def _get(client):
        payload = client.get_ptp_diagnostics_raw(PTP_DIAGNOSTICS_SUBCOMMAND)
        if len(payload) != PTP_DIAGNOSTICS_PAYLOAD_LEN:
            raise HesaiPtcError(
                'GET_PTP_DIAGNOSTICS expected {} bytes, got {}'.format(
                    PTP_DIAGNOSTICS_PAYLOAD_LEN, len(payload)
                )
            )
        offset_ns = struct.unpack('>q', payload[:8])[0]
        status = client.get_lidar_ptp_status()
        return {
            'offset_ns': offset_ns,
            'offset_us': abs(offset_ns / 1000.0),
            'ptp_status': status,
            'ptp_status_name': PTP_STATUS_NAMES.get(status, 'unknown'),
        }

    return _run(_get, ip, timeout, ptc_port)


def sample_ptp_offset_us(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Sample one |PTP offset| reading in microseconds."""
    diag = get_ptp_diagnostics(ip, timeout=timeout, ptc_port=ptc_port)
    return diag['offset_us'], diag


SPIN_RATE_RPM_600 = 600
SPIN_RATE_RPM_1200 = 1200


def get_spin_rate(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Read lidar spin rate in RPM."""
    return _run(lambda c: c.get_spin_rate(), ip, timeout, ptc_port)


def set_spin_speed(ip, rpm, timeout=2.0, ptc_port=PTC_PORT):
    """Set spin speed (600 or 1200 RPM) and verify readback."""
    rpm = int(rpm)
    if rpm not in (SPIN_RATE_RPM_600, SPIN_RATE_RPM_1200):
        raise HesaiPtcError(
            'Spin rate must be {} or {} RPM'.format(
                SPIN_RATE_RPM_600, SPIN_RATE_RPM_1200
            )
        )

    def _set(client):
        client.set_spin_speed(rpm)
        readback = client.get_spin_rate()
        if readback != rpm:
            raise HesaiPtcError(
                'SetSpinSpeed readback mismatch: expected {} RPM, got {} RPM'.format(
                    rpm, readback
                )
            )

    _run(_set, ip, timeout, ptc_port)


def get_point_cloud_config(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Read JT128 point cloud config (ultra_precise, filter) via GET 0x122."""
    return _run(lambda c: c.get_point_cloud_config(), ip, timeout, ptc_port)


def set_filter_type(ip, filter_type, timeout=2.0, ptc_port=PTC_PORT):
    """
    Set point-cloud filter type; leave ultra_precise unchanged.

    Raises HesaiPtcError on SET failure or filter readback mismatch.
    """
    filt = int(filter_type)

    def _set(client):
        client.set_point_cloud_config_selective(POINT_CLOUD_KEEP_CURRENT, filt)
        ultra, readback = client.get_point_cloud_config()
        if readback != filt:
            raise HesaiPtcError(
                'SetPointCloudConfig filter readback mismatch: expected {}, got {} '
                '(ultra_precise={})'.format(filt, readback, ultra)
            )

    _run(_set, ip, timeout, ptc_port)


def get_inventory_info(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Read and parse lidar inventory information (serial number, model, etc.)."""
    raw = _run(lambda c: c.get_inventory_info_raw(), ip, timeout, ptc_port)
    if len(raw) < 136:
        raise HesaiPtcError("GetInventoryInfo returned too few bytes ({} < 136)".format(len(raw)))

    def decode_str(b):
        return b.split(b'\x00', 1)[0].decode('ascii', errors='replace').strip()

    sn = decode_str(raw[0:18])
    model = decode_str(raw[18:50])
    calib_date = decode_str(raw[50:66])

    mac_bytes = raw[66:72]
    mac = ':'.join('{:02x}'.format(b) for b in mac_bytes)

    hw_version = decode_str(raw[72:104])
    sw_version = decode_str(raw[104:120])
    fpga_version = decode_str(raw[120:136])

    build_id = ""
    if len(raw) >= 194:
        build_id = decode_str(raw[184:194])

    return {
        'serial_number': sn,
        'model': model,
        'calibration_date': calib_date,
        'mac_address': mac,
        'hardware_version': hw_version,
        'software_version': sw_version,
        'fpga_version': fpga_version,
        'build_id': build_id,
    }




