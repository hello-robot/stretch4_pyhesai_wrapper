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
FILTER_STRONGEST = 3

FILTER_NAMES = {
    FILTER_DISABLED: 'disabled',
    FILTER_MEDIUM: 'medium',
    FILTER_STRONG: 'strong',
    FILTER_STRONGEST: 'strongest',
}

ULTRA_PRECISE_NAMES = {
    0: 'low',
    1: 'medium',
    2: 'strong',
    3: 'off',
}

# POINT_CLOUD_MODE extended PTC subcommands (cmd 0xFF)
PTC_CMD_HAS_SUBCOMMAND = 0xFF
PTC_SET_POINT_CLOUD_MODE_SUBCMD = 0x00000164
PTC_GET_POINT_CLOUD_MODE_SUBCMD = 0x00000165
PTC_UPGRADE_LIDAR_CMD = 0x83

POINT_CLOUD_MODE_GENERAL = 0
POINT_CLOUD_MODE_MAPPING = 1
POINT_CLOUD_MODE_MAPPING_GROUND = 2

POINT_CLOUD_MODE_NAMES = {
    POINT_CLOUD_MODE_GENERAL: 'general',
    POINT_CLOUD_MODE_MAPPING: 'mapping',
    POINT_CLOUD_MODE_MAPPING_GROUND: 'mapping_ground',
}

# Provisional min APP letter for Strongest filter + POINT_CLOUD_MODE.
# Update ONLY is_new_firmware_supported() when Hesai confirms the compare rule.
_NEW_FW_APP_PREFIX = '15.AF.B0.00.02.'
_NEW_FW_MIN_LETTER = 'Y'


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

    FILTER_STRONGEST (3) requires new firmware (see is_new_firmware_supported).
    Raises HesaiPtcError on SET failure or filter readback mismatch.
    """
    filt = int(filter_type)
    if filt == FILTER_STRONGEST:
        require_new_firmware(ip, timeout=timeout, ptc_port=ptc_port)

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


def is_new_firmware_supported(ip, timeout=2.0, ptc_port=PTC_PORT):
    """
    Return True if lidar FW supports Strongest filter + POINT_CLOUD_MODE.

    Rule: APP letter >= 'Y' for inventory
    hardware_version strings like 15.AF.B0.00.02.Y0 / ...H0.

    """
    app = get_inventory_info(ip, timeout=timeout, ptc_port=ptc_port)['hardware_version']
    if not app:
        return False
    # Normalize trailing digit bleed (Y0 -> Y) while keeping the letter suffix.
    normalized = app.rstrip('0123456789')
    if not normalized.startswith(_NEW_FW_APP_PREFIX):
        return False
    suffix = normalized[len(_NEW_FW_APP_PREFIX):]
    if len(suffix) != 1 or not suffix.isalpha():
        return False
    return suffix.upper() >= _NEW_FW_MIN_LETTER


def require_new_firmware(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Raise HesaiPtcError if lidar FW does not support the new PTC features."""
    if is_new_firmware_supported(ip, timeout=timeout, ptc_port=ptc_port):
        return
    app = get_inventory_info(ip, timeout=timeout, ptc_port=ptc_port)['hardware_version']
    raise HesaiPtcError(
        'Firmware too old for this feature: app={!r} (need {}{} or later)'.format(
            app, _NEW_FW_APP_PREFIX, _NEW_FW_MIN_LETTER
        )
    )


def _parse_extended_u8_payload(raw, subcmd):
    """Parse a 1-byte value from an extended PTC response (optional subcmd echo)."""
    if isinstance(raw, str):
        data = raw.encode('latin1')
    else:
        data = bytes(raw)
    if len(data) >= 5 and struct.unpack('>I', data[0:4])[0] == subcmd:
        return data[4]
    if len(data) >= 1:
        return data[0] if len(data) == 1 else data[-1]
    raise HesaiPtcError(
        'Extended PTC response too short for subcmd 0x{:08X} ({} bytes)'.format(
            subcmd, len(data)
        )
    )


def get_point_cloud_mode(ip, timeout=2.0, ptc_port=PTC_PORT):
    """Read POINT_CLOUD_MODE via GET [0xFF, 0x00000165]. Requires new firmware."""
    require_new_firmware(ip, timeout=timeout, ptc_port=ptc_port)
    payload = struct.pack('>I', PTC_GET_POINT_CLOUD_MODE_SUBCMD)

    def _get(client):
        raw = client.query_command(PTC_CMD_HAS_SUBCOMMAND, payload)
        return _parse_extended_u8_payload(raw, PTC_GET_POINT_CLOUD_MODE_SUBCMD)

    return _run(_get, ip, timeout, ptc_port)


def set_point_cloud_mode(ip, mode, timeout=2.0, ptc_port=PTC_PORT):
    """
    Set POINT_CLOUD_MODE via SET [0xFF, 0x00000164] and verify readback.

    Modes: 0 general, 1 mapping, 2 mapping+ground. Requires new firmware.
    """
    require_new_firmware(ip, timeout=timeout, ptc_port=ptc_port)
    mode = int(mode)
    if mode not in POINT_CLOUD_MODE_NAMES:
        raise HesaiPtcError(
            'POINT_CLOUD_MODE must be 0, 1, or 2 (got {})'.format(mode)
        )
    payload = struct.pack('>IB', PTC_SET_POINT_CLOUD_MODE_SUBCMD, mode)

    def _set(client):
        client.query_command(PTC_CMD_HAS_SUBCOMMAND, payload)
        get_payload = struct.pack('>I', PTC_GET_POINT_CLOUD_MODE_SUBCMD)
        raw = client.query_command(PTC_CMD_HAS_SUBCOMMAND, get_payload)
        readback = _parse_extended_u8_payload(raw, PTC_GET_POINT_CLOUD_MODE_SUBCMD)
        if readback != mode:
            raise HesaiPtcError(
                'SetPointCloudMode readback mismatch: expected {}, got {}'.format(
                    mode, readback
                )
            )

    _run(_set, ip, timeout, ptc_port)


def upgrade_lidar_firmware(
    ip,
    file_path,
    progress_callback=None,
    timeout=10.0,
    ptc_port=PTC_PORT,
    cmd_id=PTC_UPGRADE_LIDAR_CMD,
    is_extern=0,
):
    """
    Upload a firmware patch via PTC Upgrade Safe Image (0x83) with progress.

    progress_callback, if given, is called with a float percent (0-100).
    The lidar typically reboots after a successful transfer.
    """
    if not os.path.isfile(file_path):
        raise HesaiPtcError('Firmware file not found: {}'.format(file_path))

    def _upgrade(client):
        if progress_callback is not None:
            client.set_upgrade_percent_callback(progress_callback)
        client.upgrade_lidar_patch(file_path, int(cmd_id), int(is_extern))

    _run(_upgrade, ip, timeout, ptc_port)




