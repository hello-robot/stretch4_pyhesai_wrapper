#!/usr/bin/env python3
"""
REx_ptp_manager.py

Installs and checks the PTP grandmaster that keeps Stretch's Hesai lidar
clocks synchronized to the robot's system clock.

--install installs linuxptp (ptp4l, phc2sys, pmc) if missing, then installs
and enables the lidar-ptp4l and lidar-phc2sys systemd services on the
lidar-facing NIC and verifies the robot is running as PTP grandmaster. It then
checks each lidar's return mode, noise filter, and PTP lock offset against the
values stretch_system_check expects, and offers to apply any that differ.

Ported from stretch_production_tools_ii fab_tests/test_FAB_ptp_grandmaster.py.
"""

from __future__ import annotations
import argparse
import os
import re
import subprocess
import sys
import tempfile
import time

NUC_LIDAR_SUBNET_IP = '192.168.1.2'

GRANDMASTER_POLL_TIMEOUT_S = 30
GRANDMASTER_POLL_INTERVAL_S = 1

LINUXPTP_APT_PACKAGE = 'linuxptp'
LINUXPTP_BINARIES = ('pmc', 'ptp4l', 'phc2sys')
PMC_BIN = '/usr/sbin/pmc'

SERVICE_NAMES = ('lidar-ptp4l', 'lidar-phc2sys')

# ---------------------------------------------------------------------------
# Installed files. '{interface}' is replaced with the lidar-facing NIC name.
# ---------------------------------------------------------------------------

LIDAR_PTP4L_SERVICE = """\
[Unit]
Description=Lidar PTP grandmaster on {interface}
Documentation=man:ptp4l(8)
Wants=network-online.target
After=network-online.target sys-subsystem-net-devices-{interface}.device systemd-timesyncd.service
BindsTo=sys-subsystem-net-devices-{interface}.device

[Service]
Type=simple
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf -i {interface}
ExecStartPost=/usr/local/sbin/lidar-ptp-gm-settings
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
"""

LIDAR_PHC2SYS_SERVICE = """\
[Unit]
Description=Synchronize lidar NIC PHC from robot system clock
Documentation=man:phc2sys(8)
Requires=lidar-ptp4l.service
After=lidar-ptp4l.service systemd-timesyncd.service

[Service]
Type=simple
ExecStart=/usr/sbin/phc2sys -s CLOCK_REALTIME -c {interface} -O 0
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
"""

LIDAR_PTP_GM_SETTINGS = """\
#!/usr/bin/env bash
set -euo pipefail

for _ in {1..20}; do
  if /usr/sbin/pmc -u -b 0 "SET GRANDMASTER_SETTINGS_NP clockClass 6 clockAccuracy 0x29 offsetScaledLogVariance 0xffff currentUtcOffset 37 leap61 0 leap59 0 currentUtcOffsetValid 1 ptpTimescale 1 timeTraceable 1 frequencyTraceable 1 timeSource 0x50"; then
    exit 0
  fi
  sleep 1
done

exit 1
"""

PTP4L_CONF = """\
[global]
#
# Default Data Set
#
twoStepFlag		1
clientOnly		0
socket_priority		0
priority1		128
priority2		128
domainNumber		0
#utc_offset		37
clockClass              6
clockAccuracy           0x29
offsetScaledLogVariance	0xFFFF
free_running		0
freq_est_interval	1
dscp_event		0
dscp_general		0
dataset_comparison	ieee1588
G.8275.defaultDS.localPriority	128
maxStepsRemoved		255
#
# Port Data Set
#
logAnnounceInterval	1
logSyncInterval		0
operLogSyncInterval	0
logMinDelayReqInterval	0
logMinPdelayReqInterval	0
operLogPdelayReqInterval 0
announceReceiptTimeout	3
syncReceiptTimeout	0
delay_response_timeout	0
delayAsymmetry		0
fault_reset_interval	4
neighborPropDelayThresh	20000000
serverOnly		0
G.8275.portDS.localPriority	128
asCapable               auto
BMCA                    ptp
inhibit_announce        0
inhibit_delay_req       0
ignore_source_id        0
power_profile.2011.grandmasterTimeInaccuracy	-1
power_profile.2011.networkTimeInaccuracy	-1
power_profile.2017.totalTimeInaccuracy		-1
power_profile.grandmasterID			0
power_profile.version				none
ptp_minor_version       0
#
# Run time options
#
assume_two_step		0
logging_level		6
path_trace_enabled	0
follow_up_info		0
hybrid_e2e		0
inhibit_multicast_service	0
net_sync_monitor	0
tc_spanning_tree	0
tx_timestamp_timeout	10
unicast_listen		0
unicast_master_table	0
unicast_req_duration	3600
use_syslog		1
verbose			0
summary_interval	0
kernel_leap		1
check_fup_sync		0
clock_class_threshold	248
#
# Servo Options
#
pi_proportional_const	0.0
pi_integral_const	0.0
pi_proportional_scale	0.0
pi_proportional_exponent	-0.3
pi_proportional_norm_max	0.7
pi_integral_scale	0.0
pi_integral_exponent	0.4
pi_integral_norm_max	0.3
step_threshold		0.0
first_step_threshold	0.00002
max_frequency		900000000
clock_servo		pi
sanity_freq_limit	200000000
refclock_sock_address	/var/run/refclock.ptp.sock
ntpshm_segment		0
msg_interval_request	0
servo_num_offset_values 10
servo_offset_threshold  0
write_phase_mode	0
#
# Transport options
#
transportSpecific	0x0
ptp_dst_mac		01:1B:19:00:00:00
p2p_dst_mac		01:80:C2:00:00:0E
udp_ttl			1
udp6_scope		0x0E
uds_address		/var/run/ptp4l
uds_file_mode		0660
uds_ro_address		/var/run/ptp4lro
uds_ro_file_mode		0666
#
# Default interface options
#
clock_type		OC
network_transport	UDPv4
delay_mechanism		E2E
time_stamping		hardware
tsproc_mode		filter
delay_filter		moving_median
delay_filter_length	10
egressLatency		0
ingressLatency		0
boundary_clock_jbod	0
phc_index		-1
#
# Clock description
#
productDescription	;;
revisionData		;;
manufacturerIdentity	00:00:00
userDescription		;
timeSource              0x50
"""

# (content, destination, substitute interface, executable)
INSTALL_FILES = (
    (LIDAR_PTP4L_SERVICE, '/etc/systemd/system/lidar-ptp4l.service', True, False),
    (LIDAR_PHC2SYS_SERVICE, '/etc/systemd/system/lidar-phc2sys.service', True, False),
    (LIDAR_PTP_GM_SETTINGS, '/usr/local/sbin/lidar-ptp-gm-settings', False, True),
    (PTP4L_CONF, '/etc/linuxptp/ptp4l.conf', False, False),
)


def linuxptp_binaries_present() -> bool:
    """Return True if pmc, ptp4l, and phc2sys are installed under /usr/sbin."""
    return all(os.path.isfile(os.path.join('/usr/sbin', name)) for name in LINUXPTP_BINARIES)


def ensure_linuxptp_installed() -> None:
    """Install the linuxptp apt package if required binaries are missing."""
    if linuxptp_binaries_present():
        print("linuxptp: already installed")
        return
    print(f"Installing apt package {LINUXPTP_APT_PACKAGE}...")
    res = subprocess.run(['sudo', 'apt-get', 'install', '-y', LINUXPTP_APT_PACKAGE],
                         capture_output=True, text=True, check=False)
    if res.returncode != 0:
        raise RuntimeError(f"Failed to install {LINUXPTP_APT_PACKAGE}. "
                           f"stdout:\n{res.stdout}\nstderr:\n{res.stderr}")
    if not linuxptp_binaries_present():
        raise RuntimeError(f"{LINUXPTP_APT_PACKAGE} installed but binaries still missing: "
                           f"{', '.join(LINUXPTP_BINARIES)}")
    print("linuxptp: installed")


def discover_nuc_ptp_interface() -> str | None:
    """Find the ethernet interface with 192.168.1.2 assigned."""
    res = subprocess.run(['ip', '-4', '-o', 'addr', 'show'],
                         capture_output=True, text=True, check=False)
    if res.returncode != 0:
        return None
    for line in res.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[3].startswith(NUC_LIDAR_SUBNET_IP + '/'):
            return parts[1]
    return None


def _install_file(content: str, dest_path: str, interface: str | None = None, executable: bool = False) -> None:
    """Write content to a system path via sudo."""
    if interface is not None:
        content = content.replace('{interface}', interface)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', delete=False) as tmp:
        tmp.write(content)
        tmp_path = tmp.name
    try:
        subprocess.run(['sudo', 'mkdir', '-p', os.path.dirname(dest_path)], check=True)
        subprocess.run(['sudo', 'cp', tmp_path, dest_path], check=True)
        subprocess.run(['sudo', 'chmod', '755' if executable else '644', dest_path], check=True)
    finally:
        os.unlink(tmp_path)


def _parse_pmc_key_values(output: str) -> dict[str, str]:
    """Parse pmc management response lines into a key/value dict."""
    values = {}
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith('sending:') or 'RESPONSE' in stripped:
            continue
        match = re.match(r'^(\S+)\s+(.+)$', stripped)
        if match:
            values[match.group(1)] = match.group(2).strip()
    return values


def _pmc_get(query: str) -> dict[str, str]:
    res = subprocess.run(['sudo', PMC_BIN, '-u', '-b', '0', f'GET {query}'],
                         capture_output=True, text=True, check=False)
    return _parse_pmc_key_values(res.stdout)


def _service_state(name: str) -> str:
    res = subprocess.run(['systemctl', 'is-active', name], capture_output=True, text=True, check=False)
    return res.stdout.strip()


def install_grandmaster() -> bool:
    interface = discover_nuc_ptp_interface()
    if interface is None:
        print(f"ERROR: Could not find a network interface with {NUC_LIDAR_SUBNET_IP} assigned. "
              "Configure the lidar network first.")
        return False
    print(f"Lidar PTP interface: {interface}")

    ensure_linuxptp_installed()

    print("Installing PTP grandmaster services...")
    for content, dest_path, use_interface, executable in INSTALL_FILES:
        print(f"  -> {dest_path}")
        _install_file(content, dest_path, interface=interface if use_interface else None, executable=executable)

    subprocess.run(['sudo', 'systemctl', 'daemon-reload'], check=True)
    for name in SERVICE_NAMES:
        subprocess.run(['sudo', 'systemctl', 'enable', '--now', f'{name}.service'], check=True)
        # restart so a reinstall picks up new unit files/config
        subprocess.run(['sudo', 'systemctl', 'restart', f'{name}.service'], check=True)

    ok = True
    for name in SERVICE_NAMES:
        state = _service_state(name)
        print(f"{name}: {state}")
        if state != 'active':
            print(f"ERROR: {name} is not active. Check: journalctl -u {name}")
            ok = False
    if not ok:
        return False

    print("Waiting for robot to become PTP grandmaster (portState MASTER)...")
    deadline = time.time() + GRANDMASTER_POLL_TIMEOUT_S
    port_state = ''
    while time.time() < deadline:
        port_state = _pmc_get('PORT_DATA_SET').get('portState', '')
        if port_state == 'MASTER':
            break
        time.sleep(GRANDMASTER_POLL_INTERVAL_S)
    print(f"portState: {port_state or '(unknown)'}")
    if port_state != 'MASTER':
        print(f"ERROR: portState did not become MASTER within {GRANDMASTER_POLL_TIMEOUT_S}s.")
        return False

    gm = _pmc_get('GRANDMASTER_SETTINGS_NP')
    for key in ('clockClass', 'clockAccuracy', 'timeSource', 'offsetScaledLogVariance'):
        if key in gm:
            print(f"{key}: {gm[key]}")
    if gm.get('clockClass') != '6' or gm.get('clockAccuracy') != '0x29':
        print("ERROR: grandmaster settings were not applied (expected clockClass 6, clockAccuracy 0x29).")
        return False

    print("PTP grandmaster installed and verified. The lidars may take a few seconds to lock.")
    return True


def _lidars() -> tuple[tuple[str, str], ...]:
    from stretch4_pyhesai_wrapper.ptc_client import LEFT_LIDAR_IP, RIGHT_LIDAR_IP
    return (('left', LEFT_LIDAR_IP), ('right', RIGHT_LIDAR_IP))


def _lidar_settings():
    """(label, getter, setter, expected, format) for each lidar setting stretch_system_check expects."""
    from stretch4_pyhesai_wrapper import ptc_client as ptc
    return (
        ('Return mode', ptc.get_return_mode, ptc.set_return_mode, ptc.RETURN_MODE_LAST_AND_STRONGEST,
         lambda v: f"{v} ({ptc.RETURN_MODE_NAMES.get(v, 'unknown')})"),
        ('Noise filter', lambda ip: ptc.get_point_cloud_config(ip)[1], ptc.set_filter_type, ptc.FILTER_STRONG,
         lambda v: f"{v} ({ptc.FILTER_NAMES.get(v, 'unknown')})"),
        ('PTP lock offset', ptc.get_ptp_lock_offset_us, ptc.set_ptp_lock_offset_us, ptc.PTP_LOCK_OFFSET_US,
         lambda v: f"{v} us"),
    )


def check_lidar_config(side: str, ip: str) -> list | None:
    """Print each setting of one lidar. Returns the mismatched settings, or None if the lidar can't be queried."""
    from stretch4_pyhesai_wrapper.ptc_client import HesaiPtcError
    mismatched = []
    for label, getter, setter, expected, fmt in _lidar_settings():
        try:
            value = getter(ip)
        except HesaiPtcError as e:
            print(f"  {label}: unavailable ({e})")
            return None
        if value == expected:
            print(f"  {label}: {fmt(value)}")
        else:
            print(f"  {label}: {fmt(value)}  (expected: {fmt(expected)})")
            mismatched.append((label, setter, expected, fmt))
    return mismatched


def _confirm(question: str, assume_yes: bool = False) -> bool:
    if assume_yes:
        print(f"{question} [Y/n] y (--yes)")
        return True
    try:
        return not input(f"{question} [Y/n] ").strip().lower().startswith('n')
    except EOFError:
        return False


def configure_lidars(assume_yes: bool = False) -> bool:
    from stretch4_pyhesai_wrapper.ptc_client import HesaiPtcError
    ok = True
    for side, ip in _lidars():
        print(f"{side} lidar ({ip}) configuration:")
        mismatched = check_lidar_config(side, ip)
        if mismatched is None:
            print(f"ERROR: could not query the {side} lidar. Check that it is powered on and connected.")
            ok = False
            continue
        if not mismatched:
            continue
        labels = ', '.join(label for label, *_ in mismatched)
        if not _confirm(f"Change {labels} on the {side} lidar to the expected values?", assume_yes):
            print(f"Skipped. The {side} lidar is not in the expected configuration.")
            ok = False
            continue
        for label, setter, expected, fmt in mismatched:
            try:
                setter(ip, expected)
                print(f"  {label}: set to {fmt(expected)}")
            except HesaiPtcError as e:
                print(f"ERROR: failed to set {label} on the {side} lidar: {e}")
                ok = False
    return ok


def install(assume_yes: bool = False) -> bool:
    gm_ok = install_grandmaster()
    lidars_ok = configure_lidars(assume_yes)
    return gm_ok and lidars_ok


def status() -> bool:
    from stretch4_pyhesai_wrapper.ptc_client import ACCEPTABLE_PTP_STATUSES, get_lidar_ptp_status, HesaiPtcError
    ok = True
    for name in SERVICE_NAMES:
        state = _service_state(name)
        print(f"{name}: {state}")
        ok &= state == 'active'
    for side, ip in _lidars():
        print(f"{side} lidar ({ip}):")
        try:
            s = get_lidar_ptp_status(ip)
            print(f"  PTP status: {s['ptp_status_name']}")
            ok &= s['ptp_status'] in ACCEPTABLE_PTP_STATUSES
        except HesaiPtcError as e:
            print(f"  PTP status: unavailable ({e})")
            ok = False
            continue
        mismatched = check_lidar_config(side, ip)
        ok &= mismatched == []
    if not ok:
        print("PTP or lidar configuration is not healthy. Run `REx_ptp_manager --install` to fix it.")
    return ok


def main():
    parser = argparse.ArgumentParser(
        description="Install and check the PTP grandmaster that synchronizes Stretch's lidar clocks, "
                    "and the lidar settings it depends on."
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--install", action="store_true",
                       help="Install linuxptp and the lidar-ptp4l/lidar-phc2sys services (uses sudo), "
                            "then offer to fix lidar settings that differ from the expected values")
    group.add_argument("--status", action="store_true",
                       help="Show PTP service state, the lidars' PTP status, and their return mode, "
                            "noise filter, and PTP lock offset")
    parser.add_argument("-y", "--yes", action="store_true",
                        help="With --install, apply the expected lidar settings without asking")
    args = parser.parse_args()

    ok = install(assume_yes=args.yes) if args.install else status()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
