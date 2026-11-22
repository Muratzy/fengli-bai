#!/usr/bin/env python3
"""Q1 swing capture. Default observes only; --action start explicitly starts once.

COM9 is receive-only. The optional start/stop command is a single non-halting
J-Link RAM mailbox write, so a J-Link-TX to PA10 wire is not required.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
import time

import serial

ROOT = Path(__file__).resolve().parents[1]
HEADER = (
    'time_ms,gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw,roll_cdeg,pitch_cdeg,accel_weight_permille,'
    'target_roll_cdeg,target_pitch_cdeg,temp_cdeg,p_roll_milli,i_roll_milli,d_roll_milli,'
    'p_pitch_milli,i_pitch_milli,d_pitch_milli,pid_roll_milli,pid_pitch_milli,target_position_mm,'
    'target_velocity_mm_s,pwm1_requested_permille,pwm2_requested_permille,pwm3_requested_permille,'
    'pwm4_requested_permille,pwm1_applied_permille,pwm2_applied_permille,pwm3_applied_permille,'
    'pwm4_applied_permille,bus_current_adc,mode,direction_cdeg,tilt_cdeg,direction_valid,'
    'pitch_rate_cdeg_s,q1_state,q1_reason,q1_run_ms,q1_switches,predicted_peak_cdeg,'
    'left_peak_cdeg,right_peak_cdeg,en_command,en_pin,keys'
).split(',')
REASONS = ['none', 'time_up', 'user_stop', 'sample_stale', 'angle_limit',
           'no_turn', 'en_low', 'not_centered']


def parse_line(line):
    values = line.decode('ascii').strip().split(',')
    if values == HEADER:
        return None
    if len(values) != len(HEADER):
        raise ValueError('Q1 protocol requires %d fields' % len(HEADER))
    row = dict(zip(HEADER, map(int, values)))
    if not 0 <= row['time_ms'] <= 0xFFFFFFFF:
        raise ValueError('Invalid MCU timestamp')
    if not (0 <= row['q1_state'] <= 4 and 0 <= row['q1_reason'] < len(REASONS)):
        raise ValueError('Unknown Q1 state/reason')
    for i in range(1, 5):
        if not 0 <= row['pwm%d_applied_permille' % i] <= 1000:
            raise ValueError('Unexpected duty')
    return row


def mailbox(action, directory):
    manifest = json.loads((ROOT / 'build/workflow/last-run.json').read_text(encoding='utf-8-sig'))
    elf = Path(manifest['elf'])
    if not manifest.get('flashVerified') or hashlib.sha256(elf.read_bytes()).hexdigest().upper() != manifest['sha256']:
        raise RuntimeError('Current built ELF is not verified as flashed')
    if manifest.get('probeSerial') != '602712599':
        raise RuntimeError('Unexpected probe in flash manifest')
    compiler_root = Path(r'C:\cubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins')
    matches = list(compiler_root.glob('com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3*/tools/bin/arm-none-eabi-nm.exe'))
    if len(matches) != 1:
        raise RuntimeError('Cannot resolve the configured STM32 nm tool')
    symbols = subprocess.run([str(matches[0]), '-g', str(elf)], check=True,
                             capture_output=True, text=True).stdout
    match = re.search(r'^([0-9a-fA-F]+) [Bb] g_q1_remote_command$', symbols, re.M)
    if not match:
        raise RuntimeError('Mailbox not found in flashed ELF')
    address = int(match[1], 16)
    if not 0x20000000 <= address < 0x20020000 or address % 4:
        raise RuntimeError('Mailbox address is outside SRAM')
    code = '0x5131474F' if action == 'start' else '0x51315354'
    commands = 'w4 0x%08X, %s\nexit\n' % (address, code)
    (directory / ('command_' + action + '.jlink')).write_text(commands, encoding='ascii')
    staging = Path(r'C:\Temp\WindPendulumTools')
    staging.mkdir(parents=True, exist_ok=True)
    script = staging / ('q1_command_' + action + '.jlink')
    script.write_text(commands, encoding='ascii')
    result = subprocess.run([
        r'C:\Users\Muratsy\SEGGER\JLink_V962\JLink.exe', '-device', 'STM32F407VE',
        '-if', 'SWD', '-speed', '4000', '-SelectEmuBySN', '602712599',
        '-autoconnect', '1', '-NoGui', '1', '-ExitOnError', '1',
        '-CommanderScript', str(script)], capture_output=True, timeout=15)
    output = result.stdout.decode('utf-8', errors='replace') + result.stderr.decode('utf-8', errors='replace')
    (directory / ('command_' + action + '.log')).write_text(output, encoding='utf-8')
    if result.returncode or 'Script processing completed' not in output or re.search(r'Error:|FAILED|Cannot connect', output, re.I):
        raise RuntimeError('J-Link command failed; see command log')


def board_time_restarted(previous, current):
    # Captures last at most 60 seconds. A normal uint32 rollover has a small
    # forward delta; a restart such as 798141 -> 60 does not.
    return current < previous and ((current - previous) & 0xFFFFFFFF) > 60000


def analyze(rows, capture_error=None):
    active = [r for r in rows if r['q1_state'] == 1]
    restarts = [{'previous_ms': a['time_ms'], 'current_ms': b['time_ms']}
                for a, b in zip(rows, rows[1:])
                if board_time_restarted(a['time_ms'], b['time_ms'])]
    terminal_observed = bool(active) and rows[-1]['q1_state'] in (2, 3, 4)
    result = {'rows': len(rows), 'active_rows': len(active), 'full_sampling_rate': False,
              'mcu_time_restarts': len(restarts), 'time_restart_events': restarts,
              'terminal_state_observed': terminal_observed,
              'valid_for_tuning': terminal_observed and not restarts and not capture_error}
    if rows:
        result['last_state'] = rows[-1]['q1_state']
        result['last_reason'] = REASONS[rows[-1]['q1_reason']]
        result['max_switches'] = max(r['q1_switches'] for r in rows)
    if active and not restarts:
        # Coaxial beam projection with the calibrated roll/pitch convention.
        right = [0.99 * math.tan(math.radians(r['pitch_cdeg'] / 100)) /
                 math.cos(math.radians(r['roll_cdeg'] / 100)) for r in active]
        up = [0.99 * math.tan(math.radians(r['roll_cdeg'] / 100)) for r in active]
        result.update({
            'active_recorded_span_ms': (active[-1]['time_ms'] - active[0]['time_ms']) & 0xFFFFFFFF,
            'estimated_ground_span_cm': (max(right) - min(right)) * 100,
            'max_estimated_cross_track_cm': max(abs(x) for x in up) * 100,
            'pitch_deg_range': [min(r['pitch_cdeg'] for r in active) / 100,
                                max(r['pitch_cdeg'] for r in active) / 100],
            'left_on_rows': sum(r['pwm1_applied_permille'] > 0 for r in active),
            'right_on_rows': sum(r['pwm2_applied_permille'] > 0 for r in active),
            'left_full_rows': sum(r['pwm1_applied_permille'] == 1000 for r in active),
            'right_full_rows': sum(r['pwm2_applied_permille'] == 1000 for r in active),
            'coast_rows': sum(r['pwm1_applied_permille'] == r['pwm2_applied_permille'] == 0 for r in active),
            'both_on_rows': sum(bool(r['pwm1_applied_permille'] and r['pwm2_applied_permille']) for r in active),
            'note': 'Angle-derived estimates only; a ruler/ground trace must verify actual 50 cm and linearity. '
                    'Endpoint switching uses PWM ramps and turn/coast thresholds, not PID gains. '
                    'Left motor PWM1 moves the pendulum right; right motor PWM2 moves it left.'})
    return result


def capture(args):
    directory = ROOT / '数据记录/串口' / dt.datetime.now().strftime('%Y%m%d_%H%M%S_COM9_q1_' + args.action)
    directory.mkdir(parents=True)
    if args.action == 'stop':
        mailbox('stop', directory)
    port = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    port.dtr = port.rts = False
    port.port = args.port
    rows, pending, invalid, sent = [], b'', 0, False
    started = time.monotonic()
    last_telemetry_at = started
    error = None
    metadata = {'action': args.action, 'port': args.port, 'serial_write': False,
                'host_sequence_is_not_mcu_sample_sequence': True}
    manifest_path = ROOT / 'build/workflow/last-run.json'
    if manifest_path.exists():
        metadata['local_firmware_manifest'] = json.loads(manifest_path.read_text(encoding='utf-8-sig'))
    print('Session: ' + str(directory), flush=True)
    try:
        port.open()
        with (directory / 'raw.bin').open('wb') as raw, (directory / 'telemetry.csv').open('w', encoding='utf-8-sig', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=HEADER)
            writer.writeheader()
            while time.monotonic() - started < args.duration:
                chunk = port.read(max(1, min(port.in_waiting, 4096)))
                received_at = time.monotonic()
                raw.write(chunk)
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    try:
                        row = parse_line(line)
                    except (ValueError, UnicodeError):
                        invalid += 1
                        continue
                    if row is not None:
                        previous_ms = rows[-1]['time_ms'] if rows else None
                        rows.append(row)
                        writer.writerow(row)
                        last_telemetry_at = received_at
                        if args.action == 'start' and previous_ms is not None and \
                                board_time_restarted(previous_ms, row['time_ms']):
                            raise RuntimeError('MCU time restarted: %d -> %d ms; trial aborted' %
                                               (previous_ms, row['time_ms']))
                if sent and received_at - last_telemetry_at > 0.5:
                    raise RuntimeError('Telemetry lost for more than 0.5 s; trial aborted')
                if args.action == 'start' and not sent and received_at - started >= 1.0:
                    recent = rows[-20:]
                    if len(recent) < 20 or received_at - last_telemetry_at > 0.5 or any(r['mode'] != 2 or r['q1_state'] == 1 or
                        abs(r['pitch_cdeg']) > 700 or abs(r['roll_cdeg']) > 700 or
                        r['keys'] & 4 or
                        r['en_command'] != 0 or r['en_pin'] != 0 or
                        any(r['pwm%d_applied_permille' % i] for i in range(1,5))
                        for r in recent):
                        raise RuntimeError('Pre-start check failed: require READY within +/-7 degrees, zero PWM and EN off')
                    # If the write succeeds but logging fails, ensure finally still stops.
                    sent = True
                    mailbox('start', directory)
                    print('One start command sent; firmware limits the trial to 15 seconds.', flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__
    finally:
        port.close()
        if sent:
            try:
                mailbox('stop', directory)
            except Exception as exc:
                error = (error or '') + '; final stop failed: ' + str(exc)
    report = analyze(rows, capture_error=error)
    if not error and not rows:
        error = 'No valid telemetry'
    if not error and sent and not any(r['q1_state'] == 1 for r in rows):
        error = 'Start was sent but RUNNING was not observed'
    metadata.update(start_sent=sent, invalid_or_partial_lines=invalid, error=error,
                    duration_s=time.monotonic() - started, trailing_bytes=len(pending), analysis=report)
    (directory / 'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=['observe', 'start', 'stop'], default='observe')
    parser.add_argument('--port', default='COM9')
    parser.add_argument('--duration', type=float, default=20.0)
    args = parser.parse_args()
    if not 0 < args.duration <= 60 or (args.action == 'start' and args.duration < 18):
        parser.error('duration must be 0..60 s, at least 18 s for a start trial')
    capture(args)
