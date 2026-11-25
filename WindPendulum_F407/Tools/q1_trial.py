#!/usr/bin/env python3
"""Q1/Q2 capture. Default observes only; --action start explicitly starts once.

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

from trial_report import first_terminal_index, pitch_period, pitch_phase_lag_ms, pwm_saturation, save_curves

ROOT = Path(__file__).resolve().parents[1]
LEGACY_HEADER = (
    'time_ms,gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw,roll_cdeg,pitch_cdeg,accel_weight_permille,'
    'target_roll_cdeg,target_pitch_cdeg,temp_cdeg,p_roll_milli,i_roll_milli,d_roll_milli,'
    'p_pitch_milli,i_pitch_milli,d_pitch_milli,pid_roll_milli,pid_pitch_milli,target_position_mm,'
    'target_velocity_mm_s,pwm1_requested_permille,pwm2_requested_permille,pwm3_requested_permille,'
    'pwm4_requested_permille,pwm1_applied_permille,pwm2_applied_permille,pwm3_applied_permille,'
    'pwm4_applied_permille,bus_current_adc,mode,direction_cdeg,tilt_cdeg,direction_valid,'
    'pitch_rate_cdeg_s,q1_state,q1_reason,q1_run_ms,q1_switches,predicted_peak_cdeg,'
    'left_peak_cdeg,right_peak_cdeg,en_command,en_pin,keys'
).split(',')
PID_HEADER = LEGACY_HEADER + (
    'problem_id,target_length_mm,roll_rate_cdeg_s,target_pitch_rate_cdeg_s,'
    'filtered_pitch_rate_error_cdeg_s,pitch_integrating,pitch_kp_milli,'
    'pitch_ki_milli,pitch_kd_milli,reference_period_ms,reference_ramp_ms'
).split(',')
PWM_HEADER = PID_HEADER + ['pwm_limit_permille']
AMPLITUDE_HEADER = PWM_HEADER + ['q1_control_phase', 'q1_amplitude_cdeg',
                       'q1_amplitude_target_cdeg', 'q1_amplitude_error_cdeg',
                       'q1_amplitude_output_milli', 'q1_amplitude_ka_milli',
                       'q1_amplitude_kcomp_milli', 'q1_amplitude_blend_permille']
HEADER = AMPLITUDE_HEADER + ['q1_vcom_trial_id']
HEADERS_BY_COUNT = {len(fields): fields for fields in
                    (LEGACY_HEADER, PID_HEADER, PWM_HEADER, AMPLITUDE_HEADER, HEADER)}
Q2_EXTRA_FIELDS = ['host_rx_time', 'host_elapsed_s', 'requested_length_cm',
    'roll_deg', 'pitch_deg', 'target_roll_deg', 'target_pitch_deg',
    'roll_rate_dps', 'pitch_rate_dps', 'target_pitch_rate_dps',
    'pitch_error_deg', 'ground_x_cm', 'ground_y_cm',
    'pitch_p_permille', 'pitch_i_permille', 'pitch_d_permille', 'pitch_u_permille',
    'pwm1_percent', 'pwm2_percent']
REASONS = ['none', 'time_up', 'user_stop', 'sample_stale', 'angle_limit',
           'no_turn', 'en_low', 'not_centered', 'invalid_config']


def parse_line(line):
    values = line.decode('ascii').strip().split(',')
    names = HEADERS_BY_COUNT.get(len(values))
    if values == names:
        return None
    if names is None:
        raise ValueError('Trial protocol requires 47, 58, 59, 67, or 68 fields')
    row = dict.fromkeys(HEADER, 0)
    row.update(zip(names, map(int, values)))
    if names is LEGACY_HEADER:
        row['problem_id'] = 2 if row['mode'] == 8 else 1
    if not 0 <= row['time_ms'] <= 0xFFFFFFFF:
        raise ValueError('Invalid MCU timestamp')
    if not (0 <= row['q1_state'] <= 4 and 0 <= row['q1_reason'] < len(REASONS)):
        raise ValueError('Unknown Q1 state/reason')
    for i in range(1, 5):
        if not 0 <= row['pwm%d_applied_permille' % i] <= 1000:
            raise ValueError('Unexpected duty')
    if not 0 <= row['pwm_limit_permille'] <= 1000:
        raise ValueError('Unexpected PWM limit')
    if not 0 <= row['q1_control_phase'] <= 2:
        raise ValueError('Unexpected Q1 control phase')
    if not 0 <= row['q1_amplitude_blend_permille'] <= 1000:
        raise ValueError('Unexpected Q1 amplitude blend')
    return row


def q2_csv_row(row, requested_mm, host_elapsed_s):
    result = dict(row)
    result.update(host_rx_time=dt.datetime.now().astimezone().isoformat(timespec='milliseconds'),
                  host_elapsed_s=round(host_elapsed_s, 6), requested_length_cm=requested_mm/10)
    for name in ['roll', 'pitch', 'target_roll', 'target_pitch']:
        result[name + '_deg'] = row[name + '_cdeg']/100
    for name in ['roll', 'pitch', 'target_pitch']:
        result[name + '_rate_dps'] = row[name + '_rate_cdeg_s']/100
    result['pitch_error_deg'] = (row['target_pitch_cdeg'] - row['pitch_cdeg'])/100
    result['ground_x_cm'] = 99*math.tan(math.radians(row['pitch_cdeg']/100))/math.cos(math.radians(row['roll_cdeg']/100))
    result['ground_y_cm'] = 99*math.tan(math.radians(row['roll_cdeg']/100))
    for part in ['p', 'i', 'd']:
        result['pitch_' + part + '_permille'] = row[part + '_pitch_milli']/1000
    result['pitch_u_permille'] = row['pid_pitch_milli']/1000
    for side in [1, 2]:
        result['pwm%d_percent'%side] = row['pwm%d_applied_permille'%side]/10
    return result


def write_q2_fields(directory):
    with (directory/'fields.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['field', 'unit', 'description'])
        for name in HEADER + Q2_EXTRA_FIELDS:
            unit = 'integer'
            if name == 'temp_cdeg': unit = '0.01 Celsius'
            elif name.endswith('_cdeg_s'): unit = '0.01 deg/s'
            elif name.endswith('_cdeg'): unit = '0.01 deg'
            elif name == 'pitch_kp_milli': unit = '(PWM permille/deg) x1000'
            elif name == 'pitch_ki_milli': unit = '(PWM permille/(deg*s)) x1000'
            elif name == 'pitch_kd_milli': unit = '(PWM permille/(deg/s)) x1000'
            elif name.endswith('_milli'): unit = '0.001 PWM permille'
            elif name.endswith('_permille'): unit = 'permille'
            elif name.endswith('_deg'): unit = 'deg'
            elif name.endswith('_dps'): unit = 'deg/s'
            elif name.endswith('_cm'): unit = 'cm'
            elif name.endswith('_mm_s'): unit = 'mm/s'
            elif name.endswith('_mm'): unit = 'mm'
            elif name.endswith('_ms'): unit = 'ms'
            elif name.endswith('_percent'): unit = '%'
            elif name == 'host_rx_time': unit = 'ISO8601 host time'
            elif name == 'host_elapsed_s': unit = 'host seconds'
            elif name.endswith('_adc') or name.endswith('_raw'): unit = 'raw count'
            description = 'Host-derived value' if name in Q2_EXTRA_FIELDS else 'MCU telemetry snapshot'
            if name == 'problem_id': description = '1=Q1, 2=Q2; explicit selected task'
            if name == 'q1_switches': description = 'Applied PWM polarity changes; legacy Q1 rows used endpoint decisions'
            if name == 'pitch_integrating': description = '1=integral enabled; 0=disabled or stopped'
            if name == 'pwm_limit_permille': description = 'MCU duty limit; 0=not reported'
            writer.writerow([name, unit, description])


def command_script(action, address, problem='q1', length_address=None, length_mm=500):
    if action not in ('start', 'stop') or problem not in ('q1', 'q2'):
        raise ValueError('Unknown trial command')
    def checked(value):
        if value is None or not 0x20000000 <= value < 0x20020000 or value % 4:
            raise ValueError('Command address is outside aligned SRAM')
        return value
    checked(address)
    lines = []
    if action == 'start' and problem == 'q2':
        if not 300 <= length_mm <= 600 or int(length_mm) != length_mm:
            raise ValueError('Q2 length must be 300..600 integer mm')
        lines.append('w4 0x%08X, 0x%08X' % (checked(length_address), length_mm))
    code = 0x51315354 if action == 'stop' else (0x5132474F if problem == 'q2' else 0x5131474F)
    lines.extend(['w4 0x%08X, 0x%08X' % (address, code), 'exit'])
    return '\n'.join(lines) + '\n'


def mailbox(action, directory, problem='q1', length_mm=500):
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
    length_address = None
    if action == 'start' and problem == 'q2':
        match_length = re.search(r'^([0-9a-fA-F]+) [BbDd] g_q2_requested_length_mm$', symbols, re.M)
        if not match_length:
            raise RuntimeError('Q2 configuration mailbox not found in flashed ELF')
        length_address = int(match_length[1], 16)
    commands = command_script(action, address, problem, length_address, length_mm)
    (directory / ('command_' + action + '.jlink')).write_text(commands, encoding='ascii')
    staging = Path(r'C:\Temp\WindPendulumTools')
    staging.mkdir(parents=True, exist_ok=True)
    script = staging / (problem + '_command_' + action + '.jlink')
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


def reference_cycle_metrics(active, stop_ms=None):
    settings = {(r['problem_id'], r['reference_period_ms'], r['reference_ramp_ms'], r['target_length_mm'])
                for r in active}
    if len(settings) != 1: return []
    problem, period, ramp, length = next(iter(settings))
    if problem not in (1, 2) or period <= 0 or length <= 0:
        return []
    groups = {}
    for row in active:
        groups.setdefault(row['q1_run_ms']//period, []).append(row)
    result = []
    for index, group in sorted(groups.items()):
        start_ms = index * period
        end_ms = start_ms + period
        if start_ms < ramp or len(group) < 5 or \
                (stop_ms is not None and end_ms > stop_ms) or \
                group[0]['q1_run_ms'] - start_ms > 120 or \
                end_ms - group[-1]['q1_run_ms'] > 120:
            continue
        x = [99*math.tan(math.radians(r['pitch_cdeg']/100))/
             math.cos(math.radians(r['roll_cdeg']/100)) for r in group]
        errors = [(r['target_pitch_cdeg']-r['pitch_cdeg'])/100 for r in group]
        result.append({'reference_cycle':index, 'start_run_ms':start_ms,
                       'end_run_ms':end_ms, 'rows':len(group),
                       'ground_span_cm':max(x)-min(x),
                       'length_error_cm':max(x)-min(x)-length/10,
                       'left_endpoint_cm':min(x), 'right_endpoint_cm':max(x),
                       'center_cm':(max(x)+min(x))/2,
                       'pitch_rmse_deg':math.sqrt(statistics.mean(e*e for e in errors)),
                       'pitch_phase_lag_ms':pitch_phase_lag_ms(group, period)})
    return result


def analyze(rows, capture_error=None, historical_pwm_limit=None):
    active = [r for r in rows if r['q1_state'] == 1]
    amplitude_mode = any(r.get('q1_control_phase', 0) > 0 for r in rows)
    first_active = next((i for i, row in enumerate(rows) if row['q1_state'] == 1), None)
    pid_trial = any(r['mode'] == 8 or r.get('problem_id') == 2 or
                    (r.get('problem_id') == 1 and (r.get('reference_period_ms', 0) > 0 or
                     any(r.get(f'pitch_k{term}_milli', 0) for term in 'pid')))
                    for r in rows)
    restarts = [{'previous_ms': a['time_ms'], 'current_ms': b['time_ms']}
                for a, b in zip(rows, rows[1:])
                if board_time_restarted(a['time_ms'], b['time_ms'])]
    terminal = first_terminal_index(rows, first_active) if first_active is not None else None
    first_stop = None
    if terminal is not None:
        stop_row = rows[terminal]
        first_stop = {
            'run_ms': stop_row['q1_run_ms'],
            'state': stop_row['q1_state'],
            'reason': REASONS[stop_row['q1_reason']],
            'pitch_deg': stop_row['pitch_cdeg'] / 100,
            'roll_deg': stop_row['roll_cdeg'] / 100,
            'pitch_rate_dps': stop_row['pitch_rate_cdeg_s'] / 100,
        }
    terminal_observed = first_stop is not None
    result = {'rows': len(rows), 'active_rows': len(active), 'full_sampling_rate': False,
              'mcu_time_restarts': len(restarts), 'time_restart_events': restarts,
              'terminal_state_observed': terminal_observed,
              'first_stop': first_stop,
              'controller': ('q2_trajectory_pid' if any(r['mode'] == 8 or
                  r.get('problem_id') == 2 for r in rows) else
                  'q1_two_phase_amplitude' if amplitude_mode else
                  'q1_reference_pid' if pid_trial else 'q1_legacy_endpoint'),
              'switch_count_meaning': ('applied_pwm_polarity_changes'
                  if pid_trial else 'legacy_endpoint_decisions'),
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
            'note': 'Angle-derived estimates only; a ruler/ground trace must verify actual length and linearity. ' +
                    ('Q2 tracks a smooth angle trajectory using PID. '
                     if result['controller'] == 'q2_trajectory_pid' else
                     'Q1 uses startup reference PID, then amplitude feedback; the fixed-coordinate target is telemetry only after transition. '
                     if result['controller'] == 'q1_two_phase_amplitude' else
                     'Q1 follows a fixed-coordinate reference with rate PID. '
                     if pid_trial else
                     'Q1 uses the legacy endpoint switching and turn/coast thresholds. ') +
                    'Left motor PWM1 moves the pendulum right; right motor PWM2 moves it left.'})
        if pid_trial:
            errors = [(r['target_pitch_cdeg'] - r['pitch_cdeg']) / 100 for r in active]
            result['pitch_tracking_rmse_deg'] = math.sqrt(statistics.mean(e*e for e in errors))
            result['max_pitch_tracking_error_deg'] = max(map(abs, errors))
            stop_ms = first_stop['run_ms'] if first_stop is not None else None
            result['reference_cycles'] = reference_cycle_metrics(active, stop_ms)
            periods = {r['reference_period_ms'] for r in active}
            ramps = {r['reference_ramp_ms'] for r in active}
            result['target_pitch_period_ms'] = next(iter(periods)) if len(periods) == 1 and 0 not in periods else None
            result.update(pitch_period(active, max(ramps)))
            saturation = pwm_saturation(active, historical_pwm_limit)
            result['pwm_saturation'] = saturation
            limit = saturation['limit_permille']
            result['pitch_output_saturated_rows'] = (
                sum(abs(r['pid_pitch_milli']) >= limit * 1000 for r in active)
                if limit is not None else None)
            result['pitch_integrator_enabled_rows'] = sum(r['pitch_integrating'] == 1 for r in active)
            gaps = [((b['time_ms']-a['time_ms']) & 0xFFFFFFFF) for a,b in zip(active,active[1:])]
            if gaps:
                result['median_telemetry_interval_ms'] = statistics.median(gaps)
                result['max_telemetry_interval_ms'] = max(gaps)
            result['pid_configurations'] = [dict(zip(
                ['kp','ki','kd','period_ms','ramp_ms','target_length_mm'], values))
                for values in sorted({(r['pitch_kp_milli']/1000,r['pitch_ki_milli']/1000,
                    r['pitch_kd_milli']/1000,r['reference_period_ms'],r['reference_ramp_ms'],
                    r['target_length_mm']) for r in active})]
        if amplitude_mode:
            amplitude_rows = [r for r in active if r.get('q1_control_phase', 0) > 0]
            phases = ('startup_pid', 'blend', 'amplitude')
            result['q1_control_phase_rows'] = {
                phases[phase]: sum(r.get('q1_control_phase', 0) == phase for r in active)
                for phase in range(len(phases))}
            startup_rows = [r for r in active if r.get('q1_control_phase', 0) == 0]
            if startup_rows:
                startup_errors = [(r['target_pitch_cdeg'] - r['pitch_cdeg']) / 100
                                  for r in startup_rows]
                result['q1_startup_pid_rmse_deg'] = math.sqrt(
                    statistics.mean(error * error for error in startup_errors))
            if amplitude_rows:
                estimates = [r['q1_amplitude_cdeg'] / 100 for r in amplitude_rows]
                errors = [r['q1_amplitude_error_cdeg'] / 100 for r in amplitude_rows]
                result['amplitude_control'] = {
                    'target_amplitude_deg': statistics.median(
                        r['q1_amplitude_target_cdeg'] / 100 for r in amplitude_rows),
                    'first_run_ms': amplitude_rows[0]['q1_run_ms'],
                    'last_run_ms': amplitude_rows[-1]['q1_run_ms'],
                    'estimate_range_deg': [min(estimates), max(estimates)],
                    'mean_absolute_error_deg': statistics.mean(map(abs, errors)),
                    'max_absolute_output_permille': max(
                        abs(r['q1_amplitude_output_milli']) / 1000
                        for r in amplitude_rows),
                    'ka': statistics.median(
                        r['q1_amplitude_ka_milli'] / 1000 for r in amplitude_rows),
                    'kcomp': statistics.median(
                        r['q1_amplitude_kcomp_milli'] / 1000 for r in amplitude_rows),
                }
    return result


def export_report(directory, rows, report, plot):
    (directory / 'analysis.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    if plot and 'pitch_tracking_rmse_deg' in report:
        save_curves(directory / 'control_curves.png', rows, report)


def report_session(directory, historical_pwm_limit=None):
    metadata = json.loads((directory / 'session.json').read_text(encoding='utf-8-sig'))
    with (directory / 'telemetry.csv').open(encoding='utf-8-sig', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        fields = set(reader.fieldnames or ())
        if fields.issuperset(HEADER):
            names = HEADER
        elif fields.issuperset(PWM_HEADER):
            names = PWM_HEADER
        elif fields.issuperset(PID_HEADER):
            names = PID_HEADER
        elif fields.issuperset(LEGACY_HEADER):
            names = LEGACY_HEADER
        else:
            raise ValueError('Unrecognized telemetry CSV header')
        rows = [parse_line(','.join(row[name] for name in names).encode('ascii')) for row in reader]
    report = analyze(rows, capture_error=metadata.get('error'),
                     historical_pwm_limit=historical_pwm_limit)
    export_report(directory, rows, report, plot=True)
    return report


def capture(args):
    problem = getattr(args, 'problem', 'q1')
    length_mm = getattr(args, 'length_cm', 60) * 10
    parent = ROOT / '数据记录/串口'
    if problem == 'q2': parent /= 'Q2'
    directory = parent / dt.datetime.now().strftime('%Y%m%d_%H%M%S_COM9_' + problem + '_' + args.action +
                                                    ('_L%dcm'% (length_mm//10) if problem == 'q2' else ''))
    directory.mkdir(parents=True)
    if problem == 'q2': write_q2_fields(directory)
    if args.action == 'stop':
        mailbox('stop', directory)
    port = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    port.dtr = port.rts = False
    port.port = args.port
    rows, pending, invalid, sent = [], b'', 0, False
    started = time.monotonic()
    last_telemetry_at = started
    error = None
    metadata = {'action': args.action, 'problem': problem,
                'requested_length_mm': length_mm if problem == 'q2' else None,
                'port': args.port, 'serial_write': False,
                'host_sequence_is_not_mcu_sample_sequence': True}
    manifest_path = ROOT / 'build/workflow/last-run.json'
    if manifest_path.exists():
        metadata['local_firmware_manifest'] = json.loads(manifest_path.read_text(encoding='utf-8-sig'))
    print('Session: ' + str(directory), flush=True)
    try:
        port.open()
        with (directory / 'raw.bin').open('wb') as raw, (directory / 'telemetry.csv').open('w', encoding='utf-8-sig', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=HEADER + Q2_EXTRA_FIELDS if problem == 'q2' else HEADER)
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
                        writer.writerow(q2_csv_row(row, length_mm, received_at-started) if problem == 'q2' else row)
                        last_telemetry_at = received_at
                        if sent and problem == 'q2' and row['q1_state'] == 1 and (
                            row['problem_id'] != 2 or row['target_length_mm'] != length_mm):
                            raise RuntimeError('Q2 task/length acknowledgement mismatch; trial aborted')
                        if args.action == 'start' and previous_ms is not None and \
                                board_time_restarted(previous_ms, row['time_ms']):
                            raise RuntimeError('MCU time restarted: %d -> %d ms; trial aborted' %
                                               (previous_ms, row['time_ms']))
                if sent and received_at - last_telemetry_at > 0.5:
                    raise RuntimeError('Telemetry lost for more than 0.5 s; trial aborted')
                if args.action == 'start' and not sent and received_at - started >= 1.0:
                    recent = rows[-20:]
                    allowed_modes = (2, 7) if problem == 'q2' else (2,)
                    if len(recent) < 20 or received_at - last_telemetry_at > 0.5 or any(r['mode'] not in allowed_modes or r['q1_state'] == 1 or
                        abs(r['pitch_cdeg']) > 700 or abs(r['roll_cdeg']) > 700 or
                        r['keys'] & 4 or
                        r['en_command'] != 0 or r['en_pin'] != 0 or
                        any(r['pwm%d_applied_permille' % i] for i in range(1,5))
                        for r in recent):
                        raise RuntimeError('Pre-start check failed: require READY within +/-7 degrees, zero PWM and EN off')
                    # If the write succeeds but logging fails, ensure finally still stops.
                    sent = True
                    if problem == 'q2':
                        mailbox('start', directory, problem='q2', length_mm=length_mm)
                    else:
                        mailbox('start', directory)
                    print('One start command sent; waiting for the board to stop.', flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__
    finally:
        port.close()
        if sent:
            try:
                mailbox('stop', directory)
            except Exception as exc:
                error = (error or '') + '; final stop failed: ' + str(exc)
    duration_s = time.monotonic() - started
    report = analyze(rows, capture_error=error)
    if args.action == 'start' or problem == 'q2':
        try:
            export_report(directory, rows, report, plot=args.action == 'start')
        except (ImportError, OSError, ValueError, RuntimeError) as exc:
            metadata['report_error'] = str(exc)
    if not error and not rows:
        error = 'No valid telemetry'
    if not error and sent and not any(r['q1_state'] == 1 for r in rows):
        error = 'Start was sent but RUNNING was not observed'
    metadata.update(start_sent=sent, invalid_or_partial_lines=invalid, error=error,
                    duration_s=duration_s, trailing_bytes=len(pending), analysis=report)
    (directory / 'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=['observe', 'start', 'stop'], default='observe')
    parser.add_argument('--port', default='COM9')
    parser.add_argument('--duration', type=float,
                        help='Capture seconds (default: 30 for Q1 start, 20 otherwise)')
    parser.add_argument('--problem', choices=['q1', 'q2'], default='q1')
    parser.add_argument('--length-cm', type=int, default=60, help='Q2 total trace length, 30..60 cm')
    parser.add_argument('--report-session', type=Path, help='Analyze a saved session without using hardware')
    parser.add_argument('--historical-pwm-limit', type=int,
                        help='Known PWM limit in permille for a saved session that did not report it')
    args = parser.parse_args()
    if args.historical_pwm_limit is not None:
        if args.report_session is None or not 1 <= args.historical_pwm_limit <= 1000:
            parser.error('--historical-pwm-limit requires --report-session and a limit from 1 to 1000')
    if args.report_session is not None:
        try:
            result = report_session(args.report_session, args.historical_pwm_limit)
        except (ImportError, OSError, ValueError, RuntimeError) as exc:
            parser.error(str(exc))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        raise SystemExit(0)
    q1_start = args.action == 'start' and args.problem == 'q1'
    if args.duration is None:
        args.duration = 30.0 if q1_start else 20.0
    if not 0 < args.duration <= 60:
        parser.error('duration must be greater than 0 and at most 60 s')
    minimum_start_duration = 28 if q1_start else 18
    if args.action == 'start' and args.duration < minimum_start_duration:
        parser.error(f'{args.problem.upper()} start needs at least {minimum_start_duration} s of capture')
    if args.problem == 'q2' and not 30 <= args.length_cm <= 60:
        parser.error('Q2 length must be 30..60 cm')
    capture(args)
