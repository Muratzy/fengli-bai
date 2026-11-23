#!/usr/bin/env python3
"""Collect static laser-ground poses and fit the IMU sensor-to-body frame."""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np
import serial

import q1_trial

ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_ROOT = ROOT / '数据记录' / '标定' / '激光'
GYRO_BIAS_DPS = np.array([0.31284, -0.08947, 0.12095], dtype=float)
ACCEL_G_PER_COUNT = 16.0 / 32768.0
GYRO_DPS_PER_COUNT = 2000.0 / 32768.0


def evaluate_pose_stability(accel_samples, gyro_samples):
    accel = np.asarray(accel_samples, dtype=float) * ACCEL_G_PER_COUNT
    gyro = np.asarray(gyro_samples, dtype=float) * GYRO_DPS_PER_COUNT - GYRO_BIAS_DPS
    accel_norms = np.linalg.norm(accel, axis=1)
    unit = accel / accel_norms[:, None]
    mean_unit = unit.mean(axis=0)
    mean_unit /= np.linalg.norm(mean_unit)
    scatter_deg = np.degrees(np.arccos(np.clip(unit @ mean_unit, -1.0, 1.0)))
    gyro_rms = float(np.sqrt(np.mean(np.sum(gyro**2, axis=1))))
    metrics = {
        'accel_norm_g_mean': float(np.mean(accel_norms)),
        'accel_direction_rms_deg': float(np.sqrt(np.mean(scatter_deg**2))),
        'accel_direction_max_deg': float(np.max(scatter_deg)),
        'gyro_bias_corrected_rms_dps': gyro_rms,
    }
    metrics['stable_pose'] = metrics['accel_direction_rms_deg'] <= 1.5 and \
        gyro_rms <= 5.0 and 0.8 <= metrics['accel_norm_g_mean'] <= 1.2
    return metrics


def body_gravity_from_spot(x_cm, y_cm, height_cm):
    if not all(math.isfinite(v) for v in (x_cm, y_cm, height_cm)) or height_cm <= 0:
        raise ValueError('Spot coordinates and positive height must be finite')
    roll = math.atan2(y_cm, height_cm)
    pitch = math.atan2(x_cm * math.cos(roll), height_cm)
    return np.array([math.sin(pitch), math.cos(pitch) * math.sin(roll),
                     math.cos(pitch) * math.cos(roll)])


def fit_rotation(sensor_vectors, body_vectors):
    sensor = np.asarray(sensor_vectors, dtype=float)
    body = np.asarray(body_vectors, dtype=float)
    if sensor.shape != body.shape or sensor.ndim != 2 or sensor.shape[1] != 3 or len(sensor) < 4:
        raise ValueError('At least four paired 3D gravity vectors are required')
    if not np.isfinite(sensor).all() or not np.isfinite(body).all():
        raise ValueError('Calibration vectors must be finite')
    if np.linalg.matrix_rank(sensor - sensor.mean(axis=0)) < 2:
        raise ValueError('Laser poses must span both tilt axes')
    covariance = body.T @ sensor
    u, singular, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    rotation = u @ correction @ vt
    predicted = (rotation @ sensor.T).T
    errors = np.degrees(np.arccos(np.clip(np.sum(predicted * body, axis=1), -1.0, 1.0)))
    return rotation, errors, singular


def accelerometer_angles(sensor_g, rotation):
    body = np.asarray(rotation, dtype=float) @ np.asarray(sensor_g, dtype=float)
    roll = math.degrees(math.atan2(body[1], body[2]))
    pitch = math.degrees(math.atan2(body[0], math.hypot(body[1], body[2])))
    return roll, pitch


def zero_and_check(center_sensor_g, reference_sensor_g, x_cm, y_cm, height_cm, rotation):
    roll_zero, pitch_zero = accelerometer_angles(center_sensor_g, rotation)
    ref_roll, ref_pitch = accelerometer_angles(reference_sensor_g, rotation)
    expected_roll = math.degrees(math.atan2(y_cm, height_cm))
    expected_pitch = math.degrees(math.atan2(x_cm * math.cos(math.radians(expected_roll)), height_cm))
    measured_pitch_delta = ref_pitch - pitch_zero
    error = measured_pitch_delta - expected_pitch
    gain = expected_pitch / measured_pitch_delta if abs(measured_pitch_delta) > 0.1 else None
    return {'roll_zero_deg': roll_zero, 'pitch_zero_deg': pitch_zero,
        'reference_raw_roll_deg': ref_roll, 'reference_raw_pitch_deg': ref_pitch,
        'reference_corrected_roll_deg': ref_roll-roll_zero,
        'reference_corrected_pitch_deg': measured_pitch_delta,
        'reference_laser_pitch_deg': expected_pitch,
        'reference_pitch_error_deg': error,
        'diagnostic_pitch_scale_from_single_point': gain,
        'apply_pitch_scale': 1.0 if abs(error) <= 0.25 else gain,
        'within_manual_point_tolerance': abs(error) <= 0.25,
        'scope':'Center zero plus one-axis pitch check only; does not refit the full 3D rotation or cross-axis alignment.'}


def capture_pose(args):
    if not args.label.strip() or not (math.isfinite(args.x_cm) and math.isfinite(args.y_cm)):
        raise ValueError('Pose label and finite ground coordinates are required')
    CALIBRATION_ROOT.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S_COM9_laser_' + args.label)
    directory = CALIBRATION_ROOT / stamp
    directory.mkdir()
    port = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    port.dtr = port.rts = False
    port.port = args.port
    samples, pending, invalid = [], b'', 0
    started = time.monotonic()
    metadata = {'label': args.label, 'x_cm': args.x_cm, 'y_cm': args.y_cm,
        'height_cm': args.height_cm, 'duration_s_requested': args.duration,
        'port': args.port, 'receive_only': True, 'start_or_stop_command_sent': False,
        'full_sampling_rate_claimed': False}
    print('Keep the laser spot on its measured cross mark and hold still: ' + str(directory), flush=True)
    try:
        port.open()
        with (directory/'raw.bin').open('wb') as raw:
            while time.monotonic() - started < args.duration:
                chunk = port.read(max(1, min(port.in_waiting, 4096)))
                raw.write(chunk)
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    try:
                        row = q1_trial.parse_line(line)
                    except (ValueError, UnicodeError):
                        invalid += 1
                        continue
                    if row is None:
                        continue
                    if row['mode'] != 2 or row['q1_state'] != 0 or row['en_command'] != 0 or \
                            row['en_pin'] != 0 or any(row['pwm%d_applied_permille'%i] for i in range(1, 5)):
                        raise RuntimeError('Pose capture requires READY/IDLE, all PWM zero and EN off')
                    samples.append(row)
    finally:
        port.close()
    if len(samples) < 50:
        raise RuntimeError('Only %d stable telemetry rows; need at least 50' % len(samples))
    accel = np.array([[r['ax_raw'], r['ay_raw'], r['az_raw']] for r in samples], dtype=float)
    gyro = np.array([[r['gx_raw'], r['gy_raw'], r['gz_raw']] for r in samples], dtype=float)
    accel_vectors = accel * ACCEL_G_PER_COUNT
    quality = evaluate_pose_stability(accel, gyro)
    metadata.update({
        'rows': len(samples), 'invalid_or_partial_lines': invalid,
        'duration_s_actual': time.monotonic() - started,
        'board_time_ms': [samples[0]['time_ms'], samples[-1]['time_ms']],
        'board_interval_ms_median': statistics.median(b['time_ms']-a['time_ms'] for a,b in zip(samples,samples[1:])),
        'mean_accel_sensor_g': (accel_vectors.mean(axis=0)).tolist(),
    })
    metadata.update(quality)
    with (directory/'telemetry.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=q1_trial.HEADER)
        writer.writeheader()
        writer.writerows(samples)
    (directory/'pose.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    if not metadata['stable_pose']:
        raise SystemExit('Pose rejected as unstable; keep the spot fixed and recapture.')


def summarize_capture(rows):
    active = [r for r in rows if r['mode'] == 9 and r['q1_state'] == 1]
    completed_points = max((r.get('q1_switches', 0) for r in rows if r['mode'] == 9), default=0)
    return {
        'rows': len(rows),
        'valid_capture_rows': len(active),
        'outside_capture_rows': len(rows) - len(active),
        'completed_points': completed_points,
        'all_three_points_complete': completed_points >= 3,
        'board_time_ms': [rows[0]['time_ms'], rows[-1]['time_ms']] if rows else None,
    }


def capture_series(args):
    CALIBRATION_ROOT.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S_COM9_laser_remaining')
    directory = CALIBRATION_ROOT / stamp
    directory.mkdir()
    port = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    port.dtr = port.rts = False
    port.port = args.port
    rows, pending, invalid = [], b'', 0
    started = time.monotonic()
    completed = False
    metadata = {'port': args.port, 'duration_s_requested': args.duration,
        'serial_write': False, 'full_sampling_rate_claimed': False,
        'valid_flag': 'mode == 9 and q1_state == 1',
        'point_completion_flag': 'mode == 9 and q1_switches increments on KEY2'}
    print('Session: ' + str(directory), flush=True)
    try:
        port.open()
        print('Capture ready. For each remaining 20 cm point clockwise: press KEY1 to record, KEY2 to finish. Repeat three times.', flush=True)
        with (directory/'raw.bin').open('wb') as raw, \
             (directory/'telemetry.csv').open('w', encoding='utf-8-sig', newline='') as output:
            writer = csv.DictWriter(output, fieldnames=q1_trial.HEADER)
            writer.writeheader()
            while time.monotonic() - started < args.duration:
                chunk = port.read(max(1, min(port.in_waiting, 4096)))
                raw.write(chunk)
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    try:
                        row = q1_trial.parse_line(line)
                    except (ValueError, UnicodeError):
                        invalid += 1
                        continue
                    if row is None:
                        continue
                    rows.append(row)
                    writer.writerow(row)
                    if row['mode'] == 9 and row['q1_state'] == 0 and row['q1_switches'] >= 3:
                        completed = True
                        break
                if completed:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        port.close()

    report = summarize_capture(rows)
    metadata.update(report, invalid_or_partial_lines=invalid,
        duration_s=time.monotonic()-started, trailing_bytes=len(pending),
        capture_complete=completed)
    (directory/'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    print('Saved: ' + str(directory))


def analyze_series(args):
    source = Path(args.series_dir).resolve()
    with (source / 'telemetry.csv').open(encoding='utf-8-sig', newline='') as f:
        rows = [{key: int(value) for key, value in row.items()}
                for row in csv.DictReader(f)]
    if not rows or max((row.get('q1_switches', 0) for row in rows), default=0) < 3:
        raise ValueError('Series must contain all three completed points')

    specs = [('down', 0.0, -20.0), ('left', -20.0, 0.0), ('up', 0.0, 20.0)]
    points = []
    segments_dir = source / 'segments'
    segments_dir.mkdir(exist_ok=True)
    for index, (label, x_cm, y_cm) in enumerate(specs):
        segment = [row for row in rows if row['mode'] == 9 and
                   row['q1_state'] == 1 and row.get('q1_switches', 0) == index]
        if len(segment) < 50:
            raise ValueError('%s segment has too few valid rows: %d' %
                             (label, len(segment)))
        raw = np.array([[row['ax_raw'], row['ay_raw'], row['az_raw']]
                        for row in segment], dtype=float)
        gyro = np.array([[row['gx_raw'], row['gy_raw'], row['gz_raw']]
                         for row in segment], dtype=float)
        median_sensor = np.median(raw, axis=0) * ACCEL_G_PER_COUNT
        median_sensor /= np.linalg.norm(median_sensor)
        quality = evaluate_pose_stability(raw, gyro)
        segment_dir = segments_dir / ('%02d_%s' % (index + 1, label))
        segment_dir.mkdir(exist_ok=True)
        with (segment_dir / 'telemetry.csv').open('w', encoding='utf-8-sig',
                                                   newline='') as output:
            writer = csv.DictWriter(output, fieldnames=q1_trial.HEADER)
            writer.writeheader()
            writer.writerows(segment)
        pose = {
            'label': label, 'x_cm': x_cm, 'y_cm': y_cm,
            'height_cm': args.height_cm, 'source_series': str(source),
            'q1_switches': index, 'rows': len(segment),
            'board_time_ms': [segment[0]['time_ms'], segment[-1]['time_ms']],
            'median_sensor_g': median_sensor.tolist(),
            'mean_sensor_g': (raw.mean(axis=0) * ACCEL_G_PER_COUNT /
                              np.linalg.norm(raw.mean(axis=0) * ACCEL_G_PER_COUNT)).tolist(),
        }
        pose.update(quality)
        (segment_dir / 'pose.json').write_text(
            json.dumps(pose, ensure_ascii=False, indent=2), encoding='utf-8')
        points.append({'label': label, 'x_cm': x_cm, 'y_cm': y_cm,
                       'sensor': median_sensor, 'rows': len(segment),
                       'source': str(segment_dir.relative_to(ROOT)),
                       'quality': quality})

    center_pose, center_sensor = capture_mean_sensor_g(args.center_dir)
    right_pose, right_sensor = capture_mean_sensor_g(args.right_dir)
    measured = [
        {'label': 'center', 'x_cm': float(center_pose['x_cm']),
         'y_cm': float(center_pose['y_cm']), 'sensor': center_sensor,
         'rows': int(center_pose['rows']), 'source': str(Path(args.center_dir).resolve().relative_to(ROOT))},
        {'label': 'right', 'x_cm': float(right_pose['x_cm']),
         'y_cm': float(right_pose['y_cm']), 'sensor': right_sensor,
         'rows': int(right_pose['rows']), 'source': str(Path(args.right_dir).resolve().relative_to(ROOT))},
    ] + points
    sensor = np.array([point['sensor'] for point in measured])
    body = np.array([body_gravity_from_spot(point['x_cm'], point['y_cm'],
                                             args.height_cm) for point in measured])
    rotation, residuals, singular = fit_rotation(sensor, body)
    zero_roll, zero_pitch = accelerometer_angles(center_sensor, rotation)

    def pose_result(point, residual):
        body_vector = rotation @ point['sensor']
        raw_roll = math.degrees(math.atan2(body_vector[1], body_vector[2]))
        raw_pitch = math.degrees(math.atan2(
            body_vector[0], math.hypot(body_vector[1], body_vector[2])))
        roll = raw_roll - zero_roll
        pitch = raw_pitch - zero_pitch
        roll_rad, pitch_rad = math.radians(roll), math.radians(pitch)
        right_component = math.sin(pitch_rad)
        up_component = math.cos(pitch_rad) * math.sin(roll_rad)
        normal_component = math.cos(pitch_rad) * math.cos(roll_rad)
        return {
            'label': point['label'], 'x_cm': point['x_cm'], 'y_cm': point['y_cm'],
            'rows': point['rows'], 'source': point['source'],
            'fit_residual_deg': float(residual),
            'raw_roll_deg': raw_roll, 'raw_pitch_deg': raw_pitch,
            'roll_deg': roll, 'pitch_deg': pitch,
            'tilt_deg': math.degrees(math.atan2(
                math.hypot(right_component, up_component), normal_component)),
            'direction_deg': (math.degrees(math.atan2(right_component,
                                                        up_component)) + 360.0) % 360.0,
        }

    results = [pose_result(point, residual)
               for point, residual in zip(measured, residuals)]
    report = {
        'source_series': str(source), 'height_cm': args.height_cm,
        'geometry': 'laser spot coordinates are right-positive/up-positive; clockwise direction',
        'rotation_sensor_to_body_rows': ['right', 'up', 'center gravity'],
        'matrix': rotation.tolist(), 'determinant': float(np.linalg.det(rotation)),
        'orthogonality_error_max': float(np.max(np.abs(rotation @ rotation.T - np.eye(3)))),
        'fit_rms_deg': float(np.sqrt(np.mean(residuals ** 2))),
        'fit_max_deg': float(np.max(residuals)),
        'singular_values': singular.tolist(),
        'zero_offsets_deg': {'roll': zero_roll, 'pitch': zero_pitch},
        'gyro_bias_preserved_dps': GYRO_BIAS_DPS.tolist(),
        'poses': results,
        'acceptance_note': 'Five measured laser poses fitted; gyro zero bias is unchanged. Verify by a center readback before flashing.',
    }
    (source / 'laser_frame_fit.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    names = ['APP_IMU_FRAME_RIGHT', 'APP_IMU_FRAME_UP', 'APP_IMU_FRAME_NORMAL']
    lines = [('#define %s {%s}' %
              (name, ', '.join('%.8fF' % value for value in row)))
             for name, row in zip(names, rotation)]
    lines += ['#define APP_IMU_ROLL_ZERO_DEG %.8fF' % zero_roll,
              '#define APP_IMU_PITCH_ZERO_DEG %.8fF' % zero_pitch]
    (source / 'candidate_app_config.txt').write_text('\n'.join(lines) + '\n',
                                                     encoding='ascii')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('Saved split segments and fit report under: ' + str(source))


def fit_poses(args):
    source = Path(args.directory).resolve()
    poses = []
    for pose_file in sorted(source.glob('*/pose.json')):
        pose = json.loads(pose_file.read_text(encoding='utf-8'))
        if not pose.get('stable_pose'):
            continue
        with (pose_file.parent/'telemetry.csv').open(encoding='utf-8-sig', newline='') as f:
            rows = list(csv.DictReader(f))
        if len(rows) < 50:
            continue
        raw = np.array([[float(r['ax_raw']), float(r['ay_raw']), float(r['az_raw'])]
                        for r in rows], dtype=float)
        mean = raw.mean(axis=0) * ACCEL_G_PER_COUNT
        norm = np.linalg.norm(mean)
        if not 0.8 <= norm <= 1.2:
            continue
        poses.append({'label':pose['label'], 'x_cm':pose['x_cm'], 'y_cm':pose['y_cm'],
                      'mean_sensor_g':(mean/norm).tolist(), 'rows':len(rows),
                      'source':str(pose_file.parent.relative_to(ROOT))})
    if len(poses) < 5 or not any(p['label'] == 'center' and abs(p['x_cm']) < 0.1 and abs(p['y_cm']) < 0.1 for p in poses):
        raise ValueError('Need stable center plus at least four measured laser poses')
    positions = [(p['x_cm'], p['y_cm']) for p in poses]
    if len(set(positions)) < 5 or not (any(x > 1 for x,y in positions) and any(x < -1 for x,y in positions)
            and any(y > 1 for x,y in positions) and any(y < -1 for x,y in positions)):
        raise ValueError('Need distinct laser marks on left, right, up and down axes')
    sensor = np.array([p['mean_sensor_g'] for p in poses])
    body = np.array([body_gravity_from_spot(p['x_cm'], p['y_cm'], args.height_cm) for p in poses])
    rotation, errors, singular = fit_rotation(sensor, body)
    report = {'height_cm':args.height_cm, 'rotation_sensor_to_body_rows':['right','up','center gravity'],
        'matrix':rotation.tolist(), 'determinant':float(np.linalg.det(rotation)),
        'orthogonality_error_max':float(np.max(np.abs(rotation@rotation.T-np.eye(3)))),
        'fit_rms_deg':float(np.sqrt(np.mean(errors**2))), 'fit_max_deg':float(np.max(errors)),
        'singular_values':singular.tolist(), 'poses':poses,
        'pose_residuals_deg':{p['label']:float(e) for p,e in zip(poses,errors)},
        'gyro_bias_preserved_dps':GYRO_BIAS_DPS.tolist(),
        'acceptance_note':'Fit is a candidate sensor-to-body rotation from the measured laser geometry. Verify center and cross-axis poses before flashing; it does not recalibrate gyro zero bias.'}
    (source/'laser_frame_fit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    lines=[]
    names=['APP_IMU_FRAME_RIGHT','APP_IMU_FRAME_UP','APP_IMU_FRAME_NORMAL']
    for name,row in zip(names,rotation):
        lines.append('#define %s {%s}'%(name,', '.join('%.8fF'%v for v in row)))
    (source/'candidate_app_config.txt').write_text('\n'.join(lines)+'\n',encoding='ascii')
    print(json.dumps({k:v for k,v in report.items() if k!='poses'},ensure_ascii=False,indent=2))
    print('Candidate definitions saved to: '+str(source/'candidate_app_config.txt'))


def capture_mean_sensor_g(directory):
    pose_file=Path(directory)/'pose.json'
    pose=json.loads(pose_file.read_text(encoding='utf-8'))
    with (Path(directory)/'telemetry.csv').open(encoding='utf-8-sig',newline='') as f:
        rows=list(csv.DictReader(f))
    if len(rows)<50: raise ValueError('Pose needs at least 50 telemetry rows')
    raw=np.array([[float(r[k]) for k in ('ax_raw','ay_raw','az_raw')] for r in rows],dtype=float)
    median=np.median(raw,axis=0)*ACCEL_G_PER_COUNT
    norm=np.linalg.norm(median)
    if not .8<=norm<=1.2: raise ValueError('Pose gravity magnitude is outside 0.8..1.2 g')
    return pose,median/norm


def fit_zero(args):
    import re
    config=(ROOT/'Core'/'Inc'/'app_config.h').read_text(encoding='utf-8')
    matrix=[]
    for name in ('APP_IMU_FRAME_RIGHT','APP_IMU_FRAME_UP','APP_IMU_FRAME_NORMAL'):
        match=re.search(r'^#define\s+'+name+r'\s+\{([^}]+)\}',config,re.M)
        if not match: raise ValueError('Cannot parse '+name)
        matrix.append([float(v.strip().rstrip('Ff')) for v in match.group(1).split(',')])
    rotation=np.asarray(matrix,dtype=float)
    center,center_g=capture_mean_sensor_g(args.center_dir)
    reference,reference_g=capture_mean_sensor_g(args.reference_dir)
    if center['label']!='center' or abs(center['x_cm'])>.1 or abs(center['y_cm'])>.1:
        raise ValueError('Center dataset must be labeled center at x=y=0')
    result=zero_and_check(center_g,reference_g,args.x_cm,args.y_cm,args.height_cm,rotation)
    result.update({'height_cm':args.height_cm,'reference_label':reference['label'],
        'reference_x_cm':args.x_cm,'reference_y_cm':args.y_cm,
        'center_dataset':str(Path(args.center_dir).resolve()),
        'reference_dataset':str(Path(args.reference_dir).resolve()),
        'sensor_to_body_rotation_unchanged':True,'gyro_bias_unchanged_dps':GYRO_BIAS_DPS.tolist()})
    output=CALIBRATION_ROOT/'laser_zero_point_calibration.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    print('Saved calibration result: '+str(output))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    capture=sub.add_parser('capture',help='Receive one stationary laser-mark pose; sends no MCU command')
    capture.add_argument('--label',required=True,help='center/right/left/up/down')
    capture.add_argument('--x-cm',type=float,required=True,help='measured right-positive ground coordinate')
    capture.add_argument('--y-cm',type=float,required=True,help='measured up-positive ground coordinate')
    capture.add_argument('--height-cm',type=float,default=99.0,help='pivot vertical height to laser ground plane')
    capture.add_argument('--duration',type=float,default=8.0)
    capture.add_argument('--port',default='COM9')
    series=sub.add_parser('capture-series',help='Record one K1-to-K2 calibration session')
    series.add_argument('--duration',type=float,default=300.0)
    series.add_argument('--port',default='COM9')
    analysis=sub.add_parser('analyze-series',help='Split three points and fit the laser frame')
    analysis.add_argument('--series-dir',required=True)
    analysis.add_argument('--center-dir',required=True)
    analysis.add_argument('--right-dir',required=True)
    analysis.add_argument('--height-cm',type=float,default=99.0)
    fit=sub.add_parser('fit',help='Fit proper rotation from all accepted labeled poses')
    fit.add_argument('--directory',default=str(CALIBRATION_ROOT))
    fit.add_argument('--height-cm',type=float,default=99.0)
    zero=sub.add_parser('fit-zero',help='Fit center zero and validate one known laser point')
    zero.add_argument('--center-dir',required=True)
    zero.add_argument('--reference-dir',required=True)
    zero.add_argument('--x-cm',type=float,required=True)
    zero.add_argument('--y-cm',type=float,required=True)
    zero.add_argument('--height-cm',type=float,default=99.0)
    args=parser.parse_args()
    if args.action=='capture': capture_pose(args)
    elif args.action=='capture-series': capture_series(args)
    elif args.action=='analyze-series': analyze_series(args)
    elif args.action=='fit-zero': fit_zero(args)
    else: fit_poses(args)


if __name__=='__main__':
    main()
