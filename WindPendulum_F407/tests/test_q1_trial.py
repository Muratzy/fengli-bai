import argparse
import contextlib
import importlib.util
import io
import json
import pathlib
import tempfile
import unittest
from unittest import mock


SPEC = importlib.util.spec_from_file_location('q1_trial', pathlib.Path(__file__).resolve().parents[1] / 'Tools/q1_trial.py')
trial = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(trial)


def make_row(**updates):
    row = dict.fromkeys(trial.HEADER, 0)
    row.update(mode=2, time_ms=3000)
    row.update(updates)
    return row


class TrialTests(unittest.TestCase):
    def test_q2_command_script_validates_length_and_keeps_stop_global(self):
        command = 0x2000025C
        length = 0x20000260
        script = trial.command_script('start', command, 'q2', length, 400)
        self.assertEqual(script.splitlines(), [
            'w4 0x20000260, 0x00000190', 'w4 0x2000025C, 0x5132474F', 'exit'])
        self.assertIn('0x00000258', trial.command_script('start', command, 'q2', length, 600))
        self.assertEqual(trial.command_script('stop', command, 'q2').splitlines(),
                         ['w4 0x2000025C, 0x51315354', 'exit'])
        self.assertIn('0x5131474F', trial.command_script('start', command))
        for invalid in (299, 601, 400.5):
            with self.assertRaises(ValueError):
                trial.command_script('start', command, 'q2', length, invalid)
        with self.assertRaises(ValueError):
            trial.command_script('start', command, 'q2', 0x40000000, 500)

    def test_q2_capture_selects_pid_and_requested_length_once(self):
        def encode(row):
            return (','.join(str(row[k]) for k in trial.HEADER) + '\n').encode()
        chunks = [b''.join(encode(make_row(time_ms=3000 + i*20,
                    pitch_cdeg=650, pitch_rate_cdeg_s=400)) for i in range(20)),
                  encode(make_row(time_ms=3400, mode=8, q1_state=1,
                    pitch_cdeg=80, target_pitch_cdeg=100, pwm1_applied_permille=100,
                    problem_id=2, target_length_mm=400)),
                  encode(make_row(time_ms=18000, q1_state=2, q1_reason=1))]

        class FakePort:
            @property
            def in_waiting(self):
                return len(chunks[0])
            def open(self):
                pass
            def read(self, size):
                return chunks.pop(0)
            def write(self, data):
                raise AssertionError('UART must remain receive-only')
            def close(self):
                pass

        with tempfile.TemporaryDirectory() as temp, \
                mock.patch.object(trial, 'ROOT', pathlib.Path(temp)), \
                mock.patch.object(trial.serial, 'Serial', return_value=FakePort()), \
                mock.patch.object(trial.time, 'monotonic', side_effect=[0, 0, 1, 1.1, 1.2, 1.3, 1.4, 20.1, 20.2]), \
                mock.patch.object(trial, 'mailbox') as mailbox, \
                contextlib.redirect_stdout(io.StringIO()):
            trial.capture(argparse.Namespace(action='start', problem='q2', length_cm=40,
                                             port='TEST_ONLY', duration=20))
            self.assertEqual([call.args[0] for call in mailbox.call_args_list], ['start', 'stop'])
            self.assertEqual(mailbox.call_args_list[0].kwargs, {'problem':'q2', 'length_mm':400})
            metadata = json.loads(next(pathlib.Path(temp).rglob('session.json')).read_text(encoding='utf-8'))
            self.assertEqual(metadata['requested_length_mm'], 400)
            self.assertEqual(metadata['analysis']['controller'], 'q2_trajectory_pid')
            self.assertAlmostEqual(metadata['analysis']['pitch_tracking_rmse_deg'], 0.2)
            self.assertTrue(next(pathlib.Path(temp).rglob('fields.csv')).exists())
            self.assertTrue(next(pathlib.Path(temp).rglob('analysis.json')).exists())
            self.assertIn('Q2', next(pathlib.Path(temp).rglob('telemetry.csv')).parts)
            import csv
            with next(pathlib.Path(temp).rglob('telemetry.csv')).open(encoding='utf-8-sig',newline='') as f:
                captured = list(csv.DictReader(f))
            self.assertAlmostEqual(float(captured[20]['pitch_error_deg']),0.2)

    def test_legacy_rows_and_q2_reference_cycle_metrics(self):
        import math
        row = make_row()
        legacy = ','.join(str(row[k]) for k in trial.LEGACY_HEADER).encode()
        self.assertEqual(trial.parse_line(legacy)['problem_id'], 1)
        self.assertIsNone(trial.parse_line(','.join(trial.LEGACY_HEADER).encode()))
        active=[]
        for ms in range(4000,6000,20):
            angle=round(math.degrees(math.atan(0.3*math.sin(2*math.pi*ms/2000)/0.99))*100)
            active.append(make_row(time_ms=ms,mode=8,q1_state=1,q1_run_ms=ms,
                reference_period_ms=2000,reference_ramp_ms=2000,target_length_mm=600,
                pitch_cdeg=angle,target_pitch_cdeg=angle,problem_id=2))
        metrics=trial.q2_cycle_metrics(active)
        self.assertEqual(len(metrics),1)
        self.assertAlmostEqual(metrics[0]['ground_span_cm'],60,delta=0.05)
        self.assertEqual(metrics[0]['pitch_rmse_deg'],0)
        self.assertAlmostEqual(metrics[0]['center_cm'],0,delta=0.01)

    def test_schema_and_bounds(self):
        row = make_row(q1_state=1, mode=3, pwm2_applied_permille=1000)
        data = ','.join(str(row[k]) for k in trial.HEADER).encode()
        self.assertEqual(trial.parse_line(data), row)
        self.assertEqual(len(trial.HEADER), 58)
        self.assertIsNone(trial.parse_line(','.join(trial.HEADER).encode()))
        with self.assertRaises(ValueError):
            trial.parse_line(b'1,2,3')
        row['pwm2_applied_permille'] = 500
        self.assertEqual(trial.parse_line(','.join(str(row[k]) for k in trial.HEADER).encode()), row)
        row['pwm2_applied_permille'] = 1001
        with self.assertRaises(ValueError):
            trial.parse_line(','.join(str(row[k]) for k in trial.HEADER).encode())

    def test_wire_header_matches_firmware(self):
        import re
        source = (pathlib.Path(__file__).resolve().parents[1] / 'Core/Src/pendulum_app.c').read_text(encoding='utf-8')
        strings = re.search(r'static const char header\[\] =\s*(.*?);', source, re.S).group(1)
        header = ''.join(re.findall(r'"([^"]*)"', strings)).replace('\\r\\n', '').split(',')
        self.assertEqual(header, trial.HEADER)

    def test_angle_projection_analysis(self):
        rows = [make_row(q1_state=1, mode=3, pitch_cdeg=-1417,
                         pwm1_applied_permille=1000),
                make_row(q1_state=1, mode=3, time_ms=4000, pitch_cdeg=1417,
                         pwm2_applied_permille=1000),
                make_row(q1_state=2, q1_reason=1, time_ms=5000)]
        report = trial.analyze(rows)
        self.assertAlmostEqual(report['estimated_ground_span_cm'], 50, delta=0.02)
        self.assertEqual(report['both_on_rows'], 0)
        self.assertEqual(report['last_reason'], 'time_up')
        self.assertEqual(report['left_on_rows'], 1)
        self.assertEqual(report['right_on_rows'], 1)
        self.assertTrue(report['valid_for_tuning'])

    def test_restart_invalidates_analysis_but_rollover_does_not(self):
        report = trial.analyze([make_row(time_ms=798141, q1_state=1),
                                make_row(time_ms=60)])
        self.assertEqual(report['mcu_time_restarts'], 1)
        self.assertFalse(report['valid_for_tuning'])
        self.assertNotIn('estimated_ground_span_cm', report)
        report = trial.analyze([make_row(time_ms=0xFFFFFFF0, q1_state=1),
                                make_row(time_ms=4, q1_state=1)])
        self.assertEqual(report['mcu_time_restarts'], 0)
        self.assertEqual(report['active_recorded_span_ms'], 20)

    def test_incomplete_or_failed_capture_is_not_valid_for_tuning(self):
        rows = [make_row(q1_state=1, mode=3, pwm1_applied_permille=933)]
        self.assertFalse(trial.analyze(rows)['valid_for_tuning'])
        rows.append(make_row(q1_state=2, q1_reason=1, time_ms=18000))
        self.assertTrue(trial.analyze(rows)['valid_for_tuning'])
        self.assertFalse(trial.analyze(rows, capture_error='stop failed')['valid_for_tuning'])

    def test_restart_after_start_stops_once_without_retry(self):
        def encode(row):
            return (','.join(str(row[k]) for k in trial.HEADER) + '\n').encode()

        chunks = [b''.join(encode(make_row(time_ms=3000 + i * 20,
                                          pitch_cdeg=650, pitch_rate_cdeg_s=400)) for i in range(20)),
                  encode(make_row(time_ms=60))]

        class FakePort:
            @property
            def in_waiting(self):
                return len(chunks[0])
            def open(self):
                pass
            def read(self, size):
                return chunks.pop(0)
            def write(self, data):
                raise AssertionError('UART must remain receive-only')
            def close(self):
                pass

        with tempfile.TemporaryDirectory() as temp, \
                mock.patch.object(trial, 'ROOT', pathlib.Path(temp)), \
                mock.patch.object(trial.serial, 'Serial', return_value=FakePort()), \
                mock.patch.object(trial.time, 'monotonic', side_effect=[0, 0, 1, 1.1, 1.2, 1.3]), \
                mock.patch.object(trial, 'mailbox') as mailbox, \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit):
                trial.capture(argparse.Namespace(action='start', port='TEST_ONLY', duration=20))
            self.assertEqual([call.args[0] for call in mailbox.call_args_list], ['start', 'stop'])
            metadata = json.loads(next(pathlib.Path(temp).rglob('session.json')).read_text(encoding='utf-8'))
            self.assertIn('MCU time restarted', metadata['error'])
            self.assertEqual(metadata['analysis']['mcu_time_restarts'], 1)
            self.assertFalse(metadata['analysis']['valid_for_tuning'])

    def test_telemetry_loss_aborts_and_reports_failed_stop(self):
        def encode(row):
            return (','.join(str(row[k]) for k in trial.HEADER) + '\n').encode()

        chunks = [b''.join(encode(make_row(time_ms=3000 + i * 20)) for i in range(20)),
                  encode(make_row(time_ms=3400, q1_state=1, mode=3,
                                  pwm1_applied_permille=933)), b'']

        class FakePort:
            @property
            def in_waiting(self):
                return len(chunks[0])
            def open(self):
                pass
            def read(self, size):
                return chunks.pop(0)
            def write(self, data):
                raise AssertionError('UART must remain receive-only')
            def close(self):
                pass

        def command(action, directory):
            if action == 'stop':
                raise RuntimeError('test target disconnected')

        with tempfile.TemporaryDirectory() as temp, \
                mock.patch.object(trial, 'ROOT', pathlib.Path(temp)), \
                mock.patch.object(trial.serial, 'Serial', return_value=FakePort()), \
                mock.patch.object(trial.time, 'monotonic', side_effect=[0, 0, 1, 1.1, 1.2, 1.3, 2, 2.1]), \
                mock.patch.object(trial, 'mailbox', side_effect=command) as mailbox, \
                contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit):
                trial.capture(argparse.Namespace(action='start', port='TEST_ONLY', duration=20))
            self.assertEqual([call.args[0] for call in mailbox.call_args_list], ['start', 'stop'])
            metadata = json.loads(next(pathlib.Path(temp).rglob('session.json')).read_text(encoding='utf-8'))
            self.assertIn('Telemetry lost', metadata['error'])
            self.assertIn('final stop failed', metadata['error'])
            self.assertLess(metadata['duration_s'], 20)
            self.assertFalse(metadata['analysis']['valid_for_tuning'])

    def test_observe_does_not_send_any_command(self):
        row = make_row()
        data = (','.join(str(row[k]) for k in trial.HEADER) + '\n').encode()

        class FakePort:
            in_waiting = len(data)
            def open(self):
                assert self.dtr is False and self.rts is False
            def read(self, size):
                return data
            def write(self, data):
                raise AssertionError('Observation must never transmit')
            def close(self):
                pass

        with tempfile.TemporaryDirectory() as temp, \
                mock.patch.object(trial, 'ROOT', pathlib.Path(temp)), \
                mock.patch.object(trial.serial, 'Serial', return_value=FakePort()), \
                mock.patch.object(trial.time, 'monotonic', side_effect=[0, 0, 0.2, 2, 2]), \
                mock.patch.object(trial, 'mailbox') as mailbox, \
                contextlib.redirect_stdout(io.StringIO()):
            trial.capture(argparse.Namespace(action='observe', port='TEST_ONLY', duration=1))
            mailbox.assert_not_called()
            metadata = json.loads(next(pathlib.Path(temp).rglob('session.json')).read_text(encoding='utf-8'))
            self.assertFalse(metadata['start_sent'])
            self.assertFalse(metadata['serial_write'])
            self.assertEqual(metadata['analysis']['rows'], 1)


if __name__ == '__main__':
    unittest.main()
