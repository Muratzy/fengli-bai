"""Replay WP1 Q1 telemetry through the native old and current C controllers."""

import argparse
import json
from pathlib import Path
import re
import statistics
import subprocess
import tempfile

from q1_trial import ROOT, parse_line

TUNING_KEYS = ("startupKp", "startupKd", "amplitudeKp", "amplitudeKi",
               "amplitudeKd", "compensation", "targetDeg")
SENSORS = ("roll_cdeg", "pitch_cdeg", "pitch_rate_cdeg_s")
STEP_MS = 2


def prepare_replay(record, duration_ms):
    snapshot = record["snapshot"]
    rows = []
    for line in record["rows"]:
        row = parse_line(line.encode("ascii"))
        if row["problem_id"] != 1 or row["q1_vcom_trial_id"] != record["commandId"]:
            raise ValueError("Trial ID or problem does not match the snapshot")
        if row["q1_state"] == 1:
            rows.append(row)
    if not rows or rows[0]["q1_run_ms"] > 40 or rows[-1]["q1_run_ms"] < duration_ms:
        raise ValueError("Recording does not cover the requested window")
    times = [row["q1_run_ms"] for row in rows]
    if any(not 0 < b - a <= 50 for a, b in zip(times, times[1:])):
        raise ValueError("Recording has duplicate times or a gap over 50 ms")

    samples = [" ".join(str(snapshot[key]) for key in TUNING_KEYS)]
    current = 0
    for t in range(0, duration_ms + 1, STEP_MS):
        while times[current + 1] < t:
            current += 1
        left, right = rows[current:current + 2]
        fraction = max(0.0, (t - times[current]) / (times[current + 1] - times[current]))
        values = [(left[key] + fraction * (right[key] - left[key])) / 100 for key in SENSORS]
        samples.append(f"{t} " + " ".join(f"{value:.6f}" for value in values))
    return "\n".join(samples) + "\n", rows


def build_controller(compiler, source, executable):
    subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", "Core/Inc",
                    "tests/replay_q1_samples.c", str(source), "Core/Src/problems/problem_q1.c",
                    "Core/Src/control/pid.c", "Core/Src/control/pendulum_control.c",
                    "-lm", "-o", str(executable)], cwd=ROOT, check=True)


def replay(executable, samples, duration_ms):
    run = subprocess.run([str(executable)], input=samples, text=True,
                         capture_output=True, check=True)
    result = {}
    for line in run.stdout.splitlines():
        time, phase, pwm, center, correction = line.split()
        result[int(time)] = (int(phase), int(pwm), float(center), float(correction))
    if len(result) != duration_ms // STEP_MS or max(result) != duration_ms:
        raise RuntimeError("Incomplete C replay")
    return result


def compare(record, samples, recorded, baseline, candidate, duration_ms):
    trial = record["id"]
    old = replay(baseline, samples, duration_ms)
    new = replay(candidate, samples, duration_ms)
    for elapsed, before in old.items():
        after = new[elapsed]
        if before[0] != after[0] or (before[0] == 0 and before[1] != after[1]):
            raise RuntimeError(f"Trial {trial}: phase or startup output changed")
        if after[2] * after[3] > 0:
            raise RuntimeError(f"Trial {trial}: correction points away from center")
    settled = [entry for time, entry in new.items()
               if duration_ms - 3000 <= time < duration_ms and entry[0] == 2]
    if not settled:
        raise RuntimeError(f"Trial {trial}: amplitude stage not reached")
    center = statistics.mean(row[2] for row in settled)
    correction = statistics.mean(row[3] for row in settled)
    if abs(center) < 1 or center * correction >= 0:
        raise RuntimeError(f"Trial {trial}: no net correction of observed drift")

    baseline_limit = record["snapshot"]["pwmPercent"] * 10
    baseline_at_limit = sum(abs(row[1]) >= baseline_limit for row in old.values())
    candidate_at_limit = sum(abs(row[1]) >= baseline_limit for row in new.values())
    if candidate_at_limit > baseline_at_limit:
        raise RuntimeError(f"Trial {trial}: correction increased time at the PWM limit")

    measured_error = []
    for row in recorded:
        time = row["q1_run_ms"]
        if not 0 < time < duration_ms:
            continue
        phase, predicted, *_ = old.get(time, old[time // STEP_MS * STEP_MS or STEP_MS])
        if phase == row["q1_control_phase"]:
            applied = row["pwm1_applied_permille"] - row["pwm2_applied_permille"]
            measured_error.append(abs(applied - predicted))
    if not measured_error:
        raise RuntimeError(f"Trial {trial}: no aligned baseline measurements")

    return {
        "id": trial, "samples": len(new), "durationMs": duration_ms,
        "firstAmplitudeMs": next(time for time, row in new.items() if row[0] == 2),
        "baselineTelemetryMaePwmPercent": statistics.mean(measured_error) / 10,
        "baselinePeakPwmPercent": max(abs(row[1]) for row in old.values()) / 10,
        "candidatePeakPwmPercent": max(abs(row[1]) for row in new.values()) / 10,
        "baselineLimitSamples": baseline_at_limit,
        "candidateLimitSamples": candidate_at_limit,
        "last3sCenterDeg": center, "last3sCorrectionPwmPercent": correction / 10,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", nargs="+", type=Path, help="Original browser trial JSON files")
    parser.add_argument("--baseline-ref", required=True, help="Git revision of the old Q1 controller")
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    config = (ROOT / "Core/Inc/app_config.h").read_text(encoding="utf-8")
    duration_ms = int(re.search(r"#define APP_Q1_DURATION_MS\s+(\d+)U", config).group(1))
    old_source = subprocess.check_output(["git", "show",
        f"{args.baseline_ref}:WindPendulum_F407/Core/Src/problems/q1_swing.c"], cwd=ROOT)
    with tempfile.TemporaryDirectory(prefix="q1-replay-") as directory:
        folder = Path(directory)
        baseline_src = folder / "baseline.c"
        baseline_src.write_bytes(old_source)
        baseline, candidate = folder / "baseline.exe", folder / "candidate.exe"
        build_controller(args.cc, baseline_src, baseline)
        build_controller(args.cc, Path("Core/Src/problems/q1_swing.c"), candidate)
        results = []
        for path in sorted(args.records):
            record = json.loads(path.read_text(encoding="utf-8"))
            samples, rows = prepare_replay(record, duration_ms)
            results.append(compare(record, samples, rows, baseline, candidate, duration_ms))
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
