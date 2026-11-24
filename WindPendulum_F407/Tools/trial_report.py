"""Snapshot metrics and plots for the Q1/Q2 control trial."""

import math
import statistics


MAX_FIT_GAP_MS = 50


def pitch_phase_lag_ms(rows, period_ms):
    """Fit each Pitch trace at the reference frequency with an independent offset."""
    import numpy as np

    phase = np.array([row['q1_run_ms'] % period_ms for row in rows]) * (math.tau / period_ms)
    design = np.column_stack((np.sin(phase), np.cos(phase), np.ones(len(rows))))
    angles = [[row['pitch_cdeg'], row['target_pitch_cdeg']] for row in rows]
    coefficients, _, rank, _ = np.linalg.lstsq(design, angles, rcond=None)
    if rank < 3 or min(np.hypot(*coefficients[:2])) < 1.0:
        return None
    actual_phase, target_phase = np.arctan2(coefficients[1], coefficients[0])
    return math.remainder(target_phase - actual_phase, math.tau) * period_ms / math.tau


def pitch_period(active, ramp_ms):
    """Estimate period from same-direction Pitch zero crossings after ramp-up."""
    periods = []
    crossings = {}
    for a, b in zip(active, active[1:]):
        t0, t1 = a['q1_run_ms'], b['q1_run_ms']
        if t0 < ramp_ms or not 0 < t1 - t0 <= MAX_FIT_GAP_MS:
            crossings.clear()
            continue
        p0, p1 = a['pitch_cdeg'], b['pitch_cdeg']
        if not (p0 < 0 <= p1 or p0 > 0 >= p1):
            continue
        rising = p1 > p0
        if abs(a['pitch_rate_cdeg_s']) < 500:
            crossings.pop(rising, None)
            continue
        zero_ms = t0 - p0 * (t1 - t0) / (p1 - p0)
        if rising in crossings:
            periods.append(zero_ms - crossings[rising])
        crossings[rising] = zero_ms
    return {
        'actual_pitch_period_ms': statistics.median(periods) if periods else None,
        'actual_pitch_period_intervals': len(periods),
    }


def pwm_saturation(active, historical_limit=None):
    """Estimate at-limit duration from valid adjacent RUNNING snapshots."""
    limits = {row.get('pwm_limit_permille', 0) for row in active}
    limit = None
    source = None
    if len(limits) == 1:
        recorded_limit = next(iter(limits))
        limit = recorded_limit or historical_limit
        if limit is not None:
            source = 'mcu' if recorded_limit else 'provided_for_saved_session'

    def at_limit(row):
        return max(row['pwm1_applied_permille'], row['pwm2_applied_permille']) >= limit

    observed_ms = 0
    saturated_ms = 0
    for a, b in zip(active, active[1:]):
        gap = b['q1_run_ms'] - a['q1_run_ms']
        if 0 < gap <= MAX_FIT_GAP_MS:
            observed_ms += gap
            if limit is not None and at_limit(a):
                saturated_ms += gap

    return {
        'limit_permille': limit,
        'limit_source': source,
        'observed_ms': observed_ms,
        'at_limit_ms': saturated_ms if limit is not None else None,
        'at_limit_percent': 100 * saturated_ms / observed_ms
            if limit is not None and observed_ms else None,
        'at_limit_rows': sum(at_limit(row) for row in active) if limit is not None else None,
    }


def first_terminal_index(rows, first_active):
    return next((i for i in range(first_active + 1, len(rows))
                 if rows[i]['q1_state'] in (2, 3, 4)), None)


def session_timeline(rows):
    """Align the whole capture to the first run, including MCU tick wrap."""
    first_active = next(i for i, row in enumerate(rows) if row['q1_state'] == 1)
    start = rows[first_active]
    start_tick = start['time_ms'] - start['q1_run_ms']
    seconds = [((row['time_ms'] - start_tick + 0x80000000) % 0x100000000
                - 0x80000000) / 1000 for row in rows]
    return seconds, first_active, first_terminal_index(rows, first_active)


def save_curves(path, rows, report):
    from matplotlib.figure import Figure

    figure = Figure(figsize=(12, 8), layout='constrained')
    angle, rate, output = figure.subplots(3, 1, sharex=True)
    seconds, first_active, terminal = session_timeline(rows)
    plot_seconds = []
    plot_rows = []
    for t, row in zip(seconds, rows):
        if plot_seconds and t - plot_seconds[-1] > MAX_FIT_GAP_MS / 1000:
            plot_seconds.append((t + plot_seconds[-1]) / 2)
            plot_rows.append({})
        plot_seconds.append(t)
        plot_rows.append(row)

    def trace(axis, field, divisor, label, color, style='-', running_only=False):
        values = [row.get(field, float('nan')) / divisor
                  if not running_only or row.get('q1_state') == 1 else float('nan')
                  for row in plot_rows]
        axis.plot(plot_seconds, values,
                  label=label, color=color, linestyle=style, linewidth=1.4)

    trace(angle, 'target_pitch_cdeg', 100, 'Target', '#59677a', '--', running_only=True)
    trace(angle, 'pitch_cdeg', 100, 'Measured', '#006f9d')
    angle.set_ylabel('Pitch (deg)')
    if report['target_pitch_period_ms'] is not None:
        trace(rate, 'target_pitch_rate_cdeg_s', 100, 'Target', '#59677a', '--', running_only=True)
    trace(rate, 'pitch_rate_cdeg_s', 100, 'Measured', '#006f9d')
    rate.set_ylabel('Pitch rate (deg/s)')
    trace(output, 'p_pitch_milli', 10000, 'P term', '#096b8d', running_only=True)
    trace(output, 'd_pitch_milli', 10000, 'D term', '#be5318', running_only=True)
    signed_pwm = [(row['pwm1_applied_permille'] - row['pwm2_applied_permille']) / 10
                  if 'pwm1_applied_permille' in row else float('nan') for row in plot_rows]
    output.plot(plot_seconds, signed_pwm, label='Applied PWM1 - PWM2',
                color='#24252a', linewidth=1.2, linestyle='--')
    limit = report['pwm_saturation']['limit_permille']
    if limit is not None:
        output.axhline(limit / 10, color='#9a9da2', linestyle=':', label='PWM limit')
        output.axhline(-limit / 10, color='#9a9da2', linestyle=':')
    output.set_ylabel('PWM equivalent (%)')
    output.set_xlabel('Time relative to trial start (s)')

    for axis in figure.axes:
        axis.axvline(0, color='#687582', linewidth=0.85)
        if terminal is not None:
            axis.axvspan(0, seconds[terminal], color='#e8f0f6', alpha=0.45, zorder=0)
            axis.axvline(seconds[terminal], color='#a93232', linewidth=1, linestyle='--')
        axis.axhline(0, color='#a9b0b7', linewidth=0.65, zorder=0)
        axis.spines[['top', 'right']].set_visible(False)
        axis.grid(color='#dee3e9', linewidth=0.6)
        axis.legend(loc='upper left', ncols=2, fontsize=8, framealpha=0.92)
    if terminal is not None:
        stop_t = seconds[terminal]
        stop_pitch = rows[terminal]['pitch_cdeg'] / 100
        angle.scatter([stop_t], [stop_pitch], s=19, color='#a93232', zorder=4)
        angle.annotate(f'Stop {stop_t:.2f} s', (stop_t, 0.97),
                       xycoords=('data', 'axes fraction'), xytext=(5, 0),
                       textcoords='offset points', va='top', color='#a93232', fontsize=8)
    problem = f"Q{rows[first_active]['problem_id']}"
    actual_period = report['actual_pitch_period_ms']
    period_label = f'{actual_period:.0f} ms' if actual_period is not None else 'unavailable'
    target_period = report['target_pitch_period_ms']
    target_label = f'{target_period:.0f} ms' if target_period is not None else 'unavailable'
    stop = report['first_stop']
    reason = stop['reason'] if stop is not None else 'stop not observed'
    figure.suptitle(
        f"{problem} pitch control | {reason}\n"
        f"Period {period_label} (target {target_label})  |  "
        f"Angle RMSE {report['pitch_tracking_rmse_deg']:.2f} deg",
        fontsize=13, fontweight='semibold',
    )
    figure.savefig(path, dpi=160, facecolor='white')
