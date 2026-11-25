export const fields = (
  "time_ms,gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw,roll_cdeg,pitch_cdeg,accel_weight_permille," +
  "target_roll_cdeg,target_pitch_cdeg,temp_cdeg,p_roll_milli,i_roll_milli,d_roll_milli," +
  "p_pitch_milli,i_pitch_milli,d_pitch_milli,pid_roll_milli,pid_pitch_milli,target_position_mm,target_velocity_mm_s," +
  "pwm1_requested_permille,pwm2_requested_permille,pwm3_requested_permille,pwm4_requested_permille," +
  "pwm1_applied_permille,pwm2_applied_permille,pwm3_applied_permille,pwm4_applied_permille," +
  "bus_current_adc,mode,direction_cdeg,tilt_cdeg,direction_valid,pitch_rate_cdeg_s,q1_state,q1_reason,q1_run_ms,q1_switches," +
  "predicted_peak_cdeg,left_peak_cdeg,right_peak_cdeg,en_command,en_pin,keys,problem_id,target_length_mm," +
  "roll_rate_cdeg_s,target_pitch_rate_cdeg_s,filtered_pitch_rate_error_cdeg_s,pitch_integrating,pitch_kp_milli," +
  "pitch_ki_milli,pitch_kd_milli,reference_period_ms,reference_ramp_ms,pwm_limit_permille,q1_control_phase," +
  "q1_amplitude_cdeg,q1_amplitude_target_cdeg,q1_amplitude_error_cdeg,q1_amplitude_output_milli," +
  "q1_amplitude_ka_milli,q1_amplitude_kcomp_milli,q1_amplitude_blend_permille,q1_vcom_trial_id"
).split(",");

const indices = Object.fromEntries(fields.map((name, index) => [name, index]));
export const tuningKeys = ["startupKp", "startupKd", "amplitudeKp", "amplitudeKi",
  "amplitudeKd", "compensation", "targetDeg"];

export const snapshotFields = [
  ["startupKp", "启动 PD Kp", 1000], ["startupKd", "启动 PD Kd", 1000],
  ["amplitudeKp", "幅值 PID Kp", 1000], ["amplitudeKi", "幅值 PID Ki", 1000],
  ["amplitudeKd", "幅值 PID Kd", 1000], ["compensation", "阻尼补偿", 1000],
  ["targetDeg", "目标幅值 (°)", 1000], ["periodMs", "参考周期 (ms)", 1],
  ["rampMs", "参考增幅 (ms)", 1], ["durationMs", "运行窗口 (ms)", 1],
  ["pwmPercent", "PWM 上限 (%)", 10], ["rateFilterMs", "角速度滤波 (ms)", 1],
  ["amplitudeFilterMs", "幅值 D 滤波 (ms)", 1], ["holdMs", "切换保持时间 (ms)", 1],
  ["blendMs", "混合时长 (ms)", 1], ["switchPercent", "幅值切换阈值 (%)", 10],
  ["integralLimit", "幅值积分限值", 1000], ["startAngleDeg", "启动角度范围 (°)", 1000],
  ["pitchLimitDeg", "Pitch 停机阈值 (°)", 1000], ["rollLimitDeg", "Roll 停机阈值 (°)", 1000],
  ["rateLimit", "Pitch 角速度停机阈值 (°/s)", 1000], ["gainLimit", "幅值 PID 输出限值", 1000],
  ["pivotM", "支点离地高度 (m)", 1000], ["startupKi", "启动控制 Ki", 1000],
  ["mode", "控制模式（1 为两阶段）", 1], ["startupILimit", "启动 PID 积分限值", 1000],
  ["telemetryMs", "遥测发送间隔 (ms)", 1],
  ["releasePercent", "幅值切换复位阈值 (%)", 10],
];

const actions = ["INFO", "START", "STOP", "RESET"];

function crc16(payload) {
  let crc = 0xffff;
  for (const char of payload) {
    crc ^= char.charCodeAt(0) << 8;
    for (let bit = 0; bit < 8; bit++)
      crc = ((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0)) & 0xffff;
  }
  return crc.toString(16).padStart(4, "0").toUpperCase();
}

export function commandFrame(id, action, tuning) {
  const params = action === "START" ? tuningKeys.map(key => Math.round(tuning[key] * 1000)) : [];
  const payload = ["!WP1", id, action, ...params].join(",");
  return payload + "*" + crc16(payload) + "\n";
}

export function parseReply(line) {
  const match = /^(!WP1,[\x20-\x7e]+)\*([A-F0-9]{4})$/.exec(line);
  if (!match || crc16(match[1]) !== match[2]) return null;
  const [, id, status, action, ...params] = match[1].split(",");
  if (!/^\d+$/.test(id) || !actions.includes(action)) return null;
  if (status === "ERR" && params.length === 1)
    return { id: Number(id), action, error: params[0] };
  if (status !== "OK") return null;
  let snapshot;
  if (action === "INFO" || action === "START") {
    if (params.length !== snapshotFields.length ||
        params.some(value => !/^\d+$/.test(value) || Number(value) > 0xffffffff)) return null;
    snapshot = Object.fromEntries(snapshotFields.map(([key, , scale], i) => [key, Number(params[i]) / scale]));
  } else if (params.length) return null;
  return { id: Number(id), action, snapshot };
}

export function parseTelemetry(line) {
  const values = line.split(",");
  if (![59, 67, fields.length].includes(values.length) || values.some(value => !/^-?\d{1,10}$/.test(value))) return null;
  const data = values.map(Number);
  const get = name => data[indices[name]];
  const time = get("time_ms");
  const state = get("q1_state");
  const reason = get("q1_reason");
  const runTime = get("q1_run_ms");
  const problem = get("problem_id");
  const trialId = get("q1_vcom_trial_id") ?? 0;
  const pwm1 = get("pwm1_applied_permille") / 10;
  const pwm2 = get("pwm2_applied_permille") / 10;
  if (time < 0 || time > 0xffffffff || state < 0 || state > 4 || reason < 0 || reason > 8 ||
      runTime < 0 || runTime > 0xffffffff || problem < 0 || problem > 2 ||
      trialId < 0 || trialId > 0xffffffff || pwm1 < 0 || pwm1 > 100 || pwm2 < 0 || pwm2 > 100) return null;
  return {
    line, time, state, reason, runTime, problem, trialId, mode: get("mode"), en: get("en_pin"),
    angle: get("pitch_cdeg") / 100, roll: get("roll_cdeg") / 100, rate: get("pitch_rate_cdeg_s") / 100,
    targetAngle: state === 1 ? get("target_pitch_cdeg") / 100 : null,
    targetRate: state === 1 ? get("target_pitch_rate_cdeg_s") / 100 : null,
    amplitude: state === 1 && problem === 1 && values.length >= 67
      ? get("q1_amplitude_cdeg") / 100 : null,
    targetAmplitude: state === 1 && problem === 1 && values.length >= 67
      ? get("q1_amplitude_target_cdeg") / 100 : null,
    phase: get("q1_control_phase"), pwm1, pwm2, pwm: pwm1 - pwm2,
  };
}

export function isClockReset(previous, current) {
  return current < previous && ((current - previous) >>> 0) > 0x7fffffff;
}

export function createLineReader(onLine) {
  let buffer = "";
  let discard = false;
  return chunk => {
    for (const char of chunk) {
      if (char === "\n") {
        if (!discard && buffer) onLine(buffer.replace(/\r$/, ""));
        buffer = "";
        discard = false;
      } else if (buffer.length < 1024 && !discard) {
        buffer += char;
      } else {
        discard = true;
      }
    }
  };
}
