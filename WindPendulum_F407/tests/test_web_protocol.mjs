import assert from "node:assert/strict";
import { commandFrame, createLineReader, fields, parseReply, parseTelemetry } from "../上位机/protocol.mjs";

function telemetry(overrides = {}, count = fields.length) {
  const sample = { time_ms: 5000, mode: 10, problem_id: 3, q1_state: 1,
    q1_run_ms: 2000, q1_amplitude_cdeg: 720, q1_amplitude_target_cdeg: 720,
    pwm1_applied_permille: 100, pwm3_applied_permille: 200, ...overrides };
  return fields.slice(0, count).map(key => sample[key] ?? 0).join(",");
}

for (const count of [59, 67, 68, 70]) {
  for (const problem of [1, 2, 3]) {
    const row = parseTelemetry(telemetry({ problem_id: problem }, count));
    assert.equal(row.problem, problem);
    assert.equal(row.pwm1, 10);
    assert.equal(row.pwm3, 20);
    assert.equal(row.pwm4, 0);
    assert.equal(row.pwm, 10);
    assert.equal(row.amplitude, count >= 67 && problem !== 2 ? 7.2 : null);
    assert.equal(row.targetAmplitude, count >= 67 && problem !== 2 ? 7.2 : null);
  }
}
assert.equal(parseTelemetry(telemetry({ problem_id: 5 })), null);
assert.equal(parseTelemetry(telemetry({ problem_id: 3, q1_reason: 9 })), null);
const brake = parseTelemetry(telemetry({ problem_id: 4, mode: 5, q1_state: 2, q1_reason: 9,
  q1_run_ms: 2500, q1_vcom_trial_id: 41, tilt_cdeg: 120, roll_rate_cdeg_s: -300,
  pitch_rate_cdeg_s: 400, q1_control_phase: 2 }));
assert.equal(brake.angle, 1.2);
assert.equal(brake.rollRateDps, -3);
assert.equal(brake.pitchRateDps, 4);
assert.equal(brake.rate, 5);
assert.equal(brake.runTime, 2500);
assert.equal(brake.trialId, 41);
assert.equal(brake.reason, 9);
assert.equal(brake.phase, 2);
assert.equal(brake.amplitude, null);
assert.equal(parseTelemetry(telemetry({ pwm3_applied_permille: 1001 })), null);
assert.equal(parseTelemetry(telemetry({ pwm4_applied_permille: -1 })), null);
assert.equal(parseTelemetry(telemetry({ q1_state: 2 })).amplitude, null);
const q3Up = parseTelemetry(telemetry({
  roll_cdeg: 100, roll_rate_cdeg_s: 300, pitch_cdeg: 200, pitch_rate_cdeg_s: 400,
}), 0);
assert.ok(Math.abs(q3Up.angle - 1) < 0.001);
assert.ok(Math.abs(q3Up.rate - 3) < 0.01);
const q3Right = parseTelemetry(telemetry({
  roll_cdeg: 100, roll_rate_cdeg_s: 300, pitch_cdeg: 200, pitch_rate_cdeg_s: 400,
}), 90);
assert.ok(Math.abs(q3Right.angle - 2) < 0.001);
assert.ok(Math.abs(q3Right.rate - 4) < 0.02);

const received = [];
const read = createLineReader(line => received.push(parseTelemetry(line)));
const q3 = telemetry();
read(q3.slice(0, 13));
read(q3.slice(13) + "\r\n");
assert.equal(received.length, 1);
assert.equal(received[0].problem, 3);
assert.equal(received[0].trialId, 0);

const start = commandFrame(42, "START", {
  startupKp: 28, startupKd: 7, amplitudeKp: 2, amplitudeKi: 0,
  amplitudeKd: 0, compensation: 0.5, targetDeg: 16.858, durationMs: 25000,
});
assert.match(start, /^!WP1,42,START,28000,7000,2000,0,0,500,16858,25000\*[A-F0-9]{4}\n$/);
const q3Start = commandFrame(43, "START", {
  startupKp: 28, startupKd: 7, amplitudeKp: 2, amplitudeKi: 0,
  amplitudeKd: 0, compensation: 0.5, targetDeg: 7.18, durationMs: 15000,
}, { id: 3, directionDeg: 40, lineLengthMm: 250 });
assert.match(q3Start, /^!WP1,43,START,28000,7000,2000,0,0,500,7180,15000,3,40,250\*[A-F0-9]{4}\n$/);

function crc16(payload) {
  let crc = 0xffff;
  for (const char of payload) {
    crc ^= char.charCodeAt(0) << 8;
    for (let bit = 0; bit < 8; bit++)
      crc = ((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0)) & 0xffff;
  }
  return crc.toString(16).padStart(4, "0").toUpperCase();
}
const q3InfoPayload = "!WP1,44,OK,Q3INFO,3,0,200,600,15000,10";
const q3Info = parseReply(q3InfoPayload + "*" + crc16(q3InfoPayload));
assert.deepEqual(q3Info.q3Info, {
  directionSign: 0, minLengthMm: 200, maxLengthMm: 600,
  durationMs: 15000, directionStepDeg: 10,
});
const q3MixerError = "!WP1,43,ERR,START,MIXER";
assert.deepEqual(parseReply(q3MixerError + "*" + crc16(q3MixerError)), {
  id: 43, action: "START", error: "MIXER",
});
assert.match(commandFrame(45, "Q4ARM"), /^!WP1,45,Q4ARM\*[A-F0-9]{4}\n$/);
for (const action of ["Q4INFO", "Q4ARM"]) {
  const payload = `!WP1,45,OK,${action},4,-1,20000,5000,30,45,150,8000,2,5,300,50000,240000,10000,8000,850,50,1000`;
  const reply = parseReply(payload + "*" + crc16(payload));
  assert.equal(reply.action, action);
  assert.deepEqual(reply.snapshot, { problemId: 4, directionSign: -1, armTimeoutMs: 20000,
    durationMs: 5000, releaseMinDeg: 30, releaseMaxDeg: 45, holdMs: 150, releaseRate: 8,
    settleAngle: 2, settleRate: 5, settleMs: 300, stopAngle: 50, stopRate: 240,
    brakeKp: 10, brakeKd: 8, pwmPercent: 85, rateFilterMs: 50, telemetryMs: 1000 });
}
console.log("Web telemetry protocol tests passed");
