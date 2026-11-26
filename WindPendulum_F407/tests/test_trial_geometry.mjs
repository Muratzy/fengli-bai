import assert from "node:assert/strict";
import { projectTrialPoint, targetHalfSpanCm } from "../上位机/trial_geometry.mjs";

const input = {
  state: 1, pitch: 2, roll: 1, angle: 1, rate: 3, runTime: 2500,
};
const q1 = projectTrialPoint(input, { problemId: 1, pivotM: 0.99, targetDeg: 7.18 });
assert.ok(q1);
assert.ok(Math.abs(q1.x - 3.46) < 0.02);
assert.ok(Math.abs(q1.y - 1.73) < 0.02);
assert.equal(q1.t, 2.5);

const up = projectTrialPoint(input, { problemId: 3, directionDeg: 0, pivotM: 0.99 });
assert.ok(Math.abs(up.x - 1.73) < 0.02);
assert.ok(Math.abs(up.y - 3.46) < 0.02);

const right = projectTrialPoint(input, { problemId: 3, directionDeg: 90, pivotM: 0.99 });
assert.ok(Math.abs(right.x - 3.46) < 0.02);
assert.ok(Math.abs(right.y + 1.73) < 0.02);

assert.equal(projectTrialPoint({ ...input, state: 2 }, { problemId: 3, directionDeg: 0 }), null);
assert.equal(targetHalfSpanCm({ problemId: 3, lineLengthMm: 250 }), 12.5);
assert.equal(targetHalfSpanCm({ problemId: 2, lineLengthMm: 500 }), 25);
console.log("Trial coordinate projection tests passed");
