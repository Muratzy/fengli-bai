const radiansPerDegree = Math.PI / 180;

export function projectTrialPoint(row, snapshot) {
  if (row?.state !== 1 || !Number.isFinite(row.pitch) || !Number.isFinite(row.roll) ||
      !Number.isFinite(row.runTime) || Math.abs(row.pitch) >= 90 || Math.abs(row.roll) >= 90) {
    return null;
  }
  const heightCm = (snapshot.pivotM ?? 0.99) * 100;
  const roll = row.roll * radiansPerDegree;
  const pitch = row.pitch * radiansPerDegree;
  const x = heightCm * Math.tan(pitch) / Math.cos(roll);
  const y = heightCm * Math.tan(roll);
  let along = x;
  let across = y;

  if (snapshot.problemId === 3) {
    const direction = snapshot.directionDeg * radiansPerDegree;
    along = Math.sin(direction) * x + Math.cos(direction) * y;
    across = Math.cos(direction) * x - Math.sin(direction) * y;
  }

  return {
    x: along,
    y: across,
    t: row.runTime / 1000,
    pitch: row.angle,
    rate: row.rate,
  };
}

export function targetHalfSpanCm(snapshot) {
  if (snapshot.problemId === 2 || snapshot.problemId === 3) {
    return (snapshot.lineLengthMm ?? 500) / 20;
  }
  const heightCm = (snapshot.pivotM ?? 0.99) * 100;
  return heightCm * Math.tan(snapshot.targetDeg * radiansPerDegree);
}
