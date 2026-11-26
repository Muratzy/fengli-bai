export const measuredColor = "#32746c";
export const referenceColor = "#877f72";
export const secondaryColor = "#b55840";

export function chart(canvas, series, {
  start = 0, end = 12, min = -20, max = 20, positionPlot = false, zeroLine = false, maxGap = 0.15,
} = {}) {
  const horizontal = positionPlot ? "x" : "t";
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!width || !height) return;
  const ratio = devicePixelRatio || 1;
  const pixelWidth = Math.round(width * ratio);
  const pixelHeight = Math.round(height * ratio);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);

  for (const { points, value } of series) {
    for (const point of points) {
      if (point[horizontal] < start || point[horizontal] > end || !Number.isFinite(point[value])) continue;
      min = Math.min(min, point[value]);
      max = Math.max(max, point[value]);
    }
  }
  const left = 35, top = 9, plotWidth = width - left - 9, plotHeight = height - 28;
  const x = position => left + plotWidth * (position - start) / (end - start);
  const y = value => top + plotHeight * (max - value) / (max - min);

  ctx.font = "10px ui-monospace, monospace";
  ctx.fillStyle = "#726b5f";
  ctx.strokeStyle = "#e9e4da";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.textAlign = "right";
  for (let i = 0; i <= 4; i++) {
    const value = max - (max - min) * i / 4;
    const row = y(value);
    ctx.moveTo(left, row);
    ctx.lineTo(width - 9, row);
    ctx.fillText(String(positionPlot ? Number(value.toFixed(2)) : Math.round(value)), left - 4, row + 3);
  }
  const horizontalTicks = positionPlot ? 4 : 3;
  for (let i = 0; i <= horizontalTicks; i++) {
    const value = start + (end - start) * i / horizontalTicks;
    const col = x(value);
    ctx.moveTo(col, top);
    ctx.lineTo(col, top + plotHeight);
    const label = positionPlot ? String(Number(value.toFixed(2))) : value.toFixed(1) + "s";
    ctx.fillText(label, col + (i < horizontalTicks ? 12 : 0), height - 5);
  }
  ctx.stroke();

  ctx.save();
  ctx.beginPath();
  ctx.rect(left, top, plotWidth, plotHeight);
  ctx.clip();
  if (zeroLine) {
    ctx.beginPath();
    ctx.strokeStyle = referenceColor;
    ctx.setLineDash([4, 4]);
    ctx.moveTo(left, y(0));
    ctx.lineTo(left + plotWidth, y(0));
    ctx.stroke();
  }
  for (const { points, value, color, dashed, maxGap: seriesGap = maxGap } of series) {
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.65;
    ctx.setLineDash(dashed ? [4, 4] : []);
    let previous = NaN;
    for (const point of points) {
      if (!Number.isFinite(point[value])) {
        previous = NaN;
        continue;
      }
      const col = x(point[horizontal]);
      const row = y(point[value]);
      if (!Number.isFinite(previous) || point.t - previous > seriesGap || point.t < previous)
        ctx.moveTo(col, row);
      else ctx.lineTo(col, row);
      previous = point.t;
    }
    ctx.stroke();
    const last = points.at(-1);
    if (positionPlot && !dashed && last) {
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(x(last[horizontal]), y(last[value]), 2.8, 0, 2 * Math.PI);
      ctx.fill();
    }
  }
  ctx.restore();
}
