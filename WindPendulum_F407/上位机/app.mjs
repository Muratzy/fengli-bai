import { isClockReset, parseTelemetry, snapshotFields, tuningKeys } from "./protocol.mjs";
import { Vcom } from "./serial.mjs";
import { archiveTrial, historyOperation, openArchiveSettings, openHistory,
  trialCsv, trialFilename } from "./storage.mjs";
import { chart, measuredColor, referenceColor, secondaryColor } from "./charts.mjs";

const byId = id => document.getElementById(id);
const form = byId("parameters");
const targetSpan = form.elements.targetSpanCm;
const durationInput = form.elements.durationSeconds;
const gainKeys = tuningKeys.filter(key => key !== "targetDeg");
const degreesPerRadian = 180 / Math.PI;
const controls = Object.fromEntries(["connect", "start", "stop", "reset"].map(id => [id, byId(id)]));
const states = ["待机", "运行中", "已完成", "已停止", "保护停机"];
const reasons = ["", "到时", "手动停止", "数据过期", "超角或超速", "未检测到换向",
  "EN 低电平", "起摆条件不符", "配置无效"];
const phases = ["启动 PD", "切换混合", "幅值 PID"];
const terminal = row => row.state >= 2;
const trialLabel = id => "#" + String(id).padStart(3, "0");
const nowIso = () => new Date().toISOString();
const archiveButton = byId("archive-directory");
const archiveStatus = byId("archive-status");
const archivePickerSupported = typeof window.showDirectoryPicker === "function";

let database;
let serial;
let boardConfig;
let connecting = false;
let starting;
let resetting = false;
let stopping = false;
let latest;
let lastRx = 0;
let originTime;
let live = [];
let active;
let shownId;
let dirty = true;
const history = new Map();
const selected = new Set();
let comparison = [];
let trajectoryRecord;
let trajectory;
let archiveSettings;
let archiveDirectory;
let archiveQueue = Promise.resolve();
const archivedRecords = new Set();

function notify(message, error = false) {
  byId("message").textContent = message;
  byId("message").classList.toggle("error", error);
}

function updateControls() {
  const fresh = latest && performance.now() - lastRx < 1000;
  const idle = fresh && latest.state !== 1;
  controls.connect.textContent = serial ? "断开串口" : "连接串口";
  controls.connect.disabled = connecting || (!navigator.serial && !serial);
  controls.start.disabled = !(serial && boardConfig?.mode === 1 && database && idle &&
    [2, 7].includes(latest.mode) && !starting && !active && !resetting && !stopping);
  controls.stop.disabled = !(serial && boardConfig && !stopping);
  controls.reset.disabled = !(serial && boardConfig && idle && !starting && !active && !resetting && !stopping);
  for (const field of form.querySelectorAll("input")) field.disabled = Boolean(starting || active);
  durationInput.disabled = Boolean(starting || active || (boardConfig && !boardConfig.maxDurationMs));
  const connected = byId("connection");
  connected.dataset.state = serial && fresh ? "ready" : "idle";
  connected.textContent = connecting ? "正在连接" : !serial ? "串口未连接" :
    !fresh ? "VCOM · 等待遥测" : boardConfig ? "VCOM · 在线" : "VCOM · 只读";
  byId("board-state").textContent = fresh
    ? (latest.mode <= 1 ? "IMU 未就绪" : states[latest.state]) : "—";
  byId("run-id").textContent = shownId ?? "—";
}

function updateDurationSetting() {
  const min = (boardConfig?.minDurationMs ?? 5000) / 1000;
  const max = (boardConfig?.maxDurationMs ?? 60000) / 1000;
  durationInput.min = min;
  durationInput.max = max;
  if (boardConfig) durationInput.value = boardConfig.durationMs / 1000;
  byId("duration-hint").textContent = boardConfig && !boardConfig.maxDurationMs
    ? "当前固件使用固定时长"
    : min + "–" + max + " 秒，由 STM32 到时自动停止";
}

function spanForAngle(angle) {
  return 200 * (boardConfig?.pivotM ?? 0.99) * Math.tan(angle / degreesPerRadian);
}

function targetAngle() {
  const angle = degreesPerRadian * Math.atan(targetSpan.valueAsNumber /
    (200 * (boardConfig?.pivotM ?? 0.99)));
  return Math.round(angle * 1000) / 1000;
}

function updateTargetAngle() {
  byId("target-angle").textContent = targetSpan.validity.valid
    ? "峰值角度 " + targetAngle().toFixed(3) + "°"
    : "可输入 " + targetSpan.min + "–" + targetSpan.max + " cm";
}

function updateSpanLimits() {
  targetSpan.min = (Math.ceil(spanForAngle(5) * 100) / 100).toFixed(2);
  targetSpan.max = (Math.floor(spanForAngle(20) * 100) / 100).toFixed(2);
}

function showArchiveStatus(message, error = false) {
  archiveStatus.textContent = message;
  archiveStatus.classList.toggle("error", error);
}

function syncArchive() {
  const directory = archiveDirectory;
  if (!directory) return;
  archiveQueue = archiveQueue.then(async () => {
    if (directory !== archiveDirectory) return;
    try {
      for (const record of history.values()) {
        if (record === active || archivedRecords.has(record)) continue;
        await archiveTrial(directory, record);
        if (directory !== archiveDirectory) return;
        archivedRecords.add(record);
      }
      showArchiveStatus(directory.name + " · 已归档 " + archivedRecords.size + " 轮（CSV 和 JSON）");
    } catch (error) {
      if (directory === archiveDirectory) {
        showArchiveStatus("文件夹写入失败，记录仍在网页中。请重新选择数据文件夹。", true);
      }
    }
  });
}

archiveButton.addEventListener("click", async () => {
  archiveButton.disabled = true;
  try {
    const directory = await window.showDirectoryPicker({ id: "wind-pendulum-trials", mode: "readwrite" });
    if (!archiveDirectory || !(await directory.isSameEntry(archiveDirectory))) archivedRecords.clear();
    archiveDirectory = directory;
    try {
      await historyOperation(archiveSettings, "readwrite",
        store => store.put(directory, "directory"), "settings");
    } catch (error) {
      showArchiveStatus("当前页面可保存文件，重开网页后需重新选择文件夹。", true);
    }
    syncArchive();
  } catch (error) {
    if (error.name !== "AbortError") showArchiveStatus("无法打开数据文件夹 · " + error.message, true);
  } finally {
    archiveButton.disabled = false;
  }
});

async function save(record) {
  try {
    await historyOperation(database, "readwrite", store => store.put(record));
  } catch (error) {
    notify("本地保存失败。请从实验记录下载 CSV 和 JSON。", true);
  }
}

function finish(status, row) {
  if (!active) return;
  const record = active;
  active = undefined;
  record.status = status;
  record.finishedAt = nowIso();
  record.endState = row?.state;
  record.endReason = row?.reason;
  void save(record);
  syncArchive();
  renderHistory();
  updateControls();
}

function addTrialFrame(row) {
  if (row.problem !== 1 || row.trialId !== active.commandId) return;
  active.rows.push(row.line);
  if (terminal(row)) {
    const status = reasons[row.reason] || states[row.state];
    finish(status, row);
    notify("本轮结束 · " + status);
  }
}

function receiveTelemetry(line) {
  const row = parseTelemetry(line);
  if (!row) return;
  if (latest && isClockReset(latest.time, row.time)) {
    finish("STM32 重启");
    originTime = undefined;
    live = [];
    notify("检测到 STM32 时间重置。", true);
  }
  latest = row;
  lastRx = performance.now();
  originTime ??= row.time;
  row.t = ((row.time - originTime) >>> 0) / 1000;
  live.push(row);
  if (live.length > 1200) live = live.slice(-800);
  if (starting) {
    if (row.problem === 1 && row.trialId !== 0) starting.frames.push(row);
  } else if (active) {
    addTrialFrame(row);
  }
  dirty = true;
  updateControls();
}

async function disconnect(message, isError = false) {
  const current = serial;
  serial = undefined;
  boardConfig = undefined;
  updateDurationSetting();
  updateSpanLimits();
  updateTargetAngle();
  latest = undefined;
  lastRx = 0;
  originTime = undefined;
  if (active) finish("串口断开，记录未完成");
  await current?.close();
  notify(message, isError);
  updateControls();
}

controls.connect.addEventListener("click", async () => {
  if (serial) {
    await disconnect("串口已断开。");
    return;
  }
  connecting = true;
  updateControls();
  let current;
  try {
    const port = await navigator.serial.requestPort();
    current = new Vcom();
    await current.connect(port, receiveTelemetry, error => {
      if (serial === current) void disconnect("串口断开 · " + error.message, true);
    });
    serial = current;
    originTime = undefined;
    live = [];
    dirty = true;
    try {
      const reply = await current.request("INFO");
      if (serial !== current) return;
      boardConfig = reply.snapshot;
      updateDurationSetting();
      for (const key of gainKeys) form.elements[key].value = boardConfig[key];
      updateSpanLimits();
      const defaultSpan = Number(spanForAngle(boardConfig.targetDeg).toFixed(2));
      targetSpan.value = Math.min(Number(targetSpan.max), Math.max(Number(targetSpan.min), defaultSpan));
      updateTargetAngle();
      notify("STM32 已连接。参数随下次启动下发。");
    } catch (error) {
      if (serial === current) notify("串口已打开。当前固件未确认调参协议，可查看遥测。", true);
    }
  } catch (error) {
    await current?.close();
    if (error.name !== "NotFoundError") notify("无法打开串口 · " + error.message, true);
  } finally {
    connecting = false;
    updateControls();
  }
});

async function startTrial() {
  if (controls.start.disabled) return;
  if (!form.checkValidity()) {
    const section = durationInput.validity.valid ? "tuning" : "experiment";
    if (byId(section).hidden) {
      location.hash = section;
      updateNavigation();
    }
    form.reportValidity();
    return;
  }
  const tuning = Object.fromEntries(gainKeys.map(key => [key, form.elements[key].valueAsNumber]));
  tuning.targetDeg = targetAngle();
  if (boardConfig.maxDurationMs) tuning.durationMs = Math.round(durationInput.valueAsNumber * 1000);
  const current = serial;
  const capture = { frames: [] };
  starting = capture;
  updateControls();
  const startedAt = nowIso();
  let accepted;
  try {
    accepted = await current.request("START", tuning);
    if (tuning.durationMs !== undefined && accepted.snapshot.durationMs !== tuning.durationMs) {
      throw new Error("STM32 确认的运行时长与设定不一致");
    }
    const record = {
      protocol: "WP1",
      commandId: accepted.id,
      startedAt,
      acceptedAt: nowIso(),
      snapshot: accepted.snapshot,
      rows: [],
      status: "采集中",
    };
    try {
      record.id = await historyOperation(database, "readwrite", store => store.add(record));
    } catch (error) {
      record.id = "临时-" + accepted.id;
    }
    shownId = typeof record.id === "number" ? trialLabel(record.id) : record.id;
    history.set(record.id, record);
    active = record;
    trajectoryRecord = record;
    starting = undefined;
    for (const row of capture.frames) {
      if (!active) break;
      addTrialFrame(row);
    }
    if (active && serial !== current) finish("串口断开，记录未完成");
    else if (active) {
      notify(typeof record.id === "number" ? shownId + " 已启动，参数已归档。" :
        "本地存储不可用，请在本轮结束后下载数据。", typeof record.id !== "number");
    }
    renderHistory();
  } catch (error) {
    if (!error.boardRejected && serial === current) {
      notify("启动应答未收到，正在请求停止。", true);
      try {
        await current.request("STOP");
        notify("启动应答未收到，STM32 已接收停止命令。", true);
      } catch (stopError) {
        notify("无法确认启动或停止状态。可按板上 KEY3。", true);
      }
    } else {
      notify("启动失败 · " + error.message, true);
    }
  } finally {
    starting = undefined;
    updateControls();
  }
}

controls.start.addEventListener("click", startTrial);
form.addEventListener("submit", event => {
  event.preventDefault();
  void startTrial();
});
targetSpan.addEventListener("input", updateTargetAngle);

controls.stop.addEventListener("click", async () => {
  stopping = true;
  updateControls();
  try {
    await serial.request("STOP");
    if (latest?.state === 1) notify("STM32 已接收停止命令，等待停机遥测。");
    else if (!active) notify("STM32 已接收停止命令。");
  } catch (error) {
    notify("停止应答失败 · " + error.message + "。可按板上 KEY3。", true);
  } finally {
    stopping = false;
    updateControls();
  }
});

controls.reset.addEventListener("click", async () => {
  resetting = true;
  updateControls();
  try {
    await serial.request("RESET");
    shownId = undefined;
    originTime = undefined;
    live = [];
    dirty = true;
    notify("实验状态已重置，串口保持连接。");
  } catch (error) {
    notify("重置失败 · " + error.message, true);
  } finally {
    resetting = false;
    updateControls();
  }
});

function download(name, content, type) {
  const url = URL.createObjectURL(new Blob([content], { type }));
  const link = document.createElement("a");
  link.href = url;
  link.download = name;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function snapshot(record) {
  byId("snapshot-title").textContent = trialLabel(record.id) + " 参数快照";
  const container = byId("snapshot-fields");
  container.replaceChildren();
  for (const [key, label] of snapshotFields) {
    const title = document.createElement("dt");
    const value = document.createElement("dd");
    title.textContent = label;
    value.textContent = record.snapshot[key] ?? "—";
    container.append(title, value);
  }
  byId("snapshot").showModal();
}

function selectTrajectory(record) {
  trajectoryRecord = record;
  for (const button of byId("history").querySelectorAll(".trial-number")) {
    button.setAttribute("aria-pressed", String(button.dataset.trial === String(record.id)));
  }
  drawTrajectory(true);
  byId("trial-trajectory").scrollIntoView({ block: "nearest" });
}

function renderHistory() {
  if (!history.size) return;
  const records = Array.from(history.values()).reverse();
  trajectoryRecord ??= records[0];
  const tbody = byId("history");
  tbody.replaceChildren();
  for (const record of records) {
    const row = tbody.insertRow();
    const check = document.createElement("input");
    check.type = "checkbox";
    check.setAttribute("aria-label", "对比 " + trialLabel(record.id));
    check.disabled = !record.rows.length || active === record;
    check.checked = selected.has(record.id);
    check.addEventListener("change", () => {
      if (check.checked) {
        if (selected.size >= 2) selected.delete(selected.values().next().value);
        selected.add(record.id);
      } else selected.delete(record.id);
      renderHistory();
      updateComparison();
    });
    row.insertCell().append(check);
    const trial = document.createElement("button");
    trial.className = "trial-number";
    trial.dataset.trial = record.id;
    trial.textContent = trialLabel(record.id);
    trial.setAttribute("aria-label", "查看 " + trialLabel(record.id) + " 轨迹");
    trial.setAttribute("aria-pressed", String(record === trajectoryRecord));
    trial.setAttribute("aria-controls", "trial-trajectory");
    trial.addEventListener("click", () => selectTrajectory(record));
    row.insertCell().append(trial);
    row.insertCell().textContent = new Date(record.startedAt).toLocaleString("zh-CN", { hour12: false });
    row.insertCell().textContent = record.status === "采集中" && active !== record
      ? "记录中断" : record.status;
    const parameters = row.insertCell();
    const snapshotButton = document.createElement("button");
    snapshotButton.textContent = "查看 · " + record.snapshot.targetDeg + "°";
    snapshotButton.addEventListener("click", () => snapshot(record));
    parameters.append(snapshotButton);
    const actions = row.insertCell();
    const prefix = trialFilename(record);
    const csv = document.createElement("button");
    csv.textContent = "CSV";
    csv.disabled = !record.rows.length && active !== record;
    csv.addEventListener("click", () => {
      download(prefix + ".csv", trialCsv(record), "text/csv;charset=utf-8");
    });
    const json = document.createElement("button");
    json.textContent = "JSON";
    json.addEventListener("click", () => {
      download(prefix + ".json", JSON.stringify(record, null, 2), "application/json");
    });
    actions.append(csv, json);
  }
  drawTrajectory(true);
}

function updateCycles(trace, point) {
  const previous = trace.points.at(-1);
  if (!previous || point.t < previous.t || point.t - previous.t > 0.15) {
    trace.cycleStart = undefined;
    return;
  }
  if (point.t === previous.t || previous.pitch >= 0 || point.pitch < 0 ||
      Math.max(previous.rate, point.rate) < 5) return;

  const crossing = previous.t - previous.pitch * (point.t - previous.t) /
    (point.pitch - previous.pitch);
  const start = trace.cycleStart;
  trace.cycleStart = { time: crossing, index: trace.points.length };
  if (!start || crossing - start.time < trace.record.snapshot.periodMs / 2000) return;

  let left = 0;
  let right = 0;
  for (let i = start.index; i < trace.points.length; i++) {
    left = Math.min(left, trace.points[i].x);
    right = Math.max(right, trace.points[i].x);
  }
  if (left >= 0 || right <= 0) return;
  trace.cycles.push({ span: right - left, start: start.time, end: crossing });
  if (trace.cycles.length > 3) trace.cycles.shift();
}

function renderCycles(cycles, isRunning) {
  const empty = byId("cycle-empty");
  empty.hidden = cycles.length > 0;
  empty.textContent = isRunning ? "等待一次完整来回。" : "本轮未记录到完整来回。";

  const values = cycles.map(cycle => {
    const item = document.createElement("li");
    const span = document.createElement("strong");
    const time = document.createElement("small");
    span.textContent = cycle.span.toFixed(2) + " cm";
    time.textContent = cycle.start.toFixed(2) + "–" + cycle.end.toFixed(2) + " s";
    item.append(span, time);
    return item;
  });
  byId("cycle-values").replaceChildren(...values);
}

function drawTrajectory(force = false) {
  const record = trajectoryRecord;
  if (!record || byId("records").hidden) return;
  if (trajectory?.record !== record) {
    trajectory = {
      record, read: 0, points: [], cycles: [],
      xExtent: 0, yExtent: 0,
    };
    force = true;
  }
  if (!force && trajectory.read === record.rows.length) return;

  const heightCm = (record.snapshot.pivotM ?? 0.99) * 100;
  for (; trajectory.read < record.rows.length; trajectory.read++) {
    const row = parseTelemetry(record.rows[trajectory.read]);
    if (row?.state !== 1 || row.problem !== 1 || Math.abs(row.angle) >= 90 || Math.abs(row.roll) >= 90) continue;
    const roll = row.roll / degreesPerRadian;
    const x = heightCm * Math.tan(row.angle / degreesPerRadian) / Math.cos(roll);
    const y = heightCm * Math.tan(roll);
    const point = { x, y, t: row.runTime / 1000, pitch: row.angle, rate: row.rate };
    updateCycles(trajectory, point);
    trajectory.points.push(point);
    trajectory.xExtent = Math.max(trajectory.xExtent, Math.abs(x));
    trajectory.yExtent = Math.max(trajectory.yExtent, Math.abs(y));
  }

  const { points, cycles, xExtent, yExtent } = trajectory;
  byId("trial-trajectory").hidden = false;
  byId("trajectory-trial").textContent = trialLabel(record.id) + (record === active ? " · 采集中" : "");
  byId("trajectory-empty").hidden = points.length > 0;
  byId("trajectory-layout").hidden = points.length === 0;
  if (!points.length) return;

  const targetHalfSpan = heightCm * Math.tan(record.snapshot.targetDeg / degreesPerRadian);
  const xLimit = Math.ceil(Math.max(4, targetHalfSpan, xExtent) / 4) * 4;
  const yLimit = Math.ceil(Math.max(1, yExtent));
  chart(byId("chart-trajectory"), [{ points, value: "y", color: measuredColor }],
    { start: -xLimit, end: xLimit, min: -yLimit, max: yLimit, positionPlot: true, zeroLine: true });
  chart(byId("chart-displacement"), [{ points, value: "x", color: measuredColor }],
    { end: Math.max(1, points.at(-1).t), min: -xLimit, max: xLimit, zeroLine: true });
  renderCycles(cycles, record === active);
}

const limits = { angle: [-20, 20], rate: [-80, 80], amplitude: [0, 20], pwm: [0, 100] };

function updateComparison() {
  comparison = [...selected].map(id => {
    const record = history.get(id);
    const points = record.rows.map(parseTelemetry).filter(row => row?.state === 1)
      .map(row => ({ ...row, t: row.runTime / 1000 }));
    return { record, points };
  });
  byId("compare").hidden = !comparison.length;
  const labels = byId("compare-labels");
  labels.replaceChildren();
  comparison.forEach(({ record }, index) => {
    const label = document.createElement("span");
    const dot = document.createElement("i");
    dot.className = index ? "trace-secondary" : "trace-measured";
    label.append(dot, trialLabel(record.id) + " " + record.snapshot.targetDeg + "° ");
    labels.append(label);
  });
  drawComparison();
}

function drawComparison() {
  if (!comparison.length) return;
  const metric = byId("compare-metric").value;
  const [min, max] = metric === "pwm" ? [-100, 100] : limits[metric];
  const end = Math.max(1, ...comparison.map(({ points }) => points.at(-1)?.t ?? 0));
  chart(byId("chart-compare"), comparison.map(({ points }, index) =>
    ({ points, value: metric, color: index ? secondaryColor : measuredColor })), { end, min, max });
}

function drawLive() {
  const row = live.at(-1);
  const end = Math.max(12, row?.t ?? 0);
  const start = end - 12;
  const visible = live.filter(point => point.t >= start - .1);
  const draw = (metric, first, second, color, dashed) => {
    const [min, max] = limits[metric];
    chart(byId("chart-" + metric), [
      { points: visible, value: first, color: measuredColor },
      { points: visible, value: second, color, dashed },
    ], { start, end, min, max });
  };
  draw("angle", "angle", "targetAngle", referenceColor, true);
  draw("rate", "rate", "targetRate", referenceColor, true);
  draw("amplitude", "amplitude", "targetAmplitude", referenceColor, true);
  draw("pwm", "pwm1", "pwm2", secondaryColor, false);
  byId("angle").textContent = row ? row.angle.toFixed(2) + " °" : "—";
  byId("rate").textContent = row ? row.rate.toFixed(1) + " °/s" : "—";
  byId("amplitude").textContent = Number.isFinite(row?.amplitude) ? row.amplitude.toFixed(2) + " °" : "—";
  byId("pwm").textContent = row ? row.pwm1.toFixed(1) + " / " + row.pwm2.toFixed(1) + " %" : "—";
  const displayedRecord = active ?? (shownId
    ? [...history.values()].find(record => trialLabel(record.id) === shownId) : undefined);
  const runtimeLimit = displayedRecord?.snapshot.durationMs;
  const configuredSeconds = runtimeLimit === undefined ? undefined : runtimeLimit / 1000;
  const durationLabel = configuredSeconds === undefined ? "" : Number.isInteger(configuredSeconds)
    ? configuredSeconds.toFixed(0) : configuredSeconds.toFixed(1);
  byId("elapsed").textContent = row
    ? (row.runTime / 1000).toFixed(2) + (durationLabel ? " / " + durationLabel : "") + " s"
    : "—";
  byId("phase").textContent = row?.state === 1 && row.problem === 1
    ? phases[row.phase] ?? "Q1" : row ? states[row.state] : "等待遥测";
}

setInterval(() => {
  if (dirty) {
    drawLive();
    drawTrajectory();
    dirty = false;
  }
  if (active && performance.now() - lastRx > 2000) {
    finish("遥测中断，记录未完成");
    notify("超过 2 秒未收到遥测。可按板上 KEY3。", true);
  }
  updateControls();
}, 100);
setInterval(() => {
  if (active && typeof active.id === "number") void save(active);
}, 1000);
new ResizeObserver(() => {
  dirty = true;
  drawTrajectory(true);
  drawComparison();
}).observe(document.querySelector("main"));
byId("compare-metric").addEventListener("change", drawComparison);

const navigation = [...document.querySelectorAll(".navigation a")];
const overview = byId("overview");
const sections = navigation.slice(1).map(link => byId(link.hash.slice(1)));
const setup = overview.querySelector(".setup");

function updateNavigation() {
  const destination = navigation.find(link => link.hash === location.hash) ?? navigation[0];
  const viewingOverview = destination === navigation[0];
  const view = destination.hash.slice(1);
  overview.dataset.view = view;
  for (const section of sections) {
    section.hidden = !viewingOverview && section.id !== view;
  }
  setup.hidden = [...setup.children].every(section => section.hidden);
  for (const link of navigation) {
    if (link === destination) link.setAttribute("aria-current", "page");
    else link.removeAttribute("aria-current");
  }
  dirty = true;
  drawTrajectory(true);
  drawComparison();
}

window.addEventListener("hashchange", () => {
  updateNavigation();
  window.scrollTo(0, 0);
});
updateNavigation();
updateSpanLimits();
updateTargetAngle();
updateControls();
if (!navigator.serial) notify("请用 Chrome 或 Edge 打开 localhost 地址以连接 Web Serial。", true);
try {
  database = await openHistory();
  const records = await historyOperation(database, "readonly", store => store.getAll());
  for (const record of records) history.set(record.id, record);
  renderHistory();
} catch (error) {
  notify("无法打开浏览器实验存储 · " + error.message, true);
}
updateControls();

if (database && archivePickerSupported) {
  try {
    archiveSettings = await openArchiveSettings();
    const directory = await historyOperation(archiveSettings, "readonly",
      store => store.get("directory"), "settings");
    if (directory && await directory.queryPermission({ mode: "readwrite" }) === "granted") {
      archiveDirectory = directory;
      syncArchive();
    } else if (directory) {
      showArchiveStatus("请重新授权「实验数据」文件夹。");
    } else {
      showArchiveStatus("选择「实验数据」文件夹后，每轮结束自动归档。");
    }
  } catch (error) {
    showArchiveStatus("无法恢复文件夹授权，请选择数据文件夹。", true);
  } finally {
    archiveButton.disabled = false;
  }
} else if (!archivePickerSupported) {
  showArchiveStatus("请用 Chrome 或 Edge 打开本地地址以选择数据文件夹。");
}
