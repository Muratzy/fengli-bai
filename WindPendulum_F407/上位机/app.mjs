import { isClockReset, parseTelemetry, q4SnapshotFields, snapshotFields, tuningKeys } from "./protocol.mjs";
import { BluetoothGatt, Vcom } from "./serial.mjs";
import { archiveTrial, historyOperation, openArchiveSettings, openHistory,
  trialCsv, trialFilename } from "./storage.mjs";
import { projectTrialPoint, targetHalfSpanCm } from "./trial_geometry.mjs";
import { chart, measuredColor, referenceColor, secondaryColor } from "./charts.mjs";

const byId = id => document.getElementById(id);
const form = byId("parameters");
const targetSpan = form.elements.targetSpanCm;
const durationInput = form.elements.durationSeconds;
const gainKeys = tuningKeys.filter(key => key !== "targetDeg");
const degreesPerRadian = 180 / Math.PI;
const controls = Object.fromEntries(["connect", "start", "stop", "reset"].map(id => [id, byId(id)]));
const connectionMode = byId("connection-mode");
const problemButtons = [...byId("problem-switcher").querySelectorAll("[data-problem]")];
const problemFields = [...form.querySelectorAll("[data-problem-fields]")];
const problemNames = ["", "Q1/Q2 共用直线控制", "Q2 独立入口已弃用", "指定方向直线", "制动", "圆周轨迹", "抗风扰", "扩展题目"];
const states = ["待机", "运行中", "已完成", "已停止", "保护停机"];
const reasons = ["", "到时", "手动停止", "数据过期", "超角或超速", "未检测到换向",
  "EN 低电平", "启动条件不符", "配置无效", "已停稳", "等待释放超时", "5 秒内未停稳"];
const phases = ["启动 PD", "切换混合", "幅值 PID"];
const brakePhases = ["等待释放", "主动制动", "停稳观察"];
const terminal = row => row.state >= 2;
const trialLabel = (id, problem = 1) => "Q" + problem + "-" + String(id).padStart(3, "0");
const nowIso = () => new Date().toISOString();
const archiveButton = byId("archive-directory");
const archiveStatus = byId("archive-status");
const archivePickerSupported = typeof window.showDirectoryPicker === "function";

let database;
let serial;
let boardConfig;
let q3Info;
let q4Info;
let activeTransportMode;
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
let transportMode = localStorage.getItem("wind-pendulum-transport") || "jlink";
let selectedProblem = Number(localStorage.getItem("wind-pendulum-problem") || 1);
if (!["jlink", "bluetooth"].includes(transportMode)) transportMode = "jlink";
if (![1, 3, 4].includes(selectedProblem)) selectedProblem = 1;
connectionMode.value = transportMode;

function setSelectedProblem(problem, announce = true) {
  if (!problemNames[problem] || problem === 2 || active || starting || connecting) return;
  selectedProblem = problem;
  localStorage.setItem("wind-pendulum-problem", String(problem));
  const title = (problem === 1 ? "Q1/Q2" : "Q" + problem) + " 实验台";
  byId("page-title").textContent = title;
  document.title = "风力摆 · " + title;
  problemButtons.forEach(button => {
    button.setAttribute("aria-pressed", String(Number(button.dataset.problem) === problem));
  });
  problemFields.forEach(section => {
    section.hidden = Number(section.dataset.problemFields) !== problem;
    for (const field of section.querySelectorAll("input")) field.disabled = section.hidden;
  });
  const q1 = problem === 1;
  const q3 = problem === 3;
  byId("q1-duration-setting").hidden = !q1;
  if (problem > 4) {
    byId("problem-control-note").textContent = "Q" + problem + " 尚未接入控制器。";
  }
  controls.start.textContent = problem === 4 ? "预备 Q4" :
      q3 ? "通过蓝牙下发并启动 Q3" : "下发并启动 Q1/Q2";
  dirty = true;
  if (q3 && !serial) {
    transportMode = "bluetooth";
    connectionMode.value = transportMode;
    localStorage.setItem("wind-pendulum-transport", transportMode);
  }
  if (announce) notify("已切换到 Q" + problem + " · " + problemNames[problem] + "。" +
    (q3 ? "参数下发和启动使用蓝牙 GATT。" : ""));
  updateControls();
}

function notify(message, error = false) {
  byId("message").textContent = message;
  byId("message").classList.toggle("error", error);
}

function telemetryAgeLimit(periods, minimumMs) {
  const periodMs = (selectedProblem === 4 ? q4Info?.telemetryMs : undefined) ??
      boardConfig?.telemetryMs ?? (transportMode === "bluetooth" ? 1000 : 20);
  return Math.max(minimumMs, periods * periodMs);
}

function updateControls() {
  const fresh = latest && performance.now() - lastRx < telemetryAgeLimit(3, 1000);
  const idle = fresh && latest.state !== 1;
  const canConnect = transportMode === "bluetooth" ? Boolean(navigator.bluetooth) :
    Boolean(navigator.serial);
  controls.connect.textContent = serial ? "断开连接" :
    transportMode === "bluetooth" ? "连接蓝牙" : "连接 J-Link";
  controls.connect.disabled = connecting || (!canConnect && !serial);
  const readyMode = boardConfig?.mode === 1 && idle && [2, 7].includes(latest.mode);
  const q1Ready = selectedProblem === 1;
  const q3Ready = selectedProblem === 3 && activeTransportMode === "bluetooth" &&
      q3Info?.directionSign !== 0 && Boolean(q3Info);
  const q4Ready = selectedProblem === 4 && Boolean(q4Info?.directionSign) &&
      latest?.tilt <= q4Info.releaseMaxDeg;
  controls.start.disabled = !(serial && database && (readyMode && (q1Ready || q3Ready) ||
    q4Ready && idle && [2, 7].includes(latest.mode)) &&
    !starting && !active && !resetting && !stopping);
  if (selectedProblem === 3 && q3Info?.directionSign === 0)
    byId("problem-control-note").textContent = "Q3 上下电机推力方向未校准，启动已锁定。";
  else if (selectedProblem === 3 && serial && activeTransportMode !== "bluetooth")
    byId("problem-control-note").textContent = "请断开 J-Link，再选择蓝牙连接 Q3。";
  else if (selectedProblem === 3 && !q3Info && serial)
    byId("problem-control-note").textContent = "当前固件未确认 Q3 蓝牙调参协议。";
  else if (selectedProblem === 3)
    byId("problem-control-note").textContent = q3Info
      ? "运行窗口 " + q3Info.durationMs / 1000 + " 秒。"
      : "连接蓝牙后读取 Q3 能力和电机方向校准状态。";
  else if (selectedProblem === 4)
    byId("problem-control-note").textContent = q4Info?.directionSign === 0
      ? "Q4 上下电机推力方向未校准，预备已锁定。"
      : q4Info ? "预备后拉到 " + q4Info.releaseMinDeg + "–" + q4Info.releaseMaxDeg +
        "°，静止保持至少 " + q4Info.holdMs + " ms 再松手。检测到释放后开始计时。"
        : serial ? "当前固件未确认 Q4 预备协议。" : "连接后读取 Q4 制动参数。";
  byId("problem-control-note").hidden = selectedProblem === 1;
  controls.stop.disabled = !(serial && (boardConfig || q4Info) && !stopping);
  controls.reset.disabled = !(serial && (boardConfig || q4Info) && idle && !starting && !active && !resetting && !stopping);
  connectionMode.disabled = Boolean(serial || connecting || active);
  problemButtons.forEach(button => {
    button.disabled = Number(button.dataset.problem) === 2 || Boolean(starting || active);
  });
  for (const section of problemFields) {
    const hidden = Number(section.dataset.problemFields) !== selectedProblem;
    for (const field of section.querySelectorAll("input"))
      field.disabled = hidden || Boolean(starting || active);
  }
  durationInput.disabled = selectedProblem !== 1 || Boolean(starting || active ||
    (boardConfig && !boardConfig.maxDurationMs));
  const connected = byId("connection");
  connected.dataset.state = serial && fresh ? "ready" : "idle";
  const transportLabel = transportMode === "bluetooth" ? "蓝牙" : "J-Link";
  connected.textContent = connecting ? "正在连接" : !serial ?
    transportLabel + " 未连接" : !fresh ? transportLabel + (latest ? " · 遥测超时" : " · 等待遥测") :
    boardConfig ? transportLabel + " · 在线" : transportLabel + " · 只读";
  const problemLabel = latest?.problem > 0 ? "Q" + latest.problem + " · " : "";
  byId("board-state").textContent = fresh
    ? (latest.mode <= 1 ? "IMU 未就绪" : problemLabel + states[latest.state]) : "—";
  byId("run-id").textContent = shownId ?? "—";
  byId("serial-summary").textContent = selectedProblem === 3 ? "Q3 调试 · 仅 BLE" :
    (selectedProblem === 4 ? "Q4 制动 · " : "Q1/Q2 共用控制 · ") +
    (transportMode === "bluetooth" ? "BLE" : "J-Link");
  byId("serial-settings").textContent = transportMode === "bluetooth"
    ? "USART6 · 9600 · 8-N-1" : "115200 · 8-N-1";
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

async function refreshQ3Info(connection = serial) {
  q3Info = undefined;
  if (!connection || activeTransportMode !== "bluetooth") {
    updateControls();
    return;
  }
  try {
    const reply = await connection.request("Q3INFO");
    if (serial !== connection) return;
    q3Info = reply.q3Info;
    if (q3Info) {
      form.elements.q3LineLengthCm.min = q3Info.minLengthMm / 10;
      form.elements.q3LineLengthCm.max = q3Info.maxLengthMm / 10;
      form.elements.q3DirectionDeg.step = q3Info.directionStepDeg;
    }
  } catch (error) {
    if (serial === connection && selectedProblem === 3)
      notify("题目能力查询失败 · 请确认固件包含 BLE 启动协议。", true);
  }
  updateControls();
}

async function refreshQ4Info(connection = serial) {
  q4Info = undefined;
  updateControls();
  try {
    const reply = await connection.request("Q4INFO");
    if (serial === connection) {
      q4Info = reply.snapshot;
      byId("q4-pd").textContent = "双轴 PD · Kp " + q4Info.brakeKp + " · Kd " +
        q4Info.brakeKd + " · 总 PWM 上限 " + q4Info.pwmPercent + "%";
    }
  } catch (error) {
    if (serial === connection && selectedProblem === 4)
      notify("Q4 能力查询失败 · " + error.message, true);
  }
  updateControls();
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
  if (record.problemId === 4) record.endRunMs = row?.runTime;
  void save(record);
  syncArchive();
  renderHistory();
  updateControls();
}

function addTrialFrame(row) {
  if (row.problem !== active.problemId || row.trialId !== active.commandId) return;
  active.rows.push(row.line);
  if (terminal(row)) {
    const status = reasons[row.reason] || states[row.state];
    finish(status, row);
    notify("本轮结束 · " + status);
  }
}

function receiveTelemetry(line) {
  const direction = active?.snapshot.directionDeg ?? starting?.directionDeg;
  const row = parseTelemetry(line, direction);
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
    if (row.problem === starting.problemId && row.trialId !== 0) starting.frames.push(row);
  } else if (active) {
    addTrialFrame(row);
  }
  dirty = true;
  updateControls();
}

async function disconnect(message, isError = false) {
  const current = serial;
  serial = undefined;
  activeTransportMode = undefined;
  boardConfig = undefined;
  q3Info = undefined;
  q4Info = undefined;
  byId("q4-pd").textContent = "连接后读取板端制动参数。";
  if (selectedProblem === 3) {
    transportMode = "bluetooth";
    connectionMode.value = transportMode;
    localStorage.setItem("wind-pendulum-transport", transportMode);
  }
  updateDurationSetting();
  updateSpanLimits();
  updateTargetAngle();
  latest = undefined;
  lastRx = 0;
  originTime = undefined;
  if (active) finish("连接断开，记录未完成");
  await current?.close();
  notify(message, isError);
  updateControls();
}

connectionMode.addEventListener("change", async () => {
  if (serial || connecting || active) {
    connectionMode.value = transportMode;
    notify("实验或连接进行中，不能切换连接方式。", true);
    return;
  }
  if (selectedProblem === 3 && connectionMode.value !== "bluetooth") {
    connectionMode.value = "bluetooth";
    notify("Q3 调试使用蓝牙 GATT。", true);
    return;
  }
  transportMode = connectionMode.value;
  localStorage.setItem("wind-pendulum-transport", transportMode);
  updateControls();
  const label = transportMode === "bluetooth" ? "蓝牙 GATT" : "J-Link VCOM";
  notify(label + " 已选中。点击连接开始监测。");
});

problemButtons.forEach(button => button.addEventListener("click", () => {
  setSelectedProblem(Number(button.dataset.problem));
  if (selectedProblem === 3 && serial && activeTransportMode === "bluetooth")
    void refreshQ3Info();
  if (selectedProblem === 4 && serial) void refreshQ4Info();
}));

controls.connect.addEventListener("click", async () => {
  if (serial) {
    await disconnect(transportMode === "bluetooth" ? "蓝牙已断开。" : "J-Link 已断开。");
    return;
  }
  connecting = true;
  updateControls();
  let current;
  try {
    if (transportMode === "bluetooth") {
      current = new BluetoothGatt();
      await current.connect(receiveTelemetry, error => {
        if (serial === current) void disconnect("蓝牙断开 · " + error.message, true);
      });
    } else {
      const port = await navigator.serial.requestPort();
      current = new Vcom();
      await current.connect(port, receiveTelemetry, error => {
        if (serial === current) void disconnect("J-Link 断开 · " + error.message, true);
      });
    }
    serial = current;
    activeTransportMode = transportMode;
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
      if (selectedProblem === 3 && activeTransportMode === "bluetooth") {
        await refreshQ3Info(current);
        if (!q3Info) notify("蓝牙已连接，但固件未确认 Q3 启动协议。", true);
        else if (selectedProblem === 3 && q3Info.directionSign === 0)
          notify("蓝牙已连接，Q3 推力方向尚未校准。", true);
        else notify("蓝牙已连接，题目启动能力已确认。");
      } else if (selectedProblem !== 4) {
        notify("STM32 已连接。参数随下次启动下发。");
      }
    } catch (error) {
      if (serial === current) notify("连接已打开，INFO 失败 · " + error.message, true);
    }
    if (serial === current && selectedProblem === 4) {
      await refreshQ4Info(current);
      if (q4Info?.directionSign) notify("STM32 已连接，可以预备 Q4。");
    }
  } catch (error) {
    await current?.close();
    if (error.name !== "NotFoundError") notify("无法打开连接 · " + error.message, true);
  } finally {
    connecting = false;
    updateControls();
  }
});

async function startTrial() {
  if (controls.start.disabled) return;
  if (!form.checkValidity()) {
    const section = durationInput.disabled || durationInput.validity.valid ? "tuning" : "experiment";
    if (byId(section).hidden) {
      location.hash = section;
      updateNavigation();
    }
    form.reportValidity();
    return;
  }
  const q3 = selectedProblem === 3;
  const q4 = selectedProblem === 4;
  const tuning = q4 ? undefined : q3 ? {
    startupKp: form.elements.q3StartupKp.valueAsNumber,
    startupKd: form.elements.q3StartupKd.valueAsNumber,
    amplitudeKp: form.elements.q3AmplitudeKp.valueAsNumber,
    amplitudeKi: form.elements.q3AmplitudeKi.valueAsNumber,
    amplitudeKd: form.elements.q3AmplitudeKd.valueAsNumber,
    compensation: form.elements.q3Compensation.valueAsNumber,
  } : Object.fromEntries(gainKeys.map(key => [key, form.elements[key].valueAsNumber]));
  const lineLengthMm = q3 ? Math.round(form.elements.q3LineLengthCm.valueAsNumber * 10) : undefined;
  const directionDeg = q3 ? form.elements.q3DirectionDeg.valueAsNumber : undefined;
  if (tuning) {
    tuning.targetDeg = q3
      ? Math.round(Math.atan((lineLengthMm / 2000) / (boardConfig.pivotM ?? 0.99)) * 180000 / Math.PI) / 1000
      : targetAngle();
    tuning.durationMs = q3 ? q3Info.durationMs :
      (boardConfig.maxDurationMs ? Math.round(durationInput.valueAsNumber * 1000) : undefined);
  }
  const problemOptions = q3 ? { id: 3, directionDeg, lineLengthMm } : null;
  const current = serial;
  const capture = { frames: [], problemId: selectedProblem, directionDeg };
  starting = capture;
  updateControls();
  const startedAt = nowIso();
  let accepted;
  try {
    accepted = await current.request(q4 ? "Q4ARM" : "START", tuning, problemOptions);
    if (tuning?.durationMs !== undefined && accepted.snapshot.durationMs !== tuning.durationMs) {
      throw new Error("STM32 确认的运行时长与设定不一致");
    }
    if (q3 && (accepted.snapshot.problemId !== 3 || accepted.snapshot.directionDeg !== directionDeg ||
        accepted.snapshot.lineLengthMm !== lineLengthMm)) {
      throw new Error("STM32 确认的 Q3 方向或长度与设定不一致");
    }
    const snapshot = { ...accepted.snapshot, problemId: selectedProblem };
    const record = {
      protocol: "WP1",
      problemId: selectedProblem,
      commandId: accepted.id,
      startedAt,
      acceptedAt: nowIso(),
      snapshot,
      rows: [],
      status: "采集中",
    };
    try {
      record.id = await historyOperation(database, "readwrite", store => store.add(record));
    } catch (error) {
      record.id = "临时-" + accepted.id;
    }
    shownId = typeof record.id === "number" ? trialLabel(record.id, record.problemId) : record.id;
    history.set(record.id, record);
    active = record;
    trajectoryRecord = record;
    starting = undefined;
    for (const row of capture.frames) {
      if (!active) break;
      addTrialFrame(row);
    }
    if (active && serial !== current) finish("连接断开，记录未完成");
    else if (active) {
      notify(typeof record.id === "number" ? shownId + (q4 ? " 已预备，可以拉摆。" : " 已启动，参数已归档。") :
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
form.elements.q3DirectionDeg.addEventListener("input", () => {
  const angle = form.elements.q3DirectionDeg.valueAsNumber;
  byId("q3-direction-hint").textContent = Number.isFinite(angle)
    ? "方向 " + angle + "° · 上 0° · 右 90° · 下 180° · 左 270°"
    : "上 0° · 右 90° · 下 180° · 左 270°";
});

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
  byId("snapshot-title").textContent = trialLabel(record.id, record.problemId) + " 参数快照";
  const container = byId("snapshot-fields");
  container.replaceChildren();
  for (const [key, label] of record.problemId === 4 ? q4SnapshotFields : snapshotFields) {
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
  byId(record.problemId === 4 ? "trial-brake" : "trial-trajectory").scrollIntoView({ block: "nearest" });
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
    check.setAttribute("aria-label", "对比 " + trialLabel(record.id, record.problemId));
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
    trial.textContent = trialLabel(record.id, record.problemId);
    trial.setAttribute("aria-label", "查看 " + trialLabel(record.id, record.problemId) + " 轨迹");
    trial.setAttribute("aria-pressed", String(record === trajectoryRecord));
    trial.setAttribute("aria-controls", "trial-trajectory");
    trial.addEventListener("click", () => selectTrajectory(record));
    row.insertCell().append(trial);
    row.insertCell().textContent = new Date(record.startedAt).toLocaleString("zh-CN", { hour12: false });
    row.insertCell().textContent = record.status === "采集中" && active !== record
      ? "记录中断" : record.status + (record.endReason === 9 ? " · " +
        (record.endRunMs / 1000).toFixed(2) + " s" : "");
    const parameters = row.insertCell();
    const snapshotButton = document.createElement("button");
    snapshotButton.textContent = record.problemId === 4 ? "查看 · 制动 PD" : record.problemId === 3
      ? "查看 · " + record.snapshot.lineLengthMm / 10 + " cm · " + record.snapshot.directionDeg + "°"
      : "查看 · " + record.snapshot.targetDeg + "°";
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
  const problemId = record.problemId ?? record.snapshot.problemId ?? 1;
  byId("trial-brake").hidden = problemId !== 4;
  if (problemId === 4) {
    byId("trial-trajectory").hidden = true;
    if (trajectory?.record === record && trajectory.read === record.rows.length && !force) return;
    const points = record.rows.map(line => parseTelemetry(line)).filter(Boolean);
    for (const point of points) point.t = ((point.time - points[0].time) >>> 0) / 1000;
    const result = record.endReason === 9 ? " · " + (record.endRunMs / 1000).toFixed(2) + " s" : "";
    byId("brake-result").textContent = trialLabel(record.id, 4) + " · " + record.status + result;
    const end = Math.max(1, points.at(-1)?.t ?? 0);
    const maxGap = 1.5 * record.snapshot.telemetryMs / 1000;
    chart(byId("chart-brake-angle"), [{ points, value: "angle", color: measuredColor }],
      { end, min: 0, max: 50, maxGap });
    chart(byId("chart-brake-rate"), [{ points, value: "rate", color: measuredColor }],
      { end, min: 0, max: 240, maxGap });
    trajectory = { record, read: record.rows.length };
    return;
  }
  if (trajectory?.record !== record) {
    trajectory = {
      record, read: 0, points: [], cycles: [],
      xExtent: 0, yExtent: 0,
    };
    force = true;
  }
  if (!force && trajectory.read === record.rows.length) return;

  const snapshot = { ...record.snapshot, problemId };
  for (; trajectory.read < record.rows.length; trajectory.read++) {
    const row = parseTelemetry(record.rows[trajectory.read], snapshot.directionDeg);
    if (row?.problem !== problemId) continue;
    const point = projectTrialPoint(row, snapshot);
    if (!point) continue;
    updateCycles(trajectory, point);
    trajectory.points.push(point);
    trajectory.xExtent = Math.max(trajectory.xExtent, Math.abs(point.x));
    trajectory.yExtent = Math.max(trajectory.yExtent, Math.abs(point.y));
  }

  const { points, cycles, xExtent, yExtent } = trajectory;
  byId("trial-trajectory").hidden = false;
  byId("trajectory-trial").textContent = trialLabel(record.id, record.problemId) +
    (record === active ? " · 采集中" : "");
  byId("trajectory-empty").hidden = points.length > 0;
  byId("trajectory-layout").hidden = points.length === 0;
  if (!points.length) return;

  const targetHalfSpan = targetHalfSpanCm(snapshot);
  const maxGap = Math.max(0.15, (snapshot.telemetryMs ?? 20) * 1.5 / 1000);
  byId("trajectory-cross-label").textContent = problemId === 3
    ? "↑ 垂直方向偏移（cm）" : "↑ 上下偏移（cm）";
  byId("trajectory-along-label").textContent = problemId === 3
    ? "设定方向位移（cm）→" : "左右位移（cm）→";
  byId("displacement-label").textContent = problemId === 3
    ? "↑ 设定方向位移（cm）" : "↑ 左右摆动位移（cm）";
  const xLimit = Math.ceil(Math.max(4, targetHalfSpan, xExtent) / 4) * 4;
  const yLimit = Math.ceil(Math.max(1, yExtent));
  chart(byId("chart-trajectory"), [{ points, value: "y", color: measuredColor }],
    { start: -xLimit, end: xLimit, min: -yLimit, max: yLimit, positionPlot: true, zeroLine: true, maxGap });
  chart(byId("chart-displacement"), [{ points, value: "x", color: measuredColor }],
    { end: Math.max(1, points.at(-1).t), min: -xLimit, max: xLimit, zeroLine: true, maxGap });
  renderCycles(cycles, record === active);
}

const limits = { angle: [-20, 20], rate: [-80, 80], amplitude: [0, 20], pwm: [0, 100] };

function updateComparison() {
  comparison = [...selected].map(id => {
    const record = history.get(id);
    const points = record.rows.map(line => parseTelemetry(line, record.snapshot.directionDeg))
      .filter(row => row?.state === 1 && (record.problemId !== 4 || row.phase !== 0))
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
    const value = record.problemId === 4 ? "制动" : record.problemId === 3
      ? record.snapshot.lineLengthMm / 10 + " cm · " + record.snapshot.directionDeg + "°"
      : record.snapshot.targetDeg + "°";
    label.append(dot, trialLabel(record.id, record.problemId) + " " + value + " ");
    labels.append(label);
  });
  drawComparison();
}

function drawComparison() {
  if (!comparison.length) return;
  const metric = byId("compare-metric").value;
  const hasBrake = comparison.some(({ record }) => record.problemId === 4);
  const [min, max] = metric === "pwm" ? [-100, 100] :
    hasBrake && metric === "angle" ? [-20, 50] : hasBrake && metric === "rate" ? [-80, 240] : limits[metric];
  const end = Math.max(1, ...comparison.map(({ points }) => points.at(-1)?.t ?? 0));
  chart(byId("chart-compare"), comparison.map(({ points, record }, index) =>
    ({ points, value: metric, color: index ? secondaryColor : measuredColor,
      maxGap: Math.max(0.15, (record.snapshot.telemetryMs ?? 20) * 1.5 / 1000) })),
    { end, min, max });
}

function drawLive() {
  const row = live.at(-1);
  const q4 = selectedProblem === 4 || row?.problem === 4;
  const end = Math.max(12, row?.t ?? 0);
  const start = end - 12;
  const visible = live.filter(point => point.t >= start - .1);
  const draw = (metric, traces, [min, max] = limits[metric]) => {
    chart(byId("chart-" + metric), traces.map(([value, color, dashed]) =>
      ({ points: visible, value, color, dashed })),
    { start, end, min, max, maxGap: telemetryAgeLimit(1.5, 150) / 1000 });
  };
  const q3 = row?.projected ?? (selectedProblem === 3 && !row);
  byId("angle-title").textContent = q4 ? "倾角" : q3 ? "沿线角度" : "Pitch 角度";
  byId("rate-title").textContent = q4 ? "合成角速度" : q3 ? "沿线角速度" : "Pitch 角速度";
  byId("amplitude-title").textContent = q4 ? "双轴角速度" : "估算幅值";
  byId("amplitude-legend").hidden = q4;
  byId("axis-rate-legend").hidden = !q4;
  byId("vertical-pwm-legend").hidden = !q4;
  draw("angle", q4 ? [["tilt", measuredColor]] :
    [["angle", measuredColor], ["targetAngle", referenceColor, true]], q4 ? [0, 50] : undefined);
  draw("rate", q4 ? [["speed", measuredColor]] :
    [["rate", measuredColor], ["targetRate", referenceColor, true]], q4 ? [0, 240] : undefined);
  draw("amplitude", q4 ? [["rollRateDps", measuredColor], ["pitchRateDps", secondaryColor]] :
    [["amplitude", measuredColor], ["targetAmplitude", referenceColor, true]], q4 ? [-240, 240] : undefined);
  const pwmTraces = [["pwm1", measuredColor], ["pwm2", secondaryColor]];
  if (q4) pwmTraces.push(["pwm3", "#89cf87"], ["pwm4", "#f0a55e"]);
  draw("pwm", pwmTraces);
  byId("angle").textContent = row ? (q4 ? row.tilt : row.angle).toFixed(2) + " °" : "—";
  byId("rate").textContent = row ? (q4 ? row.speed : row.rate).toFixed(1) + " °/s" : "—";
  byId("amplitude").textContent = q4 && row ? row.rollRateDps.toFixed(1) + " / " +
    row.pitchRateDps.toFixed(1) + " °/s" : Number.isFinite(row?.amplitude) ? row.amplitude.toFixed(2) + " °" : "—";
  byId("pwm").textContent = row ? row.pwm1.toFixed(1) + " / " + row.pwm2.toFixed(1) + " %" +
    (row.problem === 3 || q4 ? " · PWM3/4 " + row.pwm3.toFixed(1) + " / " + row.pwm4.toFixed(1) + " %" : "") : "—";
  const displayedRecord = active ?? (shownId
    ? [...history.values()].find(record => trialLabel(record.id, record.problemId) === shownId) : undefined);
  const runtimeLimit = row?.state === 1 ? displayedRecord?.snapshot.durationMs : undefined;
  const configuredSeconds = runtimeLimit === undefined ? undefined : runtimeLimit / 1000;
  const durationLabel = configuredSeconds === undefined ? "" : Number.isInteger(configuredSeconds)
    ? configuredSeconds.toFixed(0) : configuredSeconds.toFixed(1);
  byId("elapsed").textContent = row
    ? (row.runTime / 1000).toFixed(2) + (durationLabel ? " / " + durationLabel : "") + " s"
    : "—";
  const showPhase = row?.state === 1 && (row.problem === 1 || row.problem === 3);
  byId("phase").textContent = row?.problem === 4 && row.state === 1 ? brakePhases[row.phase] :
    showPhase ? phases[row.phase] ?? states[row.state] : row ? states[row.state] : "等待遥测";
}

setInterval(() => {
  if (dirty) {
    drawLive();
    drawTrajectory();
    dirty = false;
  }
  const trialTimeoutMs = telemetryAgeLimit(5, 2000);
  if (active && performance.now() - lastRx > trialTimeoutMs) {
    finish("遥测中断，记录未完成");
    notify("超过 " + trialTimeoutMs / 1000 + " 秒未收到遥测。可按板上 KEY3。", true);
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
setSelectedProblem(selectedProblem, false);
updateNavigation();
updateSpanLimits();
updateTargetAngle();
updateControls();
if (!navigator.serial && !navigator.bluetooth) {
  notify("当前浏览器不支持 J-Link 串口或蓝牙 GATT。", true);
} else {
  notify((transportMode === "bluetooth" ? "蓝牙 GATT" : "J-Link VCOM") +
    " 已选中。点击连接开始监测。");
}
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
