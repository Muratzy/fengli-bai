import assert from "node:assert/strict";
import { BluetoothGatt } from "../上位机/serial.mjs";
import { snapshotFields } from "../上位机/protocol.mjs";

function crc16(payload) {
  let crc = 0xffff;
  for (const char of payload) {
    crc ^= char.charCodeAt(0) << 8;
    for (let bit = 0; bit < 8; bit++)
      crc = ((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0)) & 0xffff;
  }
  return crc.toString(16).padStart(4, "0").toUpperCase();
}

function frame(payload) {
  return new TextEncoder().encode(payload + "*" + crc16(payload) + "\r\n");
}

class FakeCharacteristic extends EventTarget {
  constructor(uuid, properties, onWrite) {
    super();
    this.uuid = uuid;
    this.properties = properties;
    this.onWrite = onWrite;
    this.received = [];
  }

  async startNotifications() { return this; }
  async stopNotifications() {}

  async writeValueWithResponse(bytes) {
    this.received.push(...bytes);
    if (bytes.includes(10)) {
      const line = new TextDecoder().decode(Uint8Array.from(this.received));
      this.received = [];
      this.onWrite(line);
    }
  }

  notify(bytes) {
    this.value = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    this.dispatchEvent(new Event("characteristicvaluechanged"));
  }
}

const writes = [];
let infoRequests = 0;
let ignoreStartReply = false;
let ignoreArmReply = false;
let notifyCharacteristic;
const writeCharacteristic = new FakeCharacteristic(
  "0000ffe1-0000-1000-8000-00805f9b34fb", { write: true }, line => {
    writes.push(line);
    const [, id, action] = line.trim().split("*")[0].split(",");
    if (action === "INFO") {
      infoRequests++;
      if (infoRequests === 2) {
        const values = Array(snapshotFields.length - 3).fill(0);
        values[26] = 1000;
        notifyCharacteristic.notify(frame(`!WP1,${id},OK,INFO,${values.join(",")}`));
      }
    } else if (action === "Q3INFO") {
      notifyCharacteristic.notify(frame(`!WP1,${id},OK,Q3INFO,3,0,200,600,15000,10`));
    } else if (action === "Q4INFO" || (action === "Q4ARM" && !ignoreArmReply)) {
      notifyCharacteristic.notify(frame(
        `!WP1,${id},OK,${action},4,-1,20000,5000,30,45,150,8000,2,5,300,50000,240000,10000,8000,850,50,1000`));
    } else if (action === "START" && !ignoreStartReply) {
      const values = Array(snapshotFields.length).fill(0);
      values.splice(0, 7, 28000, 7000, 2000, 0, 0, 500, 7180);
      values[9] = 15000;
      values[34] = 3;
      values[35] = 40;
      values[36] = 250;
      notifyCharacteristic.notify(frame(`!WP1,${id},OK,START,${values.join(",")}`));
    }
  });
notifyCharacteristic = new FakeCharacteristic(
  "0000ffe1-0000-1000-8000-00805f9b34fb", { notify: true }, () => {});

const service = { async getCharacteristics() { return [notifyCharacteristic, writeCharacteristic]; } };
const server = { connected: true, async getPrimaryService(uuid) {
  assert.equal(uuid, "0000ffe0-0000-1000-8000-00805f9b34fb");
  return service;
} };
const device = new EventTarget();
device.name = "WindPendulum";
device.gatt = { async connect() { return server; }, disconnect() { server.connected = false; } };
Object.defineProperty(globalThis, "navigator", {
  configurable: true,
  value: { bluetooth: {
    async getDevices() { return []; },
    async requestDevice(options) {
      assert.equal(options.acceptAllDevices, true);
      assert.deepEqual(options.optionalServices, ["0000ffe0-0000-1000-8000-00805f9b34fb"]);
      return device;
    },
  } },
});

const received = [];
const bluetooth = new BluetoothGatt();
await bluetooth.connect(line => received.push(line), error => { throw error; });
const capabilities = await bluetooth.request("Q3INFO");
assert.equal(capabilities.q3Info.directionSign, 0);
assert.equal(capabilities.q3Info.durationMs, 15000);
const tuning = {
  startupKp: 28, startupKd: 7, amplitudeKp: 2, amplitudeKi: 0,
  amplitudeKd: 0, compensation: 0.5, targetDeg: 7.18, durationMs: 15000,
};
const problem = { id: 3, directionDeg: 40, lineLengthMm: 250 };
const started = await bluetooth.request("START", tuning, problem);
assert.equal(started.snapshot.problemId, 3);
assert.equal(started.snapshot.directionDeg, 40);
assert.equal(started.snapshot.lineLengthMm, 250);
assert.match(writes[0], /,Q3INFO\*/);
assert.match(writes[1], /,15000,3,40,250\*/);
assert.ok(writes[1].length > 20);
const info = await bluetooth.request("INFO");
assert.equal(infoRequests, 2);
assert.equal(info.snapshot.telemetryMs, 1000);
ignoreStartReply = true;
const writesBeforeStart = writes.length;
await assert.rejects(bluetooth.request("START", tuning, problem), /STM32 应答超时/);
assert.equal(writes.length, writesBeforeStart + 1);
const q4Info = await bluetooth.request("Q4INFO");
const armed = await bluetooth.request("Q4ARM");
assert.deepEqual(armed.snapshot, q4Info.snapshot);
assert.equal(armed.snapshot.problemId, 4);
assert.match(writes.at(-1), /,Q4ARM\*/);
ignoreArmReply = true;
const writesBeforeArm = writes.length;
await assert.rejects(bluetooth.request("Q4ARM"), /STM32 应答超时/);
assert.equal(writes.length, writesBeforeArm + 1);
assert.deepEqual(received, []);
await bluetooth.close();
console.log("Web Bluetooth Q3 and Q4 protocol tests passed");
