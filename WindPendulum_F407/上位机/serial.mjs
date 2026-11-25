import { commandFrame, createLineReader, parseReply } from "./protocol.mjs";

const BAUD_RATE = 115200;
const BLUETOOTH_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb";
const BLUETOOTH_CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb";
const BLUETOOTH_WRITE_CHUNK = 20;

function boardError(reply, label) {
  if (!reply.error) return null;
  const message = {
    BUSY: "STM32 正在运行",
    NOT_READY: "STM32 未就绪，检查 IMU、角度和停止键",
    PARAMETERS: "STM32 拒绝参数，请检查范围",
  }[reply.error];
  const error = new Error(message || `${label}设备拒绝了 ${reply.error}`);
  error.boardRejected = true;
  return error;
}

function createProtocolChannel(write, onLine, onLost, label) {
  const pending = new Map();
  const decoder = new TextDecoder();
  let closed = false;
  let reportLost = true;
  const readLine = createLineReader(line => {
    const reply = parseReply(line);
    if (reply) {
      const request = pending.get(reply.id);
      if (request?.action === reply.action) request.resolve(reply);
    } else {
      onLine(line);
    }
  });

  function receive(bytes) {
    if (!closed) readLine(decoder.decode(bytes, { stream: true }));
  }

  function fail(error) {
    if (closed) return;
    closed = true;
    for (const { reject } of pending.values()) reject(error);
    pending.clear();
    if (reportLost) onLost(error);
  }

  async function request(action, tuning) {
    if (closed) throw new Error(`${label}未连接`);
    let id;
    do {
      id = crypto.getRandomValues(new Uint32Array(1))[0];
    } while (!id || pending.has(id));
    let timer;
    try {
      const reply = await new Promise((resolve, reject) => {
        pending.set(id, { action, resolve, reject });
        timer = setTimeout(() => reject(new Error("STM32 应答超时")), 1800);
        write(new TextEncoder().encode(commandFrame(id, action, tuning))).catch(reject);
      });
      const error = boardError(reply, label);
      if (error) throw error;
      return reply;
    } finally {
      clearTimeout(timer);
      pending.delete(id);
    }
  }

  function close() {
    if (closed) return;
    reportLost = false;
    closed = true;
    for (const { reject } of pending.values()) reject(new Error(`${label}已断开`));
    pending.clear();
  }

  return { receive, fail, request, close };
}

export class Vcom {
  #port;
  #reader;
  #writer;
  #channel;

  async connect(port, onLine, onLost) {
    this.#port = port;
    await port.open({ baudRate: BAUD_RATE, dataBits: 8, stopBits: 1,
      parity: "none", flowControl: "none" });
    this.#reader = port.readable.getReader();
    this.#writer = port.writable.getWriter();
    this.#channel = createProtocolChannel(
      bytes => this.#writer.write(bytes), onLine, onLost, "串口");
    void this.#listen();
  }

  async #listen() {
    const channel = this.#channel;
    try {
      while (true) {
        const { value, done } = await this.#reader.read();
        if (done) break;
        channel.receive(value);
      }
      channel.fail(new Error("串口输入已关闭"));
    } catch (error) {
      channel.fail(error);
    }
  }

  request(action, tuning) {
    return this.#channel.request(action, tuning);
  }

  async close() {
    this.#channel?.close();
    if (this.#reader) {
      await this.#reader.cancel().catch(() => {});
      this.#reader.releaseLock();
    }
    if (this.#writer) this.#writer.releaseLock();
    await this.#port?.close().catch(() => {});
  }
}

export class BluetoothGatt {
  #device;
  #server;
  #notifyCharacteristic;
  #writeCharacteristic;
  #channel;
  #onDisconnected;
  #onNotification;

  async connect(onLine, onLost) {
    if (!navigator.bluetooth) throw new Error("当前浏览器不支持 Web Bluetooth");
    const knownDevices = typeof navigator.bluetooth.getDevices === "function"
      ? await navigator.bluetooth.getDevices().catch(() => []) : [];
    this.#device = knownDevices.find(device => device.name === "WindPendulum") ??
      knownDevices.find(device => device.name?.startsWith("HC-05"));
    if (!this.#device) {
      this.#device = await navigator.bluetooth.requestDevice({
        acceptAllDevices: true,
        optionalServices: [BLUETOOTH_SERVICE_UUID],
      });
    }
    let connectedAt = performance.now();
    let notifications = 0;
    this.#onDisconnected = () => {
      const seconds = ((performance.now() - connectedAt) / 1000).toFixed(1);
      const error = new Error(`GATT 连接持续 ${seconds} 秒，收到 ${notifications} 个通知包`);
      if (this.#channel) this.#channel.fail(error);
      else onLost(error);
    };
    this.#device.addEventListener("gattserverdisconnected", this.#onDisconnected);
    this.#server = await this.#device.gatt.connect();
    connectedAt = performance.now();
    let service;
    try {
      service = await this.#server.getPrimaryService(BLUETOOTH_SERVICE_UUID);
    } catch (error) {
      throw new Error("所选设备没有 FFE0 服务，请选择 WindPendulum/HC-05 蓝牙模块");
    }
    const characteristics = await service.getCharacteristics();
    this.#notifyCharacteristic = characteristics.find(characteristic =>
      characteristic.uuid === BLUETOOTH_CHARACTERISTIC_UUID &&
      characteristic.properties.notify) ||
      characteristics.find(characteristic => characteristic.properties.notify);
    this.#writeCharacteristic = characteristics.find(characteristic =>
      characteristic.uuid === BLUETOOTH_CHARACTERISTIC_UUID &&
      (characteristic.properties.write || characteristic.properties.writeWithoutResponse)) ||
      characteristics.find(characteristic =>
        characteristic.properties.write || characteristic.properties.writeWithoutResponse);
    if (!this.#notifyCharacteristic) throw new Error("未找到 FFE1 通知特征");
    if (!this.#writeCharacteristic) throw new Error("未找到可写 GATT 特征");

    this.#channel = createProtocolChannel(
      bytes => this.#write(bytes), onLine, onLost, "蓝牙");
    this.#onNotification = event => {
      notifications++;
      this.#channel.receive(event.target.value);
    };
    this.#notifyCharacteristic.addEventListener("characteristicvaluechanged", this.#onNotification);
    await this.#notifyCharacteristic.startNotifications();
    if (!this.#server.connected) throw new Error("订阅通知时 GATT 已断开");
  }

  async #write(bytes) {
    const supportsResponse = this.#writeCharacteristic.properties.write;
    for (let offset = 0; offset < bytes.byteLength; offset += BLUETOOTH_WRITE_CHUNK) {
      const chunk = bytes.slice(offset, offset + BLUETOOTH_WRITE_CHUNK);
      if (supportsResponse) {
        await this.#writeCharacteristic.writeValueWithResponse(chunk);
      } else {
        await this.#writeCharacteristic.writeValueWithoutResponse(chunk);
      }
    }
  }

  request(action, tuning) {
    return this.#channel.request(action, tuning);
  }

  async close() {
    this.#channel?.close();
    if (this.#device && this.#onDisconnected) {
      this.#device.removeEventListener("gattserverdisconnected", this.#onDisconnected);
    }
    if (this.#notifyCharacteristic && this.#onNotification) {
      this.#notifyCharacteristic.removeEventListener("characteristicvaluechanged", this.#onNotification);
    }
    if (this.#server?.connected && this.#notifyCharacteristic) {
      await this.#notifyCharacteristic.stopNotifications().catch(() => {});
    }
    if (this.#device?.gatt?.connected) this.#device.gatt.disconnect();
  }
}

export const bluetoothConfig = {
  serviceUuid: BLUETOOTH_SERVICE_UUID,
  characteristicUuid: BLUETOOTH_CHARACTERISTIC_UUID,
};
