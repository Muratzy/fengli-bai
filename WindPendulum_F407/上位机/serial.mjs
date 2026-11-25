import { commandFrame, createLineReader, parseReply } from "./protocol.mjs";

export class Vcom {
  #port;
  #reader;
  #writer;
  #closed = false;
  #pending = new Map();

  async connect(port, onLine, onLost) {
    this.#port = port;
    await port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: "none", flowControl: "none" });
    this.#reader = port.readable.getReader();
    this.#writer = port.writable.getWriter();
    void this.#listen(onLine, onLost);
  }

  async #listen(onLine, onLost) {
    const decoder = new TextDecoder();
    const read = createLineReader(line => {
      const reply = parseReply(line);
      if (reply) {
        const request = this.#pending.get(reply.id);
        if (request?.action === reply.action) request.resolve(reply);
      } else {
        onLine(line);
      }
    });
    try {
      while (!this.#closed) {
        const { value, done } = await this.#reader.read();
        if (done) break;
        read(decoder.decode(value, { stream: true }));
      }
    } catch (error) {
      if (!this.#closed) onLost(error);
      return;
    }
    if (!this.#closed) onLost(new Error("串口输入已关闭"));
  }

  async request(action, tuning) {
    if (this.#closed || !this.#writer) throw new Error("串口未连接");
    let id;
    do {
      id = crypto.getRandomValues(new Uint32Array(1))[0];
    } while (!id || this.#pending.has(id));
    let timer;
    try {
      const reply = await new Promise((resolve, reject) => {
        this.#pending.set(id, { action, resolve, reject });
        timer = setTimeout(() => reject(new Error("STM32 应答超时")), 1200);
        this.#writer.write(new TextEncoder().encode(commandFrame(id, action, tuning))).catch(reject);
      });
      if (reply.error) {
        const message = { BUSY: "STM32 正在运行", NOT_READY: "STM32 未就绪，检查 IMU、角度和停止键",
          PARAMETERS: "STM32 拒绝参数，请检查范围" }[reply.error];
        const error = new Error(message || reply.error);
        error.boardRejected = true;
        throw error;
      }
      return reply;
    } finally {
      clearTimeout(timer);
      this.#pending.delete(id);
    }
  }

  async close() {
    if (this.#closed) return;
    this.#closed = true;
    for (const { reject } of this.#pending.values()) reject(new Error("串口已断开"));
    if (this.#reader) {
      await this.#reader.cancel().catch(() => {});
      this.#reader.releaseLock();
    }
    if (this.#writer) this.#writer.releaseLock();
    await this.#port?.close().catch(() => {});
  }
}
