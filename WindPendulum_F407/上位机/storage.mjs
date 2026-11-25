import { fields } from "./protocol.mjs";

function openStore(database, store, options) {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open(database, 1);
    request.onupgradeneeded = () => request.result.createObjectStore(store, options);
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
}

export function openHistory() {
  return openStore("wind-pendulum-q1", "trials", { keyPath: "id", autoIncrement: true });
}

export function openArchiveSettings() {
  return openStore("wind-pendulum-q1-storage", "settings");
}

export function historyOperation(db, mode, operation, store = "trials") {
  return new Promise((resolve, reject) => {
    const transaction = db.transaction(store, mode);
    const request = operation(transaction.objectStore(store));
    transaction.oncomplete = () => resolve(request.result);
    transaction.onabort = () => reject(transaction.error || new Error("写入实验记录失败"));
  });
}
export function trialCsv(record) {
  const count = record.rows[0]?.split(",").length ?? fields.length;
  return [fields.slice(0, count).join(","), ...record.rows, ""].join("\r\n");
}

export function trialFilename(record) {
  return "Q1-" + String(record.id).padStart(3, "0");
}

async function writeFile(directory, name, content) {
  const file = await directory.getFileHandle(name, { create: true });
  const writer = await file.createWritable();
  try {
    await writer.write(content);
    await writer.close();
  } catch (error) {
    await writer.abort().catch(() => {});
    throw error;
  }
}

export async function archiveTrial(directory, record) {
  const timestamp = record.startedAt.replace(/[:.]/g, "-");
  const prefix = [trialFilename(record), timestamp, record.commandId].join("_");
  await writeFile(directory, prefix + ".json", JSON.stringify(record, null, 2));
  await writeFile(directory, prefix + ".csv", trialCsv(record));
}
