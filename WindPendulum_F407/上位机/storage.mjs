export function openHistory() {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open("wind-pendulum-q1", 1);
    request.onupgradeneeded = () => {
      request.result.createObjectStore("trials", { keyPath: "id", autoIncrement: true });
    };
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
}

export function historyOperation(db, mode, operation) {
  return new Promise((resolve, reject) => {
    const transaction = db.transaction("trials", mode);
    const request = operation(transaction.objectStore("trials"));
    transaction.oncomplete = () => resolve(request.result);
    transaction.onabort = () => reject(transaction.error || new Error("写入实验记录失败"));
  });
}
