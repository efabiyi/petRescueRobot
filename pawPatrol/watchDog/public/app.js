async function fetchLogs() {
  const logDisplays = {
    drive: document.getElementById('driveLogs'),
    hall: document.getElementById('hallLogs'),
    laser: document.getElementById('laserLogs'),
  };

  const logs = {
    drive: [],
    hall: [],
    laser: [],
  };

  const eventSource = new EventSource('/stream');

  eventSource.onmessage = (event) => {
    const match = event.data.match(/^\[(.*?)\]\s*(.*)$/);
    if (!match) return;

    const sensor = match[1].toLowerCase();
    const message = match[2].replace(/\s*\|\s*/g, '\n');

    if (logDisplays[sensor]) {
      logs[sensor].push(message);
      if (logs[sensor].length > 25) logs[sensor].shift();
      logDisplays[sensor].textContent = logs[sensor].join('\n\n');
      logDisplays[sensor].scrollTop = logDisplays[sensor].scrollHeight;
    }
  };

  eventSource.onerror = (err) => {
    console.error("❌ Stream error:", err);
  };
}

fetchLogs();
