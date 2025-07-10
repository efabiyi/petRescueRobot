async function fetchLogs() {
  const logDisplay = document.getElementById('reflectanceLogs');
  const logs = [];

  const eventSource = new EventSource('/stream');

  eventSource.onmessage = (event) => {
    const formatted = event.data.replace(/\s*\|\s*/g, '\n')
    logs.push(formatted);

    if (logs.length > 25) {
      logs.shift(); // Keep only the last 1000 logs
    }

    logDisplay.textContent = logs.join('\n\n');
    logDisplay.scrollTop = logDisplay.scrollHeight; // Auto-scroll to bottom

  };

  eventSource.onerror = (err) => {
    console.error("❌ Stream error:", err);
  };

}

setInterval(fetchLogs, 500);
fetchLogs();
