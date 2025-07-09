async function fetchLogs() {
  const logDisplay = document.getElementById('hallLogs');
  const logs = [];

  const eventSource = new EventSource('/stream');

  eventSource.onmessage = (event) => {
   logs.push(event.data);

  if (logs.length > 25) {
      logs.shift(); // Keep only the last 1000 logs
    }

  logDisplay.textContent = logs.join('\n');
  logDisplay.scrollTop = logDisplay.scrollHeight; // Auto-scroll to bottom
  
  };

  eventSource.onerror = (err) => {
    console.error("❌ Stream error:", err);
  };

}

setInterval(fetchLogs, 500);
fetchLogs();
