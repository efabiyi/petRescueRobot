const express = require('express');
const bodyParser = require('body-parser');
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = 5000;
const LOG_FILE = 'logs/logs.txt';

let logs = [];
let clients = [];

// Load existing logs from file on startup
try {
  const fileData = fs.readFileSync(LOG_FILE, 'utf-8')
    .split('\n')
    .filter(line => line.trim() !== '');
  logs = fileData.slice(-100); // Keep only last 100 entries
  console.log(`📂 Loaded ${logs.length} logs from ${LOG_FILE}`);
} catch (err) {
  console.log(`ℹ️ No existing log file found, starting fresh.`);
}

// Middleware
app.use(bodyParser.text());
app.use(express.static(path.join(__dirname, 'public')));

// Rate limiting middleware
let lastSentTime = 0;
const RATE_LIMIT_MS = 0;

// POST endpoint for ESP32 to send logs
app.post('/log', (req, res) => {
  const logEntry = `${req.body}`.trim();

  const now = Date.now();
  if (now - lastSentTime < RATE_LIMIT_MS) {
    return res.sendStatus(429); // Too Many Requests
  }

  lastSentTime = now;

  // 🔧 Split multiple logs if block-delivered
  const entries = logEntry.split('\n').map(line => line.trim()).filter(Boolean);

  for (const entry of entries) {
    logs.push(entry);
    if (logs.length > 100) logs.shift();

    // 🔧 Send each line as a separate SSE event
    for (const client of clients) {
      client.write(`data: ${entry}\n\n`);
    }

    console.log("📥", entry);
  }

  // Update the log file
  fs.writeFile(LOG_FILE, logs.join('\n') + '\n', err => {
    if (err) {
      console.error('Failed to write logs:', err);
    }
  });

  res.sendStatus(200);
});

// SSE endpoint
app.get('/stream', (req, res) => {
  res.setHeader('Content-Type', 'text/event-stream');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Connection', 'keep-alive');
  res.flushHeaders();

  // 🔧 Send stored logs individually
  for (const log of logs) {
    res.write(`data: ${log}\n\n`);
  }

  clients.push(res);
  console.log("🧑‍💻 Client connected, total:", clients.length);

  req.on('close', () => {
    clients = clients.filter(c => c !== res);
    console.log("❌ Client disconnected, total:", clients.length);
  });
});

app.listen(PORT, '0.0.0.0', () => {
  console.log(`✅ Server running at http://0.0.0.0:${PORT}`);
});
