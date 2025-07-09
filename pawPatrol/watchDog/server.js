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

// POST endpoint for ESP32 to send logs
app.post('/log', (req, res) => {
  const logEntry = `${new Date().toISOString()} - ${req.body}`;

  // Update in-memory logs
  logs.push(logEntry);
  if (logs.length > 100) logs.shift();

  // Write full logs array to file
  fs.writeFile(LOG_FILE, logs.join('\n') + '\n', err => {
    if (err) {
      console.error('Failed to write logs:', err);
    }
  });

  // Stream to connected clients (SSE)
  clients.forEach(client => client.write(`data: ${logEntry}\n\n`));

  console.log("📥", logEntry);
  res.sendStatus(200);
});

// SSE endpoint for dashboard clients
app.get('/stream', (req, res) => {
  // Set SSE headers
  res.setHeader('Content-Type', 'text/event-stream');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Connection', 'keep-alive');
  res.flushHeaders();

  // Send existing logs right away (optional)
  logs.forEach(log => res.write(`data: ${log}\n\n`));

  // Add client to list
  clients.push(res);
  console.log("🧑‍💻 Client connected, total:", clients.length);

  // Remove client on disconnect
  req.on('close', () => {
    clients = clients.filter(c => c !== res);
    console.log("❌ Client disconnected, total:", clients.length);
  });
});

// Start server
app.listen(PORT, '0.0.0.0', () => {
  console.log(`✅ Server running at http://0.0.0.0:${PORT}`);
});
