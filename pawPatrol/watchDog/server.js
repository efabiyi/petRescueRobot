const express = require('express');
const bodyParser = require('body-parser');
const path = require('path');

const app = express();
const PORT = 5000;

let logs = [];
let clients = [];

app.use(bodyParser.text());
app.use(express.static(path.join(__dirname, 'public')));

// 🛰️ Handle ESP32 POSTs
app.post('/log', (req, res) => {
  const data = req.body;
  const logEntry = `${new Date().toISOString()} - ${data}`;
  logs.push(logEntry);
  if (logs.length > 1000) logs.shift();

  // 📡 Stream the log to all connected clients
  clients.forEach((client) => {
    client.write(`data: ${logEntry}\n\n`);
  });

  console.log("📥", logEntry);
  res.status(200).send('OK');
});

// 🔌 Streaming route for browser
app.get('/stream', (req, res) => {
  res.setHeader('Content-Type', 'text/event-stream');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Connection', 'keep-alive');
  res.flushHeaders();

  clients.push(res);
  console.log("🧑‍💻 Client connected to /stream");

  req.on('close', () => {
    console.log("❌ Client disconnected from /stream");
    clients = clients.filter(c => c !== res);
  });
});

app.listen(PORT, '0.0.0.0', () => {
  console.log(`✅ Server running at http://0.0.0.0:${PORT}`);
});
