(() => {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const elements = {
    connectionStatus: $("connectionStatus"), connectionText: $("connectionText"),
    connectButton: $("connectButton"), zeroButton: $("zeroButton"), zeroFeedback: $("zeroFeedback"),
    angleValue: $("angleValue"), turnLabel: $("turnLabel"), needle: $("needle"), dialProgress: $("dialProgress"),
    absoluteCount: $("absoluteCount"), zeroedCount: $("zeroedCount"), countBar: $("countBar"),
    agcValue: $("agcValue"), magnetStatus: $("magnetStatus"), parityErrors: $("parityErrors"),
    protocolErrors: $("protocolErrors"), sampleRate: $("sampleRate"), terminal: $("terminal"),
    pauseButton: $("pauseButton"), clearButton: $("clearButton"), streamDot: $("streamDot"),
    streamState: $("streamState"), lastSample: $("lastSample"), chart: $("angleChart"), toast: $("toast")
  };

  const state = {
    port: null, reader: null, connected: false, connecting: false, paused: false,
    lineBuffer: "", headers: [], history: [], lastSampleAt: 0, samplesThisSecond: 0,
    displayedRate: 0, toastTimer: null, demoTimer: null, demoZero: 0
  };
  const circumference = 2 * Math.PI * 126;

  function setConnection(mode, label) {
    state.connected = mode === "online";
    state.connecting = mode === "connecting";
    elements.connectionStatus.className = `status-pill ${mode}`;
    elements.connectionText.textContent = label;
    elements.connectButton.textContent = state.connected ? "Disconnect" : state.connecting ? "Connecting…" : "Connect Teensy";
    elements.connectButton.disabled = state.connecting;
    elements.zeroButton.disabled = !state.connected;
    elements.streamDot.classList.toggle("live", state.connected);
    elements.streamState.textContent = state.connected ? "Stream live" : "Stream idle";
    if (!state.connected) elements.zeroFeedback.textContent = "Connect the Teensy to enable zeroing.";
  }

  function showToast(message) {
    elements.toast.textContent = message;
    elements.toast.classList.add("show");
    clearTimeout(state.toastTimer);
    state.toastTimer = setTimeout(() => elements.toast.classList.remove("show"), 2400);
  }

  function addTerminalLine(line, type = "data") {
    if (state.paused) return;
    const row = document.createElement("div");
    row.className = `terminal-line ${type}`;
    const tag = document.createElement("span");
    tag.textContent = type === "system" ? "SYS" : type === "command" ? "CMD" : type === "error" ? "ERR" : "RX";
    const value = document.createElement("div");
    value.textContent = line;
    row.append(tag, value);
    elements.terminal.appendChild(row);
    while (elements.terminal.childElementCount > 240) elements.terminal.firstElementChild.remove();
    elements.terminal.scrollTop = elements.terminal.scrollHeight;
  }

  function parseSample(line) {
    if (line.startsWith("time_us,")) {
      state.headers = line.split(",");
      addTerminalLine(line, "system");
      return;
    }
    if (line.startsWith("#")) {
      const isError = line.includes("FAILED") || line.includes("UNKNOWN") || line.includes("TOO_LONG");
      addTerminalLine(line, isError ? "error" : "command");
      if (line.startsWith("# ZEROED,")) {
        elements.zeroFeedback.textContent = `Zero set at absolute count ${line.split(",")[1]}.`;
        showToast("Current position is now 0°");
      } else if (line.startsWith("# ZERO_FAILED")) {
        elements.zeroFeedback.textContent = "Zero failed—the encoder could not be read.";
        showToast("Could not zero encoder");
      }
      return;
    }

    addTerminalLine(line);
    const parts = line.split(",");
    let sample;
    if (parts.length >= 8) {
      sample = {
        timeUs: Number(parts[0]), absolute: Number(parts[1]), zeroed: Number(parts[2]),
        angle: Number(parts[3]), agc: Number(parts[4]), status: parts[5],
        parity: Number(parts[6]), protocol: Number(parts[7])
      };
    } else if (parts.length >= 7) {
      sample = {
        timeUs: Number(parts[0]), absolute: Number(parts[1]), zeroed: Number(parts[1]),
        angle: Number(parts[2]), agc: Number(parts[3]), status: parts[4],
        parity: Number(parts[5]), protocol: Number(parts[6])
      };
    }
    if (!sample || !Number.isFinite(sample.angle) || !Number.isFinite(sample.absolute)) return;
    updateDashboard(sample);
  }

  function updateDashboard(sample) {
    const angle = ((sample.angle % 360) + 360) % 360;
    elements.angleValue.textContent = angle.toFixed(2);
    elements.turnLabel.textContent = angle < 0.5 || angle > 359.5 ? "AT ZERO REFERENCE" : `${(angle / 360).toFixed(3)} REVOLUTION`;
    elements.needle.style.transform = `rotate(${angle}deg)`;
    elements.dialProgress.style.strokeDasharray = circumference.toFixed(2);
    elements.dialProgress.style.strokeDashoffset = (circumference * (1 - angle / 360)).toFixed(2);
    elements.absoluteCount.textContent = Math.round(sample.absolute).toLocaleString();
    elements.zeroedCount.textContent = Math.round(sample.zeroed).toLocaleString();
    elements.countBar.style.width = `${Math.max(0, Math.min(100, sample.absolute / 16383 * 100))}%`;
    elements.agcValue.textContent = Number.isFinite(sample.agc) ? Math.round(sample.agc) : "—";
    elements.magnetStatus.textContent = (sample.status || "UNKNOWN").replaceAll("_", " ");
    const healthy = sample.status === "OK";
    elements.magnetStatus.className = `health-chip ${healthy ? "healthy" : "warning"}`;
    elements.parityErrors.textContent = Number.isFinite(sample.parity) ? sample.parity : 0;
    elements.protocolErrors.textContent = Number.isFinite(sample.protocol) ? sample.protocol : 0;

    state.samplesThisSecond += 1;
    state.lastSampleAt = Date.now();
    elements.lastSample.textContent = "Sample received just now";
    state.history.push(angle);
    if (state.history.length > 180) state.history.shift();
    drawChart();
  }

  function drawChart() {
    const canvas = elements.chart;
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const width = Math.max(1, rect.width);
    const height = Math.max(1, rect.height);
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    ctx.strokeStyle = "rgba(124, 245, 202, 0.08)";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i += 1) {
      const y = i * height / 4 + 0.5;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
    }
    if (state.history.length < 2) return;
    const gradient = ctx.createLinearGradient(0, 0, width, 0);
    gradient.addColorStop(0, "#7cf5ca"); gradient.addColorStop(1, "#e2ff78");
    ctx.strokeStyle = gradient; ctx.lineWidth = 2; ctx.lineJoin = "round"; ctx.lineCap = "round";
    ctx.shadowColor = "rgba(124, 245, 202, 0.5)"; ctx.shadowBlur = 8;
    ctx.beginPath();
    state.history.forEach((value, index) => {
      const x = index / 179 * width;
      const y = height - value / 360 * height;
      if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  function processChunk(chunk) {
    state.lineBuffer += chunk;
    const lines = state.lineBuffer.split(/\r?\n/);
    state.lineBuffer = lines.pop() || "";
    lines.map((line) => line.trim()).filter(Boolean).forEach(parseSample);
  }

  async function readSerialLoop() {
    const decoder = new TextDecoder();
    try {
      while (state.port?.readable && state.connected) {
        state.reader = state.port.readable.getReader();
        try {
          while (true) {
            const { value, done } = await state.reader.read();
            if (done) break;
            if (value) processChunk(decoder.decode(value, { stream: true }));
          }
        } finally {
          state.reader.releaseLock();
          state.reader = null;
        }
      }
    } catch (error) {
      addTerminalLine(`Serial read error: ${error.message}`, "error");
    } finally {
      if (state.connected) await disconnectSerial();
    }
  }

  async function connectPort(port) {
    setConnection("connecting", "Opening serial port");
    try {
      await port.open({ baudRate: 115200 });
      state.port = port;
      setConnection("online", "Teensy connected");
      elements.zeroFeedback.textContent = "Ready. Move to a reference position, then press zero.";
      addTerminalLine("USB serial connected at 115200 baud.", "system");
      readSerialLoop();
    } catch (error) {
      setConnection("offline", "Teensy offline");
      addTerminalLine(`Connection failed: ${error.message}`, "error");
      showToast("Could not open the serial port");
    }
  }

  async function disconnectSerial() {
    state.connected = false;
    try { await state.reader?.cancel(); } catch (_) { /* port may already be gone */ }
    try { await state.port?.close(); } catch (_) { /* port may already be gone */ }
    state.reader = null; state.port = null;
    setConnection("offline", "Teensy offline");
    addTerminalLine("USB serial disconnected.", "system");
  }

  async function toggleConnection() {
    if (state.demoTimer) { stopDemo(); return; }
    if (state.connected) { await disconnectSerial(); return; }
    if (!("serial" in navigator)) {
      addTerminalLine("Web Serial is unavailable. Use Chrome or Edge on localhost.", "error");
      showToast("Open this page in Chrome or Edge");
      return;
    }
    try {
      const port = await navigator.serial.requestPort();
      await connectPort(port);
    } catch (error) {
      if (error.name !== "NotFoundError") addTerminalLine(`Port selection failed: ${error.message}`, "error");
      setConnection("offline", "Teensy offline");
    }
  }

  async function sendZero() {
    if (state.demoTimer) {
      state.demoZero = performance.now() / 28;
      elements.zeroFeedback.textContent = "Demo zero reference updated.";
      showToast("Demo position is now 0°");
      addTerminalLine("TX zero", "command");
      return;
    }
    if (!state.port?.writable) return;
    const writer = state.port.writable.getWriter();
    try {
      await writer.write(new TextEncoder().encode("zero\n"));
      elements.zeroFeedback.textContent = "Zero command sent—waiting for Teensy confirmation…";
      addTerminalLine("TX zero", "command");
    } catch (error) {
      elements.zeroFeedback.textContent = `Zero command failed: ${error.message}`;
      addTerminalLine(`Write failed: ${error.message}`, "error");
    } finally {
      writer.releaseLock();
    }
  }

  function startDemo() {
    let tick = 0;
    setConnection("online", "Demo stream");
    elements.connectButton.textContent = "Stop demo";
    elements.zeroFeedback.textContent = "Demo mode: the Zero button is fully interactive.";
    addTerminalLine("Demo stream started—no hardware is being accessed.", "system");
    state.demoTimer = setInterval(() => {
      tick += 1;
      const absolute = Math.round(((performance.now() / 28) + 900 * Math.sin(tick / 25)) % 16384);
      const zeroed = (absolute - Math.round(state.demoZero) + 16384) % 16384;
      const angle = zeroed * 360 / 16384;
      parseSample(`${Math.round(performance.now() * 1000)},${absolute},${zeroed},${angle.toFixed(4)},118,OK,0,0`);
    }, 100);
  }

  function stopDemo() {
    clearInterval(state.demoTimer); state.demoTimer = null;
    setConnection("offline", "Teensy offline");
    addTerminalLine("Demo stream stopped.", "system");
  }

  async function tryAutoReconnect() {
    if (!("serial" in navigator) || new URLSearchParams(location.search).has("demo")) return;
    try {
      const ports = await navigator.serial.getPorts();
      if (ports.length === 1) {
        addTerminalLine("Previously authorized Teensy found; reconnecting…", "system");
        await connectPort(ports[0]);
      }
    } catch (error) {
      addTerminalLine(`Automatic reconnect skipped: ${error.message}`, "error");
    }
  }

  elements.connectButton.addEventListener("click", toggleConnection);
  elements.zeroButton.addEventListener("click", sendZero);
  elements.pauseButton.addEventListener("click", () => {
    state.paused = !state.paused;
    elements.pauseButton.textContent = state.paused ? "Resume" : "Pause";
    if (!state.paused) addTerminalLine("Terminal output resumed.", "system");
  });
  elements.clearButton.addEventListener("click", () => { elements.terminal.replaceChildren(); addTerminalLine("Terminal cleared.", "system"); });
  window.addEventListener("resize", drawChart);
  navigator.serial?.addEventListener("disconnect", () => { if (state.connected) disconnectSerial(); });

  setInterval(() => {
    state.displayedRate = state.samplesThisSecond;
    state.samplesThisSecond = 0;
    elements.sampleRate.textContent = state.connected ? `${state.displayedRate} Hz` : "— Hz";
    if (state.lastSampleAt) {
      const age = Math.floor((Date.now() - state.lastSampleAt) / 1000);
      elements.lastSample.textContent = age < 2 ? "Sample received just now" : `Last sample ${age}s ago`;
    }
  }, 1000);

  drawChart();
  if (new URLSearchParams(location.search).has("demo")) startDemo();
  else tryAutoReconnect();
})();
