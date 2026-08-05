(() => {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const elements = {
    connectionPill: $("connectionPill"), connectionText: $("connectionText"), connectButton: $("connectButton"),
    modeValue: $("modeValue"), modeExplanation: $("modeExplanation"), globalAlert: $("globalAlert"),
    zeroButton: $("zeroButton"), testModeButton: $("testModeButton"), applyGainsButton: $("applyGainsButton"),
    tuneHoldButton: $("tuneHoldButton"), tuneHint: $("tuneHint"), armButton: $("armButton"), disarmButton: $("disarmButton"),
    gainArm: $("gainArm"), gainPendulum: $("gainPendulum"), gainArmRate: $("gainArmRate"), gainPendulumRate: $("gainPendulumRate"), gainFeedback: $("gainFeedback"),
    pendulumDegrees: $("pendulumDegrees"), armDegrees: $("armDegrees"), pendulumRate: $("pendulumRate"), armRate: $("armRate"),
    rawCount: $("rawCount"), agcValue: $("agcValue"), parityErrors: $("parityErrors"), protocolErrors: $("protocolErrors"), loopTiming: $("loopTiming"),
    torqueValue: $("torqueValue"), odriveErrors: $("odriveErrors"), gainSummary: $("gainSummary"),
    readinessRing: $("readinessRing"), readinessTitle: $("readinessTitle"), readinessText: $("readinessText"),
    spaceStatus: $("spaceStatus"), stateChart: $("stateChart"), eventLog: $("eventLog"), downloadButton: $("downloadButton"), clearLogButton: $("clearLogButton"), toast: $("toast")
  };

  const modeDescriptions = {
    DISARMED: "Motor output is disabled.", TEST: "Sensor test only; motor output is disabled.",
    SWING_UP: "Automatic swing-up is active.", BALANCE: "Upright balance control is active.",
    TUNING: "Low-torque Spacebar dead-man test is active.", FAULT: "A safety check stopped the controller.", OFFLINE: "Connect the Teensy to begin."
  };
  const modelGains = [-0.0914, 1.44432, -0.06921, 0.13886];
  const state = {
    port: null, reader: null, writer: null, connected: false, connecting: false, lineBuffer: "",
    sample: null, history: [], telemetryRows: [], events: [], toastTimer: null, deadmanTimer: null,
    spaceHeld: false, demoTimer: null, lastMessageAt: 0
  };

  function setConnection(kind, text) {
    state.connected = kind === "online";
    state.connecting = kind === "connecting";
    elements.connectionPill.className = `pill ${kind}`;
    elements.connectionText.textContent = text;
    elements.connectButton.textContent = state.connected ? "Disconnect" : state.connecting ? "Connecting…" : "Connect Teensy";
    elements.connectButton.disabled = state.connecting;
    render();
  }

  function toast(message) {
    elements.toast.textContent = message;
    elements.toast.classList.add("show");
    clearTimeout(state.toastTimer);
    state.toastTimer = setTimeout(() => elements.toast.classList.remove("show"), 2300);
  }

  function logEvent(message, level = "info") {
    const timestamp = new Date().toLocaleTimeString([], { hour12: false });
    state.events.push({ timestamp, message, level });
    if (state.events.length > 240) state.events.shift();
    const row = document.createElement("p");
    row.className = level.toLowerCase();
    const time = document.createElement("time");
    time.textContent = timestamp;
    const text = document.createElement("span");
    text.textContent = message;
    row.append(time, text);
    elements.eventLog.appendChild(row);
    while (elements.eventLog.childElementCount > 240) elements.eventLog.firstElementChild.remove();
    elements.eventLog.scrollTop = elements.eventLog.scrollHeight;
  }

  function parseNumber(value) {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : 0;
  }

  function parseTelemetry(parts, original) {
    if (parts.length < 26) return;
    state.sample = {
      ms: parseNumber(parts[1]), mode: parts[2], arm: parseNumber(parts[3]), pendulum: parseNumber(parts[4]),
      armRate: parseNumber(parts[5]), pendulumRate: parseNumber(parts[6]), torque: parseNumber(parts[7]),
      deadman: parts[8] === "1", odrive: parts[9] === "1", encoder: parts[10], zero: parts[11] === "1",
      count: parseNumber(parts[12]), agc: parseNumber(parts[13]), parity: parseNumber(parts[14]), protocol: parseNumber(parts[15]),
      gains: parts.slice(16, 20).map(parseNumber), odriveErrors: parseNumber(parts[20]),
      loopUs: parseNumber(parts[21]), maximumLoopUs: parseNumber(parts[22]), setup: parts[23] === "1",
      automatic: parts[24] === "1", fault: parts.slice(25).join(",")
    };
    state.lastMessageAt = Date.now();
    state.telemetryRows.push(original);
    if (state.telemetryRows.length > 15000) state.telemetryRows.shift();
    state.history.push({ pendulum: state.sample.pendulum * 180 / Math.PI, arm: state.sample.arm * 180 / Math.PI });
    if (state.history.length > 300) state.history.shift();
    render();
    drawChart();
  }

  function processLine(line) {
    const clean = line.trim();
    if (!clean) return;
    if (clean.startsWith("@T,")) {
      parseTelemetry(clean.split(","), clean);
    } else if (clean.startsWith("@E,")) {
      const parts = clean.split(",");
      const level = (parts[1] || "INFO").toLowerCase();
      const code = parts[2] || "EVENT";
      const message = parts.slice(3).join(",") || code;
      logEvent(`${code}: ${message}`, level === "error" ? "error" : level === "warn" ? "warn" : "info");
      if (level === "error" || code === "REFUSED") toast(message);
      if (code === "GAINS") elements.gainFeedback.textContent = message;
      if (code === "ZEROED") toast("Reference position saved");
    } else if (clean.startsWith("@HELLO,")) {
      logEvent(`Firmware connected (${clean.split(",").slice(1).join(" · ")})`);
    } else {
      logEvent(`Unrecognized serial line: ${clean}`, "warn");
    }
  }

  function processChunk(chunk) {
    state.lineBuffer += chunk;
    const lines = state.lineBuffer.split(/\r?\n/);
    state.lineBuffer = lines.pop() || "";
    lines.forEach(processLine);
  }

  async function readLoop() {
    const decoder = new TextDecoder();
    try {
      while (state.connected && state.port?.readable) {
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
      logEvent(`Serial read failed: ${error.message}`, "error");
    } finally {
      if (state.connected) await disconnect();
    }
  }

  async function connect(port) {
    setConnection("connecting", "Opening USB serial");
    try {
      await port.open({ baudRate: 115200 });
      state.port = port;
      state.writer = port.writable.getWriter();
      setConnection("online", "Teensy connected");
      logEvent("USB serial connection opened.");
      readLoop();
    } catch (error) {
      logEvent(`Connection failed: ${error.message}`, "error");
      setConnection("offline", "Not connected");
    }
  }

  async function disconnect() {
    if (state.writer) await send("deadman_release");
    releaseSpaceDeadman(true);
    state.connected = false;
    try { await state.reader?.cancel(); } catch (_) { /* already closed */ }
    if (state.writer) { try { state.writer.releaseLock(); } catch (_) { /* no-op */ } }
    state.writer = null;
    try { await state.port?.close(); } catch (_) { /* device may have vanished */ }
    state.port = null;
    state.sample = null;
    setConnection("offline", "Not connected");
    logEvent("USB serial disconnected.", "warn");
  }

  async function send(command) {
    if (!state.connected || !state.writer) return false;
    try {
      await state.writer.write(new TextEncoder().encode(`${command}\n`));
      return true;
    } catch (error) {
      logEvent(`Could not send “${command}”: ${error.message}`, "error");
      return false;
    }
  }

  function checks() {
    const sample = state.sample;
    return {
      connection: state.connected && sample !== null && Date.now() - state.lastMessageAt < 1000,
      encoder: Boolean(sample && sample.encoder === "OK"), odrive: Boolean(sample && sample.odrive && sample.odriveErrors === 0),
      deadman: Boolean(sample && sample.deadman && spaceControlIsActive()),
      mechanism: Boolean(sample && sample.setup), zero: Boolean(sample && sample.zero)
    };
  }

  function updateCheck(id, good, waitingText = "WAITING") {
    const card = $(id);
    card.classList.toggle("good", good);
    card.classList.toggle("bad", state.connected && !good);
    card.querySelector("b").textContent = good ? "READY" : waitingText;
  }

  function render() {
    const sample = state.sample;
    const safeMode = sample && ["DISARMED", "TEST"].includes(sample.mode);
    const activeMode = sample && ["SWING_UP", "BALANCE", "TUNING"].includes(sample.mode);
    const ready = checks();
    const readyCount = Object.values(ready).filter(Boolean).length;
    updateCheck("checkConnection", ready.connection, "CONNECT");
    updateCheck("checkEncoder", ready.encoder, sample?.encoder || "WAITING");
    updateCheck("checkOdrive", ready.odrive, sample?.odriveErrors ? "DRIVE ERROR" : "WAITING");
    updateCheck("checkDeadman", ready.deadman, "HOLD SPACE");
    updateCheck("checkMechanism", ready.mechanism, "VERIFY SIGN");
    updateCheck("checkZero", ready.zero, "NOT SAVED");

    const mode = sample?.mode || "OFFLINE";
    elements.modeValue.textContent = mode.replaceAll("_", " ");
    elements.modeExplanation.textContent = modeDescriptions[mode] || "Unknown controller state.";
    elements.zeroButton.disabled = !(state.connected && sample && safeMode && ready.encoder && ready.odrive);
    elements.testModeButton.disabled = !(state.connected && sample && safeMode);
    elements.testModeButton.textContent = mode === "TEST" ? "Leave sensor test" : "Enter sensor test";
    elements.applyGainsButton.disabled = !(state.connected && sample && safeMode);

    const tuningPositionReady = Boolean(sample && Math.abs(sample.pendulum) < 0.14 && Math.abs(sample.pendulumRate) < 1);
    const allReady = readyCount === 6;
    elements.tuneHoldButton.disabled = !(allReady && safeMode && tuningPositionReady);
    elements.tuneHint.textContent = !allReady ? "Complete every setup check, including holding Space." : !safeMode ? "Disarm before tuning." : !tuningPositionReady ? "Hold the pendulum nearly upright and motionless." : "Ready—keep Space held and click to begin.";
    elements.armButton.disabled = !(allReady && mode === "DISARMED" && sample.automatic);
    elements.disarmButton.disabled = !(state.connected && sample && (activeMode || mode === "FAULT" || mode === "TEST"));

    elements.readinessRing.textContent = `${readyCount}/6`;
    elements.readinessTitle.textContent = allReady && sample?.automatic ? "Ready to arm" : allReady ? "Swing-up locked" : "Not ready";
    elements.readinessText.textContent = allReady && sample?.automatic ? "Hardware checks pass and automatic swing-up is deliberately enabled." : allReady ? "Finish and review stable upright trials before enabling automatic swing-up in firmware." : "Complete the setup checklist and hold Space.";
    elements.spaceStatus.classList.toggle("holding", spaceControlIsActive());
    elements.spaceStatus.querySelector("strong").textContent = spaceControlIsActive() ? "SPACE HELD" : "HOLD SPACE";
    elements.spaceStatus.querySelector("small").textContent = spaceControlIsActive() ? "Release to stop motor output" : "Focused-tab dead-man control";
    elements.globalAlert.classList.toggle("hidden", mode !== "FAULT");
    if (mode === "FAULT") elements.globalAlert.textContent = `Stopped by safety fault: ${sample.fault === "-" ? "see event log" : sample.fault}. Correct the cause, then press DISARM / STOP to acknowledge it; zero must be saved again.`;

    if (!sample) return;
    const degrees = 180 / Math.PI;
    elements.pendulumDegrees.textContent = (sample.pendulum * degrees).toFixed(1);
    elements.armDegrees.textContent = (sample.arm * degrees).toFixed(1);
    elements.pendulumRate.textContent = sample.pendulumRate.toFixed(2);
    elements.armRate.textContent = sample.armRate.toFixed(2);
    elements.rawCount.textContent = Math.round(sample.count).toLocaleString();
    elements.agcValue.textContent = Math.round(sample.agc);
    elements.parityErrors.textContent = sample.parity;
    elements.protocolErrors.textContent = sample.protocol;
    elements.loopTiming.textContent = `${(sample.loopUs / 1000).toFixed(2)} / ${(sample.maximumLoopUs / 1000).toFixed(2)} ms`;
    elements.torqueValue.textContent = sample.torque.toFixed(3);
    elements.odriveErrors.textContent = sample.odriveErrors === 0 ? "None" : `0x${sample.odriveErrors.toString(16).toUpperCase()}`;
    elements.gainSummary.textContent = sample.gains.map((value) => value.toFixed(2)).join(" · ");
  }

  function drawChart() {
    const canvas = elements.stateChart;
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const width = Math.max(1, rect.width), height = Math.max(1, rect.height);
    canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr);
    const context = canvas.getContext("2d");
    context.setTransform(dpr, 0, 0, dpr, 0, 0); context.clearRect(0, 0, width, height);
    context.strokeStyle = "rgba(145,244,198,.09)"; context.lineWidth = 1;
    for (let i = 0; i <= 4; i += 1) { const y = i * height / 4; context.beginPath(); context.moveTo(0, y); context.lineTo(width, y); context.stroke(); }
    const plot = (key, color, range) => {
      if (state.history.length < 2) return;
      context.beginPath(); context.strokeStyle = color; context.lineWidth = 2; context.lineJoin = "round";
      state.history.forEach((point, index) => {
        const x = index / 299 * width; const y = height / 2 - Math.max(-range, Math.min(range, point[key])) / (2 * range) * height;
        if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
      });
      context.stroke();
    };
    plot("pendulum", "#91f4c6", 190); plot("arm", "#72b9ff", 190);
  }

  function applyPreset(scale) {
    [elements.gainArm, elements.gainPendulum, elements.gainArmRate, elements.gainPendulumRate].forEach((input, index) => {
      input.value = (modelGains[index] * scale).toFixed(5);
    });
    document.querySelectorAll(".preset").forEach((button) => button.classList.toggle("selected", Number(button.dataset.scale) === scale));
    elements.gainFeedback.textContent = "Review the four values, then apply while disarmed.";
  }

  async function applyGains() {
    const values = [elements.gainArm, elements.gainPendulum, elements.gainArmRate, elements.gainPendulumRate].map((input) => Number(input.value));
    if (!values.every(Number.isFinite)) { toast("Every gain must be a number"); return; }
    const limits = [0.16, 2.5, 0.13, 0.25];
    const accepted = values.every((value, index) => {
      const scale = value / modelGains[index];
      return Math.abs(value) <= limits[index] && scale >= 0.5 && scale <= 1.5;
    });
    if (!accepted) { toast("A gain is outside the firmware-safe range"); return; }
    if (await send(`gains ${values.join(" ")}`)) elements.gainFeedback.textContent = "Values sent; waiting for firmware validation…";
  }

  function spaceControlIsActive() {
    return state.spaceHeld && document.hasFocus() && !document.hidden;
  }

  function isTextEntry(target) {
    return target instanceof HTMLElement &&
      (target.isContentEditable || ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName));
  }

  function beginSpaceDeadman(event) {
    if (event.code !== "Space" || isTextEntry(event.target)) return;
    event.preventDefault();
    if (event.repeat || state.spaceHeld) return;
    if (!state.connected) { toast("Connect the Teensy before using the Space control"); return; }
    state.spaceHeld = true;
    send("deadman_hold");
    clearInterval(state.deadmanTimer);
    state.deadmanTimer = setInterval(() => {
      if (spaceControlIsActive()) send("deadman_hold");
      else releaseSpaceDeadman(false);
    }, 150);
    render();
  }

  function releaseSpaceDeadman(skipCommand = false) {
    const wasHeld = state.spaceHeld || state.deadmanTimer !== null;
    state.spaceHeld = false;
    clearInterval(state.deadmanTimer);
    state.deadmanTimer = null;
    if (wasHeld && !skipCommand && state.connected) send("deadman_release");
    render();
  }

  async function startTuningTrial() {
    if (elements.tuneHoldButton.disabled || !spaceControlIsActive()) return;
    if (!(await send("deadman_hold"))) return;
    await send("tune_start CONFIRM");
  }

  function downloadCsv() {
    const header = "record,time_ms,mode,arm_rad,pendulum_rad,arm_rate_rad_s,pendulum_rate_rad_s,torque_nm,browser_deadman_held,odrive_online,encoder_status,zero_valid,encoder_count,agc,parity_errors,protocol_errors,k_arm,k_pendulum,k_arm_rate,k_pendulum_rate,odrive_errors,loop_us,max_loop_us,mechanism_setup_complete,automatic_swing_up_enabled,fault";
    const blob = new Blob([[header, ...state.telemetryRows].join("\n")], { type: "text/csv" });
    const link = document.createElement("a"); link.href = URL.createObjectURL(blob);
    link.download = `furuta-session-${new Date().toISOString().replaceAll(":", "-")}.csv`; link.click();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  }

  function startDemo() {
    setConnection("online", "Demonstration data");
    let tick = 0;
    state.demoTimer = setInterval(() => {
      tick += 1; const pendulum = -Math.PI + 0.05 * Math.sin(tick / 12), arm = 0.15 * Math.sin(tick / 30);
      processLine(`@T,${tick * 40},TEST,${arm},${pendulum},0.02,0.08,0,1,1,OK,1,8192,118,0,0,-0.05941,0.93881,-0.04498,0.09026,0,2100,2900,0,0,-`);
    }, 40);
    elements.connectButton.textContent = "Demo active"; elements.connectButton.disabled = true;
    logEvent("Demonstration mode started; no commands reach hardware.");
  }

  document.querySelectorAll(".tab").forEach((button) => button.addEventListener("click", () => {
    document.querySelectorAll(".tab").forEach((tab) => tab.classList.toggle("active", tab === button));
    document.querySelectorAll(".page").forEach((page) => page.classList.toggle("active", page.id === button.dataset.page));
    history.replaceState(null, "", `#${button.dataset.page}`); setTimeout(drawChart, 0);
  }));
  document.querySelectorAll(".preset").forEach((button) => button.addEventListener("click", () => applyPreset(Number(button.dataset.scale))));
  elements.connectButton.addEventListener("click", async () => {
    if (state.connected) { await disconnect(); return; }
    if (!("serial" in navigator)) { toast("Use current Chrome or Edge on localhost"); return; }
    try { await connect(await navigator.serial.requestPort()); } catch (error) { if (error.name !== "NotFoundError") logEvent(error.message, "error"); }
  });
  elements.zeroButton.addEventListener("click", () => send("zero"));
  elements.testModeButton.addEventListener("click", () => send(state.sample?.mode === "TEST" ? "test_stop" : "test_start"));
  elements.applyGainsButton.addEventListener("click", applyGains);
  elements.tuneHoldButton.addEventListener("click", startTuningTrial);
  elements.armButton.addEventListener("click", async () => {
    if (!spaceControlIsActive()) { toast("Hold Space in this tab before arming"); return; }
    if (await send("deadman_hold") && spaceControlIsActive()) send("arm CONFIRM");
  });
  elements.disarmButton.addEventListener("click", () => send("disarm"));
  elements.downloadButton.addEventListener("click", downloadCsv);
  elements.clearLogButton.addEventListener("click", () => { elements.eventLog.replaceChildren(); state.events = []; state.telemetryRows = []; });
  document.addEventListener("keydown", beginSpaceDeadman);
  document.addEventListener("keyup", (event) => {
    if (event.code !== "Space") return;
    if (state.spaceHeld) event.preventDefault();
    releaseSpaceDeadman(false);
  });
  window.addEventListener("resize", drawChart);
  window.addEventListener("blur", () => releaseSpaceDeadman(false));
  document.addEventListener("visibilitychange", () => { if (document.hidden) releaseSpaceDeadman(false); });
  setInterval(() => { if (state.connected && state.lastMessageAt && Date.now() - state.lastMessageAt > 1000) render(); }, 500);
  if (new URLSearchParams(location.search).get("demo") === "1") startDemo();
  else setConnection("offline", "Not connected");
  const initial = location.hash.slice(1);
  if (["setup", "test", "tune", "run"].includes(initial)) document.querySelector(`.tab[data-page="${initial}"]`)?.click();
})();
