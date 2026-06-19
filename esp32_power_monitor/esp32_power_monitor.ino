/*
 * ============================================================
 *  ESP32 Power Monitor
 *  Sensors : ZMPT101B (Voltage) + ACS712 (Current)
 *  Load    : LED Bulb
 *  Feature : Hosted Web Dashboard (Wi-Fi AP or STA mode)
 * ============================================================
 *
 *  WIRING GUIDE
 *  ─────────────────────────────────────────────────────────
 *  ZMPT101B  →  ESP32
 *    VCC     →  3.3 V
 *    GND     →  GND
 *    OUT     →  GPIO 34  (ADC1_CH6 – input only pin)
 *
 *  ACS712    →  ESP32
 *    VCC     →  5 V  (or 3.3 V for ACS712-3V3 variant)
 *    GND     →  GND
 *    OUT     →  GPIO 35  (ADC1_CH7 – input only pin)
 *
 *  NOTE: Both ADC pins must be on ADC1 (GPIO 32-39).
 *        ADC2 pins conflict with Wi-Fi on ESP32.
 *
 *  LIBRARY REQUIREMENTS (install via Arduino Library Manager)
 *  ─────────────────────────────────────────────────────────
 *    • WiFi         (built-in ESP32 core)
 *    • WebServer    (built-in ESP32 core)
 *    • ArduinoJson  v6.x  by Benoit Blanchon
 *
 *  CALIBRATION
 *  ─────────────────────────────────────────────────────────
 *  1. VOLTAGE_CALIBRATION : Adjust until web reading matches
 *     a true RMS multimeter on the mains side.
 *     Typical starting value for ZMPT101B at 3.3 V ref: ~0.95
 *
 *  2. ACS712_SENSITIVITY  : 185 mV/A for 5 A model
 *                           100 mV/A for 20 A model
 *                            66 mV/A for 30 A model
 *
 *  3. ACS712_VREF         : Midpoint voltage of the ACS712
 *     output. Run the sketch with NO load and print rawCurrent
 *     to find the actual value (should be ~1.65 V at 3.3 V VCC).
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <math.h>

// ─── Wi-Fi ──────────────────────────────────────────────────
// Option A – Access Point (no router needed)
#define WIFI_MODE_AP       true        // set false for Station mode

const char* AP_SSID     = "PowerMonitor";
const char* AP_PASSWORD = "monitor123";   // min 8 chars

// Option B – connect to existing router (used if WIFI_MODE_AP = false)
const char* STA_SSID    = "OnePluse";
const char* STA_PASSWORD= "1234567890";

// ─── GPIO pins ──────────────────────────────────────────────
#define VOLTAGE_PIN   34    // ZMPT101B output
#define CURRENT_PIN   35    // ACS712 output

// ─── ADC reference / resolution ─────────────────────────────
#define ADC_RESOLUTION    4095.0f    // 12-bit ESP32
#define VREF              3.3f       // ESP32 ADC reference voltage

// ─── Voltage sensor calibration ─────────────────────────────
#define VOLTAGE_CALIBRATION  0.95f   // tuning multiplier
#define V_SAMPLES            500     // samples per RMS cycle

// ─── ACS712 calibration ─────────────────────────────────────
// Change sensitivity to match your ACS712 variant:
#define ACS712_SENSITIVITY   0.185f  // V/A  (5 A model)
// #define ACS712_SENSITIVITY 0.100f  // V/A  (20 A model)
// #define ACS712_SENSITIVITY 0.066f  // V/A  (30 A model)
#define ACS712_VREF          1.65f   // mid-point output voltage (no load)
#define I_SAMPLES            500

// ─── Power factor (assumed; LED bulb ~0.5–0.6) ──────────────
#define POWER_FACTOR         0.55f

// ─── Thresholds for status indicators ───────────────────────
#define HIGH_VOLTAGE_THRESHOLD  250.0f   // Vrms
#define HIGH_CURRENT_THRESHOLD    2.0f   // Arms
#define HIGH_POWER_THRESHOLD    200.0f   // W

// ─── Update interval ────────────────────────────────────────
#define SAMPLE_INTERVAL_MS   500   // readings refresh every 500 ms

// ─── Globals ────────────────────────────────────────────────
WebServer server(80);

float gVoltage  = 0.0f;
float gCurrent  = 0.0f;
float gPower    = 0.0f;
float gEnergy   = 0.0f;   // kWh accumulator
unsigned long gStartTime = 0;
unsigned long gLastSample = 0;

// ════════════════════════════════════════════════════════════
//  Sensor Reading Functions
// ════════════════════════════════════════════════════════════

/**
 * Measure RMS voltage using ZMPT101B.
 * Reads multiple samples, computes sum-of-squares, returns RMS.
 */
float measureVoltageRMS() {
    double sumSquares = 0.0;
    for (int i = 0; i < V_SAMPLES; i++) {
        int raw = analogRead(VOLTAGE_PIN);
        // Convert to voltage centered around mid-rail
        float v = ((float)raw / ADC_RESOLUTION * VREF) - (VREF / 2.0f);
        sumSquares += (double)v * v;
        delayMicroseconds(100);
    }
    float vrms = sqrt(sumSquares / V_SAMPLES);
    // Scale to actual mains voltage using calibration factor
    return vrms * VOLTAGE_CALIBRATION * 310.0f;  // 310 ≈ empirical scale for ZMPT101B
}

/**
 * Measure RMS current using ACS712.
 */
float measureCurrentRMS() {
    double sumSquares = 0.0;
    for (int i = 0; i < I_SAMPLES; i++) {
        int raw = analogRead(CURRENT_PIN);
        float adcVoltage = (float)raw / ADC_RESOLUTION * VREF;
        // Centered current value
        float iSample = (adcVoltage - ACS712_VREF) / ACS712_SENSITIVITY;
        sumSquares += (double)iSample * iSample;
        delayMicroseconds(100);
    }
    float irms = sqrt(sumSquares / I_SAMPLES);
    // Zero-out noise floor
    if (irms < 0.05f) irms = 0.0f;
    return irms;
}

// ════════════════════════════════════════════════════════════
//  HTML Dashboard (stored in PROGMEM to save SRAM)
// ════════════════════════════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>ESP32 Power Monitor</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=Share+Tech+Mono&display=swap');

  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  :root {
    --bg:        #050d14;
    --panel:     #0a1929;
    --border:    #0d2137;
    --accent-v:  #00e5ff;
    --accent-i:  #00ff9d;
    --accent-p:  #ff6b35;
    --accent-e:  #c77dff;
    --dim:       #3a5068;
    --text:      #cde8ff;
    --text-soft: #5e8ca8;
    --glow-v: 0 0 20px #00e5ff66, 0 0 60px #00e5ff22;
    --glow-i: 0 0 20px #00ff9d66, 0 0 60px #00ff9d22;
    --glow-p: 0 0 20px #ff6b3566, 0 0 60px #ff6b3522;
    --glow-e: 0 0 20px #c77dff66, 0 0 60px #c77dff22;
  }

  html, body { height: 100%; }

  body {
    background: var(--bg);
    font-family: 'Share Tech Mono', monospace;
    color: var(--text);
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* ── Animated grid background ── */
  body::before {
    content: '';
    position: fixed; inset: 0;
    background-image:
      linear-gradient(rgba(0,229,255,.03) 1px, transparent 1px),
      linear-gradient(90deg, rgba(0,229,255,.03) 1px, transparent 1px);
    background-size: 40px 40px;
    pointer-events: none;
    z-index: 0;
  }

  /* ── Scan-line overlay ── */
  body::after {
    content: '';
    position: fixed; inset: 0;
    background: repeating-linear-gradient(
      0deg,
      transparent,
      transparent 2px,
      rgba(0,0,0,.12) 2px,
      rgba(0,0,0,.12) 4px
    );
    pointer-events: none;
    z-index: 0;
  }

  .wrapper {
    position: relative; z-index: 1;
    max-width: 1100px;
    margin: 0 auto;
    padding: 2rem 1.5rem 4rem;
  }

  /* ── Header ── */
  header {
    text-align: center;
    margin-bottom: 2.5rem;
  }
  .header-badge {
    display: inline-block;
    background: linear-gradient(135deg, #0a2540, #061528);
    border: 1px solid var(--accent-v);
    border-radius: 4px;
    padding: .25rem 1rem;
    font-size: .65rem;
    letter-spacing: .25em;
    color: var(--accent-v);
    text-transform: uppercase;
    margin-bottom: .75rem;
    box-shadow: var(--glow-v);
  }
  h1 {
    font-family: 'Orbitron', sans-serif;
    font-weight: 900;
    font-size: clamp(1.6rem, 5vw, 3rem);
    letter-spacing: .05em;
    color: #fff;
    text-shadow: 0 0 30px rgba(0,229,255,.5);
  }
  h1 span { color: var(--accent-v); }
  .subtitle {
    font-size: .75rem;
    color: var(--text-soft);
    letter-spacing: .2em;
    margin-top: .4rem;
    text-transform: uppercase;
  }

  /* ── Status bar ── */
  .status-bar {
    display: flex;
    align-items: center;
    gap: .75rem;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: .6rem 1.2rem;
    margin-bottom: 2rem;
    font-size: .72rem;
    color: var(--text-soft);
  }
  .dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: #00ff9d;
    box-shadow: 0 0 8px #00ff9d;
    animation: pulse 2s infinite;
  }
  @keyframes pulse {
    0%,100% { opacity: 1; } 50% { opacity: .3; }
  }
  .status-bar .sep { margin-left: auto; }
  #uptime { color: var(--text); }

  /* ── Cards grid ── */
  .cards {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    gap: 1.25rem;
    margin-bottom: 2rem;
  }

  .card {
    background: var(--panel);
    border-radius: 10px;
    padding: 1.5rem 1.5rem 1.2rem;
    position: relative;
    overflow: hidden;
    border: 1px solid var(--border);
    transition: transform .2s, border-color .3s;
  }
  .card:hover { transform: translateY(-3px); }

  /* colour accents per card */
  .card-v  { border-top: 2px solid var(--accent-v); }
  .card-i  { border-top: 2px solid var(--accent-i); }
  .card-p  { border-top: 2px solid var(--accent-p); }
  .card-e  { border-top: 2px solid var(--accent-e); }

  .card-v:hover { border-color: var(--accent-v); box-shadow: var(--glow-v); }
  .card-i:hover { border-color: var(--accent-i); box-shadow: var(--glow-i); }
  .card-p:hover { border-color: var(--accent-p); box-shadow: var(--glow-p); }
  .card-e:hover { border-color: var(--accent-e); box-shadow: var(--glow-e); }

  /* card corner decoration */
  .card::before {
    content: '';
    position: absolute;
    top: -30px; right: -30px;
    width: 90px; height: 90px;
    border-radius: 50%;
    opacity: .07;
  }
  .card-v::before { background: var(--accent-v); }
  .card-i::before { background: var(--accent-i); }
  .card-p::before { background: var(--accent-p); }
  .card-e::before { background: var(--accent-e); }

  .card-label {
    font-size: .65rem;
    letter-spacing: .2em;
    text-transform: uppercase;
    margin-bottom: .9rem;
  }
  .card-v .card-label { color: var(--accent-v); }
  .card-i .card-label { color: var(--accent-i); }
  .card-p .card-label { color: var(--accent-p); }
  .card-e .card-label { color: var(--accent-e); }

  .card-icon {
    font-size: 1.4rem;
    float: right;
    margin-top: -1.8rem;
    opacity: .6;
  }

  .card-value {
    font-family: 'Orbitron', sans-serif;
    font-size: clamp(1.8rem, 4vw, 2.6rem);
    font-weight: 700;
    line-height: 1;
    margin-bottom: .35rem;
    transition: color .4s;
  }
  .card-v .card-value { color: var(--accent-v); text-shadow: var(--glow-v); }
  .card-i .card-value { color: var(--accent-i); text-shadow: var(--glow-i); }
  .card-p .card-value { color: var(--accent-p); text-shadow: var(--glow-p); }
  .card-e .card-value { color: var(--accent-e); text-shadow: var(--glow-e); }

  .card-unit {
    font-size: .75rem;
    color: var(--text-soft);
    letter-spacing: .1em;
  }

  /* alert flash */
  .card.alert .card-value { animation: alertFlash .5s step-end infinite; }
  @keyframes alertFlash {
    0%, 100% { opacity: 1; } 50% { opacity: .2; }
  }

  /* ── Progress bar ── */
  .bar-wrap {
    margin-top: 1rem;
    height: 4px;
    background: var(--border);
    border-radius: 2px;
    overflow: hidden;
  }
  .bar-fill {
    height: 100%;
    border-radius: 2px;
    transition: width .6s ease;
  }
  .card-v .bar-fill { background: var(--accent-v); box-shadow: 0 0 6px var(--accent-v); }
  .card-i .bar-fill { background: var(--accent-i); box-shadow: 0 0 6px var(--accent-i); }
  .card-p .bar-fill { background: var(--accent-p); box-shadow: 0 0 6px var(--accent-p); }
  .card-e .bar-fill { background: var(--accent-e); box-shadow: 0 0 6px var(--accent-e); }

  /* ── Chart section ── */
  .chart-section {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 1.5rem;
    margin-bottom: 2rem;
  }
  .chart-header {
    display: flex;
    align-items: center;
    gap: 1rem;
    margin-bottom: 1rem;
    font-size: .7rem;
    letter-spacing: .15em;
    text-transform: uppercase;
    color: var(--text-soft);
  }
  .chart-header span { color: var(--text); }
  .legend { display: flex; gap: 1.2rem; margin-left: auto; }
  .leg { display: flex; align-items: center; gap: .4rem; font-size: .65rem; }
  .leg-dot { width: 8px; height: 8px; border-radius: 50%; }

  canvas#chart { width: 100% !important; height: 200px; display: block; }

  /* ── Info table ── */
  .info-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 1rem;
  }
  @media (max-width: 500px) { .info-grid { grid-template-columns: 1fr; } }

  .info-box {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 1rem 1.2rem;
  }
  .info-box h3 {
    font-family: 'Orbitron', sans-serif;
    font-size: .65rem;
    letter-spacing: .2em;
    text-transform: uppercase;
    color: var(--text-soft);
    margin-bottom: .8rem;
    border-bottom: 1px solid var(--border);
    padding-bottom: .5rem;
  }
  .info-row {
    display: flex;
    justify-content: space-between;
    font-size: .72rem;
    padding: .25rem 0;
    border-bottom: 1px solid rgba(255,255,255,.03);
    color: var(--text-soft);
  }
  .info-row:last-child { border-bottom: none; }
  .info-row b { color: var(--text); font-weight: normal; }

  /* ── Footer ── */
  footer {
    text-align: center;
    margin-top: 2.5rem;
    font-size: .65rem;
    color: var(--dim);
    letter-spacing: .1em;
  }
</style>
</head>
<body>
<div class="wrapper">

  <header>
    <div class="header-badge">ESP32 · IoT Monitoring System</div>
    <h1>POWER <span>MONITOR</span></h1>
    <p class="subtitle">ZMPT101B Voltage &nbsp;·&nbsp; ACS712 Current &nbsp;·&nbsp; LED Load</p>
  </header>

  <div class="status-bar">
    <div class="dot"></div>
    <span>LIVE DATA</span>
    <span>|</span>
    <span id="refresh-rate">Refresh: 500 ms</span>
    <span class="sep"></span>
    <span>UPTIME: <span id="uptime">—</span></span>
  </div>

  <!-- ── Metric Cards ── -->
  <div class="cards">
    <div class="card card-v" id="card-v">
      <div class="card-label">⚡ Voltage RMS</div>
      <div class="card-icon">🔌</div>
      <div class="card-value" id="voltage">—</div>
      <div class="card-unit">VOLTS (V)</div>
      <div class="bar-wrap"><div class="bar-fill" id="bar-v" style="width:0%"></div></div>
    </div>
    <div class="card card-i" id="card-i">
      <div class="card-label">〰 Current RMS</div>
      <div class="card-icon">⚗️</div>
      <div class="card-value" id="current">—</div>
      <div class="card-unit">AMPERES (A)</div>
      <div class="bar-wrap"><div class="bar-fill" id="bar-i" style="width:0%"></div></div>
    </div>
    <div class="card card-p" id="card-p">
      <div class="card-label">🔆 Active Power</div>
      <div class="card-icon">🔥</div>
      <div class="card-value" id="power">—</div>
      <div class="card-unit">WATTS (W)</div>
      <div class="bar-wrap"><div class="bar-fill" id="bar-p" style="width:0%"></div></div>
    </div>
    <div class="card card-e" id="card-e">
      <div class="card-label">📦 Energy Used</div>
      <div class="card-icon">🔋</div>
      <div class="card-value" id="energy">—</div>
      <div class="card-unit">WATT·HOURS (Wh)</div>
      <div class="bar-wrap"><div class="bar-fill" id="bar-e" style="width:0%"></div></div>
    </div>
  </div>

  <!-- ── Live Chart ── -->
  <div class="chart-section">
    <div class="chart-header">
      <span>LIVE WAVEFORM &mdash; <span id="chart-label">Last 30 readings</span></span>
      <div class="legend">
        <div class="leg"><div class="leg-dot" style="background:#00e5ff"></div>Voltage</div>
        <div class="leg"><div class="leg-dot" style="background:#00ff9d"></div>Current ×100</div>
        <div class="leg"><div class="leg-dot" style="background:#ff6b35"></div>Power</div>
      </div>
    </div>
    <canvas id="chart"></canvas>
  </div>

  <!-- ── Info boxes ── -->
  <div class="info-grid">
    <div class="info-box">
      <h3>System Info</h3>
      <div class="info-row"><span>MCU</span><b>ESP32 (Xtensa LX6)</b></div>
      <div class="info-row"><span>Voltage Sensor</span><b>ZMPT101B</b></div>
      <div class="info-row"><span>Current Sensor</span><b>ACS712</b></div>
      <div class="info-row"><span>Load</span><b>LED Bulb</b></div>
      <div class="info-row"><span>ADC Resolution</span><b>12-bit (0–4095)</b></div>
      <div class="info-row"><span>Sample Count</span><b>500 per reading</b></div>
    </div>
    <div class="info-box">
      <h3>Derived Metrics</h3>
      <div class="info-row"><span>Apparent Power</span><b id="apparent">—</b></div>
      <div class="info-row"><span>Power Factor</span><b id="pf">0.55 (est.)</b></div>
      <div class="info-row"><span>Frequency</span><b>50 Hz</b></div>
      <div class="info-row"><span>Est. Monthly Energy</span><b id="monthly">—</b></div>
      <div class="info-row"><span>Data Points</span><b id="dp-count">0</b></div>
      <div class="info-row"><span>Last Update</span><b id="last-update">—</b></div>
    </div>
  </div>

  <footer>ESP32 Power Monitor &nbsp;·&nbsp; Built with Arduino &amp; C++</footer>
</div>

<script>
// ── Chart setup ──────────────────────────────────────────────
const MAX_POINTS = 30;
const canvas = document.getElementById('chart');
const ctx = canvas.getContext('2d');
canvas.width = canvas.offsetWidth || 900;
canvas.height = 200;
window.addEventListener('resize', () => { canvas.width = canvas.offsetWidth; drawChart(); });

const history = { v:[], i:[], p:[] };

function drawChart() {
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  // background
  ctx.fillStyle = '#050d14';
  ctx.fillRect(0, 0, W, H);

  // grid
  ctx.strokeStyle = 'rgba(0,229,255,.05)';
  ctx.lineWidth = 1;
  for (let x = 0; x < W; x += 40) { ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,H); ctx.stroke(); }
  for (let y = 0; y < H; y += 40) { ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); }

  if (history.v.length < 2) return;

  const datasets = [
    { data: history.v, color: '#00e5ff', max: 260, glow: '#00e5ff' },
    { data: history.i.map(v=>v*100), color: '#00ff9d', max: 260, glow: '#00ff9d' },
    { data: history.p, color: '#ff6b35', max: 260, glow: '#ff6b35' },
  ];

  datasets.forEach(ds => {
    if (ds.data.length < 2) return;
    const pts = ds.data;
    const n = pts.length;
    ctx.shadowBlur = 8;
    ctx.shadowColor = ds.glow;
    ctx.strokeStyle = ds.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    pts.forEach((v,i) => {
      const x = (i / (MAX_POINTS-1)) * W;
      const y = H - (Math.max(0, Math.min(v, ds.max)) / ds.max) * (H - 20) - 5;
      i === 0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
    });
    ctx.stroke();
    ctx.shadowBlur = 0;

    // area fill
    ctx.beginPath();
    pts.forEach((v,i) => {
      const x = (i / (MAX_POINTS-1)) * W;
      const y = H - (Math.max(0, Math.min(v, ds.max)) / ds.max) * (H - 20) - 5;
      i === 0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
    });
    ctx.lineTo(W, H); ctx.lineTo(0, H); ctx.closePath();
    ctx.fillStyle = ds.color + '12';
    ctx.fill();
  });
}

// ── Data fetch ───────────────────────────────────────────────
let dpCount = 0;
const startTime = Date.now();

function fmt(n, d=1) { return isNaN(n) ? '—' : n.toFixed(d); }

function pad(n) { return String(n).padStart(2,'0'); }
function fmtUptime(ms) {
  const s = Math.floor(ms/1000);
  const h = Math.floor(s/3600), m = Math.floor((s%3600)/60), sc = s%60;
  return `${pad(h)}:${pad(m)}:${pad(sc)}`;
}

async function fetchData() {
  try {
    const res = await fetch('/data');
    if (!res.ok) return;
    const d = await res.json();

    const v = d.voltage, i = d.current, p = d.power, e = d.energy;

    // update cards
    document.getElementById('voltage').textContent = fmt(v);
    document.getElementById('current').textContent = fmt(i, 3);
    document.getElementById('power').textContent   = fmt(p);
    document.getElementById('energy').textContent  = fmt(e*1000, 2);  // kWh→Wh

    // progress bars (max scale: 260V, 5A, 1300W, 500Wh)
    document.getElementById('bar-v').style.width = Math.min(v/260*100, 100)+'%';
    document.getElementById('bar-i').style.width = Math.min(i/5*100,   100)+'%';
    document.getElementById('bar-p').style.width = Math.min(p/1300*100,100)+'%';
    document.getElementById('bar-e').style.width = Math.min(e*1000/500*100,100)+'%';

    // alert flash
    document.getElementById('card-v').classList.toggle('alert', v > 250);
    document.getElementById('card-i').classList.toggle('alert', i > 2);
    document.getElementById('card-p').classList.toggle('alert', p > 200);

    // derived
    const apparent = v * i;
    document.getElementById('apparent').textContent = fmt(apparent) + ' VA';
    document.getElementById('monthly').textContent  = fmt(p * 24 * 30 / 1000, 3) + ' kWh';

    // history
    history.v.push(v); history.i.push(i); history.p.push(p);
    if (history.v.length > MAX_POINTS) { history.v.shift(); history.i.shift(); history.p.shift(); }

    // meta
    dpCount++;
    document.getElementById('dp-count').textContent = dpCount;
    document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
    document.getElementById('uptime').textContent = fmtUptime(Date.now() - startTime);

    drawChart();
  } catch(e) {
    console.warn('Fetch failed:', e);
  }
}

setInterval(fetchData, 500);
fetchData();
</script>
</body>
</html>
)rawliteral";

// ════════════════════════════════════════════════════════════
//  HTTP Route Handlers
// ════════════════════════════════════════════════════════════

void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
    StaticJsonDocument<200> doc;
    doc["voltage"] = gVoltage;
    doc["current"] = gCurrent;
    doc["power"]   = gPower;
    doc["energy"]  = gEnergy;
    doc["uptime"]  = (millis() - gStartTime) / 1000;

    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
}

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

// ════════════════════════════════════════════════════════════
//  Setup
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== ESP32 Power Monitor ===");

    // Configure ADC
    analogReadResolution(12);       // 12-bit (0–4095)
    analogSetAttenuation(ADC_11db); // 0–3.3 V input range
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(CURRENT_PIN, INPUT);

    // ── Wi-Fi ──────────────────────────────────────────
    if (WIFI_MODE_AP) {
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
        Serial.printf("Connect to Wi-Fi: %s  Password: %s\n", AP_SSID, AP_PASSWORD);
        Serial.printf("Then open: http://%s\n", WiFi.softAPIP().toString().c_str());
    } else {
        WiFi.begin(STA_SSID, STA_PASSWORD);
        Serial.print("Connecting to Wi-Fi");
        while (WiFi.status() != WL_CONNECTED) {
            delay(500); Serial.print(".");
        }
        Serial.println("\nConnected!");
        Serial.print("IP address: http://");
        Serial.println(WiFi.localIP());
    }

    // ── Web server routes ──────────────────────────────
    server.on("/",      handleRoot);
    server.on("/data",  handleData);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("Web server started.");

    gStartTime = millis();
}

// ════════════════════════════════════════════════════════════
//  Loop
// ════════════════════════════════════════════════════════════
void loop() {
    server.handleClient();

    unsigned long now = millis();
    if (now - gLastSample >= SAMPLE_INTERVAL_MS) {
        gLastSample = now;

        // Read sensors
        gVoltage = measureVoltageRMS();
        gCurrent = measureCurrentRMS();

        // Apparent power (S = V × I)
        float apparentPower = gVoltage * gCurrent;

        // Active power (P = S × PF)
        gPower = apparentPower * POWER_FACTOR;

        // Accumulate energy (kWh)
        // ΔkWh = P(W) × Δt(h) / 1000
        float dtHours = (float)SAMPLE_INTERVAL_MS / 3600000.0f;
        gEnergy += gPower * dtHours / 1000.0f;

        // Debug output
        Serial.printf("[%lu s] V=%.2f V  I=%.4f A  P=%.2f W  E=%.6f kWh\n",
            (now - gStartTime) / 1000,
            gVoltage, gCurrent, gPower, gEnergy);
    }
}
