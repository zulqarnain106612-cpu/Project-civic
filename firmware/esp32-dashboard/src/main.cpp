/*  Civic CAN Dashboard — ESP32 firmware
 *  Wi-Fi AP  : Civic-CAN-Monitor / CANmonitor26  ch6  max4
 *  HTTP      : port 80   → serves embedded HTML page (dashboard)
 *  WebSocket : port 81   → live JSON telemetry + TX commands
 *  HTTP POST : /update       → ESP32 OTA firmware upload (.bin)
 *  HTTP POST : /update-stm32 → STM32 firmware upload (.bin, flashed via UART bootloader)
 *  STM32 UART: HardwareSerial Stm32(2)  RX=GPIO16 (← PA9/USART1_TX)  TX=GPIO17 (→ PA10/USART1_RX)  115200 8N1
 *  STM32 boot: GPIO4 → BOOT0 pin, GPIO5 → NRST pin (2 extra wires needed)
 *  Transceiver: TJA1051 (NXP) — only supported CAN transceiver.
 *
 *  Hardware wiring for STM32 remote flash:
 *    ESP32 GPIO4  → STM32 BOOT0 pin  (active HIGH = bootloader mode)
 *    ESP32 GPIO5  → STM32 NRST pin   (active LOW  = reset, via 100Ω series resistor)
 *    ESP32 GPIO16 → STM32 PA3 (USART2 RX of STM32)   [existing]
 *    ESP32 GPIO17 → STM32 PA2 (USART2 TX of STM32)   [existing]
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <esp_task_wdt.h>

static constexpr char     AP_SSID[]  = "Civic-CAN-Monitor";
static constexpr char     AP_PASS[]  = "CANmonitor26";
static constexpr int      AP_CHANNEL = 6;
static constexpr int      AP_MAX_CONN= 4;
static constexpr int      STM32_RX   = 16;
static constexpr int      STM32_TX   = 17;
static constexpr uint32_t STM32_BAUD = 5250000;

/* STM32 bootloader control pins — wire these before using STM32 OTA */
static constexpr int STM32_BOOT0 = 4;   /* HIGH = ROM bootloader mode  */
static constexpr int STM32_NRST  = 5;   /* LOW  = reset (100Ω series)  */

WebServer        http(80);
WebSocketsServer ws(81);
HardwareSerial   Stm32(2);

static char     lineBuf[221];
static uint16_t lineLen    = 0;
static uint32_t framesSeen = 0;
static uint32_t bitrate    = 0;
static bool     locked     = false;
static uint32_t dwellMs    = 0;

/* ═══════════════════════════════════════════════════════════════════
   PAGE — exactly matches the sample dashboard screenshot
   ═══════════════════════════════════════════════════════════════════ */
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Vehicle Telemetry Dashboard</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{font:13px/1.5 system-ui,sans-serif;background:#0d1117;color:#c9d1d9;min-height:100vh;padding:10px 12px}
h1{font-size:1.1rem;font-weight:700;color:#c9d1d9;margin-bottom:10px;letter-spacing:.02em}
/* ── top metric cards ── */
.top-cards{display:grid;grid-template-columns:repeat(4,1fr) 220px;gap:8px;margin-bottom:8px;align-items:start}
.mcard{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px 12px;position:relative}
.mcard-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:2px}
.mcard-title{font-size:.75rem;font-weight:600;color:#8b949e;letter-spacing:.04em;text-transform:uppercase}
.badge{font-size:.65rem;font-weight:700;padding:1px 7px;border-radius:20px;letter-spacing:.03em}
.badge-ok  {background:#1a3829;color:#56d364}
.badge-warn{background:#3d2e08;color:#e3b341}
.badge-alrt{background:#3d1212;color:#f85149}
.badge-none{background:#21262d;color:#8b949e}
.mcard-val{font-size:1.9rem;font-weight:700;line-height:1.05;color:#e6edf3;margin:3px 0 2px}
.mcard-val small{font-size:.75rem;font-weight:400;color:#8b949e}
.mcard-sub{font-size:.68rem;color:#8b949e;margin-bottom:4px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.spark svg{display:block;width:100%;height:38px}
/* ── quality side panel ── */
.dq-panel{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px 12px}
.dq-title{font-size:.72rem;font-weight:700;color:#8b949e;text-transform:uppercase;letter-spacing:.04em;margin-bottom:8px}
.dq-row{display:flex;align-items:center;gap:6px;margin-bottom:5px}
.dq-name{font-size:.7rem;color:#8b949e;width:52px;flex-shrink:0}
.dq-bar-wrap{flex:1;height:7px;background:#21262d;border-radius:3px;overflow:hidden}
.dq-bar{height:7px;border-radius:3px;transition:width .5s}
.dq-pct{font-size:.68rem;width:28px;text-align:right;flex-shrink:0}
/* ── main grid ── */
.main{display:grid;grid-template-columns:1fr 320px;gap:8px;margin-bottom:8px}
.left-col{display:flex;flex-direction:column;gap:8px}
.right-col{display:flex;flex-direction:column;gap:8px}
/* ── cards ── */
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px 12px}
.card-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px}
.card-title{font-size:.78rem;font-weight:700;color:#c9d1d9}
.card-sub{font-size:.65rem;color:#8b949e}
/* ── chart canvases ── */
canvas{display:block;border-radius:4px;background:#0d1117}
/* ── temp legend ── */
.tleg{display:flex;gap:12px;font-size:.68rem;color:#8b949e;align-items:center}
.tleg-dot{width:7px;height:7px;border-radius:50%;display:inline-block;margin-right:3px;vertical-align:middle}
/* ── alerts table ── */
table{width:100%;border-collapse:collapse;font-size:.72rem}
thead th{color:#8b949e;padding:4px 8px;border-bottom:1px solid #30363d;text-align:left;font-weight:600;white-space:nowrap}
tbody td{padding:4px 8px;border-bottom:1px solid #21262d;white-space:nowrap}
.al-warn td{background:rgba(227,179,65,.07)}
.al-crit td{background:rgba(248,81,73,.07)}
/* ── status bar ── */
#statusBar{font-size:.68rem;color:#8b949e;margin-bottom:8px;padding:4px 10px;background:#161b22;border:1px solid #30363d;border-radius:6px}
/* ── CAN analyzer ── */
.can-wrap{overflow-x:auto;max-height:300px;overflow-y:auto}
.can-wrap table{font-size:.68rem;font-family:'ui-monospace',monospace}
.can-wrap thead th{padding:3px 6px}
.can-wrap tbody td{padding:2px 6px}
.live-badge{font-size:.65rem;font-weight:700;padding:2px 8px;border-radius:20px;background:#1a3829;color:#56d364}
.live-badge.off{background:#21262d;color:#8b949e}
/* ── sig explorer ── */
.sig-group{border:1px solid #21262d;border-radius:6px;margin-bottom:6px;overflow:hidden}
.sig-group-hdr{padding:4px 10px;background:#1c2128;font-size:.7rem;font-weight:600;color:#79c0ff;border-bottom:1px solid #21262d}
.sig-row{cursor:pointer}
.sig-row:hover td{background:#1c2128}
.sig-row.sel td{background:#162032}
/* ── mode panel ── */
.mode-tabs{display:flex;gap:6px;margin-bottom:8px}
.mtab{padding:4px 14px;border-radius:6px;border:1px solid #30363d;background:#21262d;color:#8b949e;cursor:pointer;font-size:.75rem;font-weight:600}
.mtab.on{background:#1c3a5e;border-color:#58a6ff;color:#c9d1d9}
.fld{margin:6px 0}
.fld label{font-size:.72rem;color:#8b949e;display:block;margin-bottom:2px}
.fld select,.fld input[type=text]{padding:5px 8px;background:#21262d;border:1px solid #30363d;border-radius:5px;color:#e6edf3;font-size:.75rem}
.byte-row{display:flex;gap:4px;flex-wrap:wrap;margin:4px 0}
.byte-row>label{display:flex;flex-direction:column;align-items:center;gap:2px;font-size:.65rem;color:#8b949e}
.byte-row input{width:32px;text-align:center;padding:3px;background:#21262d;border:1px solid #30363d;border-radius:4px;color:#c9d1d9;font-family:monospace;font-size:.75rem}
.btn{padding:5px 14px;border-radius:6px;border:1px solid #30363d;background:#1c3a5e;color:#c9d1d9;cursor:pointer;font-size:.75rem;font-weight:600;margin-right:6px}
.btn:hover{background:#1f4878}
.btn-sm{padding:3px 10px;font-size:.68rem}
#txStat{margin-top:5px;font-size:.68rem;color:#8b949e;min-height:1.2em}
/* ── full-width sections ── */
.full{margin-bottom:8px}
@media(max-width:900px){.top-cards{grid-template-columns:repeat(2,1fr)}.main{grid-template-columns:1fr}}
</style></head><body>
<h1>Vehicle Telemetry Dashboard</h1>
<div id="statusBar">Connecting…</div>

<!-- ══ TOP ROW: 4 metric cards + data quality panel ══ -->
<div class="top-cards">

  <!-- RPM -->
  <div class="mcard">
    <div class="mcard-head"><span class="mcard-title">RPM</span><span class="badge badge-none" id="b0C">NO DATA</span></div>
    <div class="mcard-val" id="v0C">—<small> rpm</small></div>
    <div class="mcard-sub" id="s0C">Avg — rpm</div>
    <div class="spark"><svg id="sk0C" viewBox="0 0 260 38" preserveAspectRatio="none"><line x1="0" y1="30" x2="260" y2="30" stroke="#30363d" stroke-width="1"/></svg></div>
  </div>

  <!-- Coolant Temp -->
  <div class="mcard">
    <div class="mcard-head"><span class="mcard-title">Coolant Temp</span><span class="badge badge-none" id="b05">NO DATA</span></div>
    <div class="mcard-val" id="v05">—<small> °C</small></div>
    <div class="mcard-sub" id="s05">Range —–— °C</div>
    <div class="spark"><svg id="sk05" viewBox="0 0 260 38" preserveAspectRatio="none"><line x1="0" y1="30" x2="260" y2="30" stroke="#30363d" stroke-width="1"/></svg></div>
  </div>

  <!-- Fuel Level -->
  <div class="mcard">
    <div class="mcard-head"><span class="mcard-title">Fuel Level</span><span class="badge badge-none" id="b2F">NO DATA</span></div>
    <div class="mcard-val" id="v2F">—<small> %</small></div>
    <div class="mcard-sub" id="s2F">Most common —</div>
    <div class="spark"><svg id="sk2F" viewBox="0 0 260 38" preserveAspectRatio="none"><line x1="0" y1="30" x2="260" y2="30" stroke="#30363d" stroke-width="1"/></svg></div>
  </div>

  <!-- Battery Voltage -->
  <div class="mcard">
    <div class="mcard-head"><span class="mcard-title">Battery Voltage</span><span class="badge badge-none" id="b42">NO DATA</span></div>
    <div class="mcard-val" id="v42">—<small> V</small></div>
    <div class="mcard-sub" id="s42">Avg — V</div>
    <div class="spark"><svg id="sk42" viewBox="0 0 260 38" preserveAspectRatio="none"><line x1="0" y1="30" x2="260" y2="30" stroke="#30363d" stroke-width="1"/></svg></div>
  </div>

  <!-- Data Quality panel (top-right, next to metric cards) -->
  <div class="dq-panel">
    <div class="dq-title">Data Quality <span style="font-weight:400;text-transform:none">Missing % by Metric</span></div>
    <div id="dqRows"></div>
  </div>

</div><!-- end top-cards -->

<!-- ══ MAIN GRID: left charts + right panels ══ -->
<div class="main">
  <div class="left-col">

    <!-- RPM Over Samples -->
    <div class="card">
      <div class="card-head">
        <span class="card-title">RPM Over Samples</span>
        <span class="card-sub" id="rpmSubLbl">— samples</span>
      </div>
      <canvas id="cRpm" height="200" style="width:100%"></canvas>
    </div>

    <!-- Temperatures -->
    <div class="card">
      <div class="card-head">
        <span class="card-title">Temperatures</span>
        <div class="tleg">
          <span><span class="tleg-dot" style="background:#f85149"></span>Coolant</span>
          <span><span class="tleg-dot" style="background:#e3b341"></span>Intake</span>
          <span><span class="tleg-dot" style="background:#56d364"></span>Ambient</span>
        </div>
      </div>
      <canvas id="cTemp" height="130" style="width:100%"></canvas>
    </div>

    <!-- Alerts -->
    <div class="card">
      <div class="card-head"><span class="card-title">Alerts</span><span class="card-sub" id="alCount">0 active</span></div>
      <table>
        <thead><tr><th>Status</th><th>Rule</th><th>Value</th><th>Time</th></tr></thead>
        <tbody id="alBody"><tr><td colspan="4" style="color:#8b949e;padding:8px">No alerts.</td></tr></tbody>
      </table>
    </div>

  </div><!-- end left-col -->

  <div class="right-col">

    <!-- RPM vs Throttle -->
    <div class="card">
      <div class="card-head"><span class="card-title">RPM vs Throttle</span></div>
      <div style="display:grid;grid-template-columns:1fr 80px;gap:4px;margin-top:4px">
        <canvas id="cScat" height="200" style="width:100%"></canvas>
        <canvas id="cHist" height="200" style="width:100%"></canvas>
      </div>
      <div style="font-size:.63rem;color:#8b949e;margin-top:3px;text-align:center">Throttle (%)</div>
    </div>

  </div><!-- end right-col -->
</div><!-- end main -->

<!-- ══ FULL WIDTH: CAN Analyzer ══ -->
<div class="card full">
  <div class="card-head">
    <span class="card-title">CAN Analyzer</span>
    <span class="live-badge off" id="canBadge">WAITING FOR FRAMES</span>
  </div>
  <div class="can-wrap">
    <table>
      <thead><tr><th>ID</th><th>DLC</th><th>B0</th><th>B1</th><th>B2</th><th>B3</th><th>B4</th><th>B5</th><th>B6</th><th>B7</th><th>Rate</th><th>Frames</th><th>Time</th></tr></thead>
      <tbody id="canBody"><tr><td colspan="13" style="color:#8b949e;padding:10px">Waiting for incoming CAN frames…</td></tr></tbody>
    </table>
  </div>
</div>

<!-- ══ FULL WIDTH: Signal Explorer ══ -->
<div class="card full">
  <div class="card-head">
    <span class="card-title">Signal Explorer — All ECU Signals</span>
    <span class="card-sub" id="sigCnt">0 signals</span>
  </div>
  <div style="max-height:380px;overflow-y:auto" id="sigExp"><p style="color:#8b949e;padding:8px">Waiting for PROFILE/SIG data…</p></div>
</div>

<!-- ══ FULL WIDTH: Operating Mode ══ -->
<div class="card full">
  <div class="card-head"><span class="card-title">Operating Mode</span></div>
  <div class="mode-tabs">
    <button class="mtab on" id="tabR" onclick="setMode('r')">Read Mode</button>
    <button class="mtab"    id="tabW" onclick="setMode('w')">Write Mode</button>
  </div>
  <div id="pnlR"><p style="font-size:.75rem;color:#8b949e">Passive monitor — no CAN frames are transmitted. All data is receive-only.</p></div>
  <div id="pnlW" style="display:none">
    <div style="display:grid;grid-template-columns:160px 200px 1fr;gap:10px;align-items:end">
      <div class="fld"><label>Format</label><select id="txFmt" style="width:100%"><option value="S">Standard 11-bit</option><option value="E">Extended 29-bit</option></select></div>
      <div class="fld"><label>CAN ID (hex)</label><input type="text" id="txId" placeholder="7DF" maxlength="8" style="width:100%"></div>
      <div class="fld" style="display:flex;align-items:center;gap:10px;padding-bottom:1px">
        <div><label>DLC</label><input type="text" id="txDlc" placeholder="8" maxlength="1" style="width:40px"></div>
        <div style="display:flex;align-items:center;gap:4px;margin-top:14px"><input type="checkbox" id="txRtr"><label for="txRtr" style="font-size:.72rem;color:#8b949e">RTR</label></div>
      </div>
    </div>
    <div class="fld"><label>Data Bytes</label>
      <div class="byte-row">
        <label>B0<input type="text" id="b0" maxlength="2" placeholder="00"></label>
        <label>B1<input type="text" id="b1" maxlength="2" placeholder="00"></label>
        <label>B2<input type="text" id="b2" maxlength="2" placeholder="00"></label>
        <label>B3<input type="text" id="b3" maxlength="2" placeholder="00"></label>
        <label>B4<input type="text" id="b4" maxlength="2" placeholder="00"></label>
        <label>B5<input type="text" id="b5" maxlength="2" placeholder="00"></label>
        <label>B6<input type="text" id="b6" maxlength="2" placeholder="00"></label>
        <label>B7<input type="text" id="b7" maxlength="2" placeholder="00"></label>
      </div>
    </div>
    <button class="btn" onclick="sendFrame()">Send Frame</button>
    <button class="btn btn-sm" onclick="loadDef()">Load Default</button>
    <div id="txStat"></div>
  </div>
</div>

<script>
/* ── state ── */
const pid$  = {};  // pid→[{ts,v}] max300
const prof$ = {};  // id→{dlc,rate,label}
const sig$  = {};  // "id:b"→{last,min,max,rate,label,hist:[]}
const frm$  = {};  // id→{n,rate,ts,bytes:[],prev:[],dlc,label}
const alts  = [];  // [{type,rule,val,time}]
const scat  = [];  // [{thr,rpm}] max500
const seen$ = {};  // pid→epoch
let   sock  = null;
const $     = id => document.getElementById(id);
const clamp = (v,lo,hi) => Math.max(lo,Math.min(hi,v));
const avg   = a  => a.length ? a.reduce((s,x)=>s+x,0)/a.length : 0;

/* ── metric card config ── */
const CARDS = {
  '0C':{el:'0C', unit:'rpm',  color:'#58a6ff', wdir:null,  wval:0,    wlbl:''},
  '05':{el:'05', unit:'°C',   color:'#3fb950', wdir:'>=',  wval:105,  wlbl:'Hot'},
  '2F':{el:'2F', unit:'%',    color:'#e3b341', wdir:'<=',  wval:30,   wlbl:'Low'},
  '42':{el:'42', unit:'V',    color:'#f85149', wdir:'<=',  wval:12.0, wlbl:'Alert'},
};

/* ── sparkline ── */
function drawSpark(svgId, samples, color){
  const el=$(svgId); if(!el||samples.length<2) return;
  const W=260,H=38,PAD=4;
  const mn=Math.min(...samples), mx=Math.max(...samples), rng=Math.max(mx-mn,0.001);
  const pts=samples.map((v,i)=>{
    const x=(i/(samples.length-1))*W;
    const y=H-PAD-((v-mn)/rng)*(H-PAD*2);
    return x.toFixed(1)+','+y.toFixed(1);
  }).join(' ');
  el.innerHTML='<polyline points="'+pts+'" fill="none" stroke="'+color+'" stroke-width="1.8"/>';
}

/* ── update metric card ── */
function updCard(pid, val){
  const c=CARDS[pid]; if(!c) return;
  const smp=(pid$[pid]||[]).map(x=>x.v);
  /* value */
  const d=pid==='42'?val.toFixed(2):pid==='0C'?Math.round(val).toString():val.toFixed(1);
  $('v'+c.el).innerHTML=d+'<small> '+c.unit+'</small>';
  /* badge */
  const bEl=$('b'+c.el);
  let cls='badge ', txt='';
  if(c.wdir===null){ cls+='badge-ok'; txt='LIVE'; }
  else {
    const trig=(c.wdir==='>='&&val>=c.wval)||(c.wdir==='<='&&val<=c.wval);
    if(trig){ cls+=(c.wdir==='>='?'badge-warn':'badge-alrt'); txt=c.wlbl; }
    else     { cls+='badge-ok'; txt='OK'; }
  }
  bEl.className=cls; bEl.textContent=txt;
  /* sub-label */
  const sEl=$('s'+c.el);
  if(pid==='0C'){
    sEl.textContent='Avg '+Math.round(avg(smp))+' rpm';
  } else if(pid==='05'){
    const mn=smp.length?Math.min(...smp).toFixed(0):'—';
    const mx=smp.length?Math.max(...smp).toFixed(0):'—';
    sEl.textContent='Range '+mn+'–'+mx+' °C';
  } else if(pid==='2F'){
    const freq={};
    smp.forEach(v=>{const k=Math.round(v);freq[k]=(freq[k]||0)+1;});
    let mV='—',mC=0;
    for(const[k,c2] of Object.entries(freq)) if(c2>mC){mC=c2;mV=k;}
    sEl.textContent='Most common '+(mV!=='—'?mV+'% ('+mC+'x)':'—');
  } else if(pid==='42'){
    sEl.textContent='Avg '+avg(smp).toFixed(2)+' V';
  }
  /* sparkline */
  drawSpark('sk'+c.el, smp, c.color);
}

/* ── data quality ── */
const DQ=[
  {pid:'0C',name:'RPM'},{pid:'05',name:'Coolant'},{pid:'0F',name:'Intake'},
  {pid:'46',name:'Ambient'},{pid:'2F',name:'Fuel'},{pid:'42',name:'Voltage'},
  {pid:'10',name:'MAF'},{pid:'04',name:'Load'},
];
function updQuality(){
  const now=Date.now();
  $('dqRows').innerHTML=DQ.map(({pid,name})=>{
    const ls=seen$[pid];
    const miss=ls?Math.min(100,Math.floor((now-ls)/1000)):100;
    const pct=100-miss;
    const col=miss<30?'#56d364':miss<70?'#e3b341':'#f85149';
    return `<div class="dq-row">
      <span class="dq-name">${name}</span>
      <div class="dq-bar-wrap"><div class="dq-bar" style="width:${pct}%;background:${col}"></div></div>
      <span class="dq-pct" style="color:${col}">${miss}%</span>
    </div>`;
  }).join('');
}

/* ── alerts ── */
const ALT_RULES=[
  {pid:'05',rule:'Coolant Temp High', unit:'°C', chk:v=>v>=105, type:'warn'},
  {pid:'2F',rule:'Fuel Level Low',    unit:'%',  chk:v=>v<=30,  type:'warn'},
  {pid:'42',rule:'Battery Voltage Low',unit:'V', chk:v=>v<=12.0,type:'crit'},
];
function chkAlerts(pid,val){
  for(const r of ALT_RULES){
    if(r.pid!==pid) continue;
    if(r.chk(val)){
      const t=new Date().toTimeString().slice(0,8);
      const v=r.unit==='V'?val.toFixed(2):Math.round(val);
      if(!alts.length||alts[0].rule!==r.rule){
        alts.unshift({type:r.type,rule:r.rule,val:v+' '+r.unit,time:t});
        if(alts.length>20) alts.pop();
      }
    }
  }
}
function updAlerts(){
  $('alCount').textContent=alts.length+' active';
  if(!alts.length){
    $('alBody').innerHTML='<tr><td colspan="4" style="color:#8b949e;padding:8px">No alerts.</td></tr>';
    return;
  }
  $('alBody').innerHTML=alts.map(a=>{
    const ic=a.type==='crit'?'<span style="color:#f85149">&#9679;</span>':'<span style="color:#e3b341">&#9651;</span>';
    return `<tr class="${a.type==='crit'?'al-crit':'al-warn'}"><td>${ic}</td><td>${a.rule}</td><td style="font-family:monospace">${a.val}</td><td style="color:#8b949e">${a.time}</td></tr>`;
  }).join('');
}

/* ── RPM chart ── */
function drawRpm(){
  const c=$('cRpm'); if(!c) return;
  const dpr=window.devicePixelRatio||1, W=c.clientWidth||600, H=200;
  c.width=W*dpr; c.height=H*dpr;
  const ctx=c.getContext('2d'); ctx.scale(dpr,dpr);
  const L=38,R=8,T=12,B=22;
  ctx.fillStyle='#0d1117'; ctx.fillRect(0,0,W,H);
  const pts=(pid$['0C']||[]).map(x=>x.v);
  const YMAX=6000;
  /* grid */
  ctx.strokeStyle='#21262d'; ctx.lineWidth=1;
  ctx.fillStyle='#6e7681'; ctx.font='9px system-ui'; ctx.textAlign='right';
  [0,1000,2000,3000,4000,5000,6000].forEach(y=>{
    const py=T+(YMAX-y)/YMAX*(H-T-B);
    ctx.beginPath(); ctx.moveTo(L,py); ctx.lineTo(W-R,py); ctx.stroke();
    if(y<6000) ctx.fillText(y,L-3,py+3);
  });
  /* x labels */
  ctx.textAlign='center'; ctx.fillStyle='#6e7681';
  [50,100,150,200,250,300].forEach(n=>{
    if(n>pts.length) return;
    const px=L+(n/300)*(W-L-R);
    ctx.beginPath(); ctx.moveTo(px,H-B); ctx.lineTo(px,H-B+3); ctx.strokeStyle='#30363d'; ctx.stroke();
    ctx.fillText(n,px,H-6);
  });
  /* line */
  if(pts.length>=2){
    ctx.strokeStyle='#58a6ff'; ctx.lineWidth=1.6; ctx.lineJoin='round';
    ctx.beginPath();
    pts.forEach((v,i)=>{
      const px=L+(i/Math.max(pts.length-1,1))*(W-L-R);
      const py=T+(YMAX-clamp(v,0,YMAX))/YMAX*(H-T-B);
      i?ctx.lineTo(px,py):ctx.moveTo(px,py);
    });
    ctx.stroke();
  } else {
    ctx.fillStyle='#30363d'; ctx.textAlign='center'; ctx.font='12px system-ui';
    ctx.fillText('Waiting for RPM…',W/2,H/2);
  }
  $('rpmSubLbl').textContent=pts.length+' samples';
}

/* ── Temperature chart ── */
function drawTemp(){
  const c=$('cTemp'); if(!c) return;
  const dpr=window.devicePixelRatio||1, W=c.clientWidth||600, H=130;
  c.width=W*dpr; c.height=H*dpr;
  const ctx=c.getContext('2d'); ctx.scale(dpr,dpr);
  const L=32,R=8,T=8,B=18,YMIN=50,YMAX=150;
  ctx.fillStyle='#0d1117'; ctx.fillRect(0,0,W,H);
  ctx.strokeStyle='#21262d'; ctx.lineWidth=1;
  ctx.fillStyle='#6e7681'; ctx.font='9px system-ui'; ctx.textAlign='right';
  [50,75,100,125,150].forEach(y=>{
    const py=T+(YMAX-y)/(YMAX-YMIN)*(H-T-B);
    ctx.beginPath(); ctx.moveTo(L,py); ctx.lineTo(W-R,py); ctx.stroke();
    ctx.fillText(y,L-3,py+3);
  });
  [
    {pid:'05',col:'#f85149'},
    {pid:'0F',col:'#e3b341'},
    {pid:'46',col:'#56d364'},
  ].forEach(({pid,col})=>{
    const d=(pid$[pid]||[]).map(x=>x.v);
    if(d.length<2) return;
    ctx.strokeStyle=col; ctx.lineWidth=1.6; ctx.lineJoin='round';
    ctx.beginPath();
    d.forEach((v,i)=>{
      const px=L+(i/Math.max(d.length-1,1))*(W-L-R);
      const py=T+(YMAX-clamp(v,YMIN,YMAX))/(YMAX-YMIN)*(H-T-B);
      i?ctx.lineTo(px,py):ctx.moveTo(px,py);
    });
    ctx.stroke();
  });
}

/* ── Scatter + histogram ── */
function drawScat(){
  /* scatter */
  const cs=$('cScat'); if(!cs) return;
  const dpr=window.devicePixelRatio||1;
  {
    const W=cs.clientWidth||220, H=200;
    cs.width=W*dpr; cs.height=H*dpr;
    const ctx=cs.getContext('2d'); ctx.scale(dpr,dpr);
    const L=30,R=4,T=6,B=18;
    ctx.fillStyle='#0d1117'; ctx.fillRect(0,0,W,H);
    ctx.strokeStyle='#21262d'; ctx.lineWidth=1;
    ctx.fillStyle='#6e7681'; ctx.font='8px system-ui';
    [0,1000,2000,3000,4000].forEach(y=>{
      const py=T+(4000-y)/4000*(H-T-B);
      ctx.beginPath(); ctx.moveTo(L,py); ctx.lineTo(W-R,py); ctx.stroke();
      ctx.textAlign='right'; ctx.fillText(y||'0',L-2,py+3);
    });
    [0,25,50,75,100].forEach(x=>{
      const px=L+x/100*(W-L-R);
      ctx.beginPath(); ctx.moveTo(px,T); ctx.lineTo(px,H-B); ctx.stroke();
      ctx.textAlign='center'; ctx.fillText(x,px,H-4);
    });
    ctx.fillStyle='rgba(88,166,255,0.6)';
    scat.forEach(p=>{
      const px=L+clamp(p.thr,0,100)/100*(W-L-R);
      const py=T+(4000-clamp(p.rpm,0,4000))/4000*(H-T-B);
      ctx.beginPath(); ctx.arc(px,py,2.5,0,Math.PI*2); ctx.fill();
    });
  }
  /* histogram */
  const ch=$('cHist'); if(!ch) return;
  {
    const W=ch.clientWidth||80, H=200;
    ch.width=W*dpr; ch.height=H*dpr;
    const ctx=ch.getContext('2d'); ctx.scale(dpr,dpr);
    const L=4,R=4,T=6,B=18;
    ctx.fillStyle='#0d1117'; ctx.fillRect(0,0,W,H);
    const bkt=new Array(10).fill(0);
    scat.forEach(p=>{ bkt[clamp(Math.floor(p.thr/10),0,9)]++; });
    const mx=Math.max(...bkt,1);
    const bw=(W-L-R)/10;
    bkt.forEach((n,i)=>{
      const bh=(n/mx)*(H-T-B);
      const x=L+i*bw;
      ctx.fillStyle='#1c3a5e'; ctx.fillRect(x+1,T+H-T-B-bh,bw-2,bh);
      ctx.fillStyle='#58a6ff'; ctx.fillRect(x+1,T+H-T-B-bh,bw-2,2);
    });
    ctx.fillStyle='#6e7681'; ctx.font='7px system-ui'; ctx.textAlign='center';
    bkt.forEach((_,i)=>ctx.fillText(i*10+'%',L+i*bw+bw/2,H-4));
  }
}

/* ── CAN Analyzer ── */
function updCAN(){
  const ids=Object.keys(frm$).sort((a,b)=>(frm$[b].rate||0)-(frm$[a].rate||0)).slice(0,60);
  if(!ids.length){
    $('canBody').innerHTML='<tr><td colspan="13" style="color:#8b949e;padding:8px">Waiting…</td></tr>';
    $('canBadge').textContent='WAITING FOR FRAMES'; $('canBadge').className='live-badge off';
    return;
  }
  $('canBadge').textContent='LIVE · '+ids.length+' IDs'; $('canBadge').className='live-badge';
  $('canBody').innerHTML=ids.map(id=>{
    const s=frm$[id];
    const t=new Date(s.ts).toTimeString().slice(0,8);
    const bCells=Array.from({length:8},(_,i)=>{
      const b=s.bytes[i]||'--', p=s.prev[i];
      const col=b!=='--'&&p!==undefined&&p!==b?'#f85149':(b!=='--'?'#56d364':'#30363d');
      return `<td style="color:${col}">${b}</td>`;
    }).join('');
    return `<tr><td>${id}</td><td>${s.dlc||'—'}</td>${bCells}<td>${s.rate?s.rate.toFixed(1):'—'} Hz</td><td>${s.n}</td><td style="color:#8b949e">${t}</td></tr>`;
  }).join('');
}

/* ── Signal Explorer ── */
function tsSVG(hist){
  if(!hist||hist.length<2) return '<svg width="56" height="18"></svg>';
  const W=56,H=18, mn=Math.min(...hist),mx=Math.max(...hist),rng=Math.max(mx-mn,1);
  const pts=hist.map((v,i)=>{
    const x=(i/(hist.length-1))*W, y=H-2-((v-mn)/rng)*(H-4);
    return x.toFixed(1)+','+y.toFixed(1);
  }).join(' ');
  return `<svg width="${W}" height="${H}" viewBox="0 0 ${W} ${H}"><polyline points="${pts}" fill="none" stroke="#58a6ff" stroke-width="1.2"/></svg>`;
}
let selSig=null;
function updSigs(){
  const keys=Object.keys(sig$);
  $('sigCnt').textContent=keys.length+' signals';
  if(!keys.length){$('sigExp').innerHTML='<p style="color:#8b949e;padding:8px">Waiting for PROFILE/SIG data…</p>';return;}
  const byId={};
  keys.forEach(k=>{const id=k.split(':')[0];(byId[id]=byId[id]||[]).push(k);});
  $('sigExp').innerHTML=Object.keys(byId).sort().map(id=>{
    const lbl=prof$[id]?prof$[id].label:'—';
    const rows=byId[id].sort((a,b)=>+a.split(':')[1]-+b.split(':')[1]).map(k=>{
      const s=sig$[k], sel=selSig===k;
      return `<tr class="sig-row${sel?' sel':''}" onclick="selSig='${k}';updSigs()">
        <td style="font-family:monospace">${id}</td><td style="color:#79c0ff">${lbl}</td>
        <td>${s.byte}</td><td>${s.label||'—'}</td>
        <td style="font-family:monospace">${s.last}</td>
        <td style="font-family:monospace;color:#6e7681">${s.min}</td>
        <td style="font-family:monospace;color:#6e7681">${s.max}</td>
        <td>${s.rate?s.rate.toFixed(1):'—'}</td>
        <td>${tsSVG(s.hist)}</td></tr>`;
    }).join('');
    return `<div class="sig-group"><div class="sig-group-hdr">0x${id} — ${lbl}</div>
      <table><thead><tr><th>ID</th><th>Frame</th><th>Byte</th><th>Signal</th><th>Last</th><th>Min</th><th>Max</th><th>Rate</th><th>Trend</th></tr></thead>
      <tbody>${rows}</tbody></table></div>`;
  }).join('');
}

/* ── mode switch ── */
function setMode(m){
  const w=m==='w';
  $('pnlW').style.display=w?'block':'none';
  $('pnlR').style.display=w?'none':'block';
  $('tabW').className='mtab'+(w?' on':'');
  $('tabR').className='mtab'+(w?'':' on');
}

/* ── send frame ── */
function sendFrame(){
  if(!sock||sock.readyState!==1){$('txStat').textContent='Not connected.';return;}
  const fmt=$('txFmt').value, id=$('txId').value.trim().toUpperCase();
  const bytes=['b0','b1','b2','b3','b4','b5','b6','b7'].map(i=>($( i).value||'00').trim().toUpperCase().padStart(2,'0'));
  const rtr=$('txRtr').checked?1:0;
  let dlc=parseInt($('txDlc').value)||0;
  if(!dlc){dlc=8;for(let i=7;i>=0;i--){if(bytes[i]!=='00'){dlc=i+1;break;}}if(!dlc)dlc=1;}
  dlc=clamp(dlc,1,8);
  const maxId=fmt==='S'?0x7FF:0x1FFFFFFF;
  if(!/^[0-9A-F]{1,8}$/.test(id)||parseInt(id,16)>maxId){$('txStat').textContent='Invalid ID.';return;}
  const data=bytes.slice(0,dlc).join('');
  sock.send(JSON.stringify({op:'tx',format:fmt,id,rtr,dlc,data}));
  $('txStat').textContent='Sent → TX,'+fmt+','+id+','+rtr+','+dlc+','+data;
}
function loadDef(){
  const id=$('txId').value.trim().toUpperCase();
  if(!id){$('txStat').textContent='Enter a CAN ID first.';return;}
  for(let i=0;i<8;i++){const k=id+':'+i;if(sig$[k])$('b'+i).value=sig$[k].last||'00';}
  const p=prof$[id]; if(p&&p.dlc) $('txDlc').value=p.dlc;
  $('txStat').textContent='Loaded defaults for '+id;
}

/* ── message handler ── */
function onMsg(m){
  if(m.type==='status'){
    $('statusBar').textContent=(m.locked?'🔒 Locked ':'🔍 Scanning ')
      +(m.bitrate?m.bitrate+' bps':'—')
      +' | Frames: '+(m.frames||0)
      +(m.dwell?' | Dwell: '+m.dwell+' ms':'');
    return;
  }
  if(m.type==='frame'){
    const id=m.id; if(!id) return;
    const s=frm$[id]||(frm$[id]={n:0,rate:0,ts:0,bytes:[],prev:[],dlc:0,label:''});
    s.prev=[...s.bytes]; s.n++; s.ts=Date.now(); s.dlc=m.dlc;
    const hex=(m.data||'').replace(/[^0-9a-fA-F]/g,'');
    s.bytes=Array.from({length:8},(_,i)=>hex.length>=(i+1)*2?hex.substr(i*2,2).toUpperCase():'--');
    if(prof$[id]) s.rate=prof$[id].rate;
    return;
  }
  if(m.type==='pid'){
    const pid=(m.pid||'').toUpperCase(), val=parseFloat(m.value);
    if(isNaN(val)) return;
    if(!pid$[pid]) pid$[pid]=[];
    pid$[pid].push({ts:Date.now(),v:val});
    if(pid$[pid].length>300) pid$[pid].shift();
    seen$[pid]=Date.now();
    updCard(pid,val);
    chkAlerts(pid,val); updAlerts();
    if(pid==='0C'&&pid$['11']&&pid$['11'].length){
      const thr=pid$['11'][pid$['11'].length-1].v;
      scat.push({thr,rpm:val}); if(scat.length>500) scat.shift();
    }
    return;
  }
  if(m.type==='profile'){
    const id=(m.id||'').toUpperCase();
    prof$[id]={dlc:m.dlc,rate:parseFloat(m.rate)||0,label:m.label||''};
    if(frm$[id]) frm$[id].label=m.label||'';
    if(frm$[id]) frm$[id].rate=parseFloat(m.rate)||0;
    return;
  }
  if(m.type==='sig'){
    const id=(m.id||'').toUpperCase(), b=m.byte||0, k=id+':'+b;
    const s=sig$[k]||(sig$[k]={last:'',min:'',max:'',rate:0,label:'',hist:[],byte:b});
    s.last=m.last||''; s.min=m.min||''; s.max=m.max||'';
    s.rate=parseFloat(m.rate)||0; s.label=m.label||''; s.byte=b;
    const n=parseInt(m.last,16);
    if(!isNaN(n)){s.hist.push(n);if(s.hist.length>20)s.hist.shift();}
    return;
  }
  if(m.type==='tx'){ $('txStat').textContent='STM32: '+(m.message||''); return; }
  if(m.type==='stm32_progress'){ handleFwProgress(m); return; }
}

/* ── websocket ── */
function connect(){
  $('statusBar').textContent='Connecting…';
  try{ sock=new WebSocket('ws://'+location.hostname+':81/'); }
  catch(e){ $('statusBar').textContent='WS error: '+e.message; setTimeout(connect,2000); return; }
  sock.onopen =()=>$('statusBar').textContent='Connected — waiting for STM32…';
  sock.onclose=()=>{ $('statusBar').textContent='Disconnected — reconnecting…'; setTimeout(connect,1500); };
  sock.onerror=()=>$('statusBar').textContent='WebSocket error';
  sock.onmessage=e=>{ try{onMsg(JSON.parse(e.data));}catch{} };
}

/* ── 1s redraw loop ── */
setInterval(()=>{ drawRpm(); drawTemp(); drawScat(); updQuality(); updCAN(); updSigs(); }, 1000);
window.addEventListener('resize',()=>{ drawRpm(); drawTemp(); drawScat(); });
connect();
</script>

<!-- ══ FULL WIDTH: Firmware Update ══ -->
<div class="card full" id="fwCard">
  <div class="card-head">
    <span class="card-title">⚡ Firmware Update</span>
    <span class="card-sub" id="fwStatus">Ready</span>
  </div>
  <div style="display:grid;grid-template-columns:1fr 1fr;gap:16px;margin-top:4px">

    <!-- ESP32 update -->
    <div style="background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:12px">
      <div style="font-size:.78rem;font-weight:700;color:#79c0ff;margin-bottom:6px">ESP32 Dashboard Firmware</div>
      <div style="font-size:.68rem;color:#8b949e;margin-bottom:8px">Upload a new <code style="background:#21262d;padding:1px 4px;border-radius:3px">.bin</code> built with PlatformIO. The ESP32 will reboot automatically after flashing.</div>
      <div id="esp32DropZone" style="border:2px dashed #30363d;border-radius:6px;padding:14px;text-align:center;cursor:pointer;transition:border-color .2s" ondragover="event.preventDefault();this.style.borderColor='#58a6ff'" ondragleave="this.style.borderColor='#30363d'" ondrop="dropFw(event,'esp32')">
        <div style="font-size:.72rem;color:#8b949e">Drag &amp; drop <b>.bin</b> here, or</div>
        <label style="margin-top:6px;display:inline-block">
          <input type="file" id="esp32File" accept=".bin" style="display:none" onchange="selectFw('esp32')">
          <span class="btn btn-sm" style="margin:0" onclick="document.getElementById('esp32File').click()">Browse</span>
        </label>
        <div id="esp32FileName" style="font-size:.68rem;color:#8b949e;margin-top:4px">No file selected</div>
      </div>
      <div style="margin-top:8px">
        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px">
          <span style="font-size:.68rem;color:#8b949e" id="esp32Pct">0%</span>
          <button class="btn btn-sm" onclick="flashESP32()" id="esp32Btn" disabled>Flash ESP32</button>
        </div>
        <div style="height:6px;background:#21262d;border-radius:3px;overflow:hidden">
          <div id="esp32Bar" style="height:6px;width:0%;background:#58a6ff;border-radius:3px;transition:width .2s"></div>
        </div>
        <div id="esp32Msg" style="font-size:.65rem;color:#8b949e;margin-top:4px;min-height:1em"></div>
      </div>
    </div>

    <!-- STM32 update -->
    <div style="background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:12px">
      <div style="font-size:.78rem;font-weight:700;color:#e3b341;margin-bottom:6px">STM32F407 Firmware</div>
      <div style="font-size:.68rem;color:#8b949e;margin-bottom:4px">Upload a <code style="background:#21262d;padding:1px 4px;border-radius:3px">.bin</code> built with <code style="background:#21262d;padding:1px 4px;border-radius:3px">make firmware</code>. Requires <b>GPIO4→BOOT0</b> and <b>GPIO5→NRST</b> wired.</div>
      <div style="font-size:.65rem;background:#1c2128;border:1px solid #30363d;border-radius:4px;padding:6px;margin-bottom:8px;color:#8b949e">
        <b style="color:#e3b341">⚠ Wiring required:</b> ESP32 GPIO4 → STM32 BOOT0 &nbsp;|&nbsp; ESP32 GPIO5 → STM32 NRST (100Ω)
      </div>
      <div id="stm32DropZone" style="border:2px dashed #30363d;border-radius:6px;padding:14px;text-align:center;cursor:pointer;transition:border-color .2s" ondragover="event.preventDefault();this.style.borderColor='#e3b341'" ondragleave="this.style.borderColor='#30363d'" ondrop="dropFw(event,'stm32')">
        <div style="font-size:.72rem;color:#8b949e">Drag &amp; drop <b>.bin</b> here, or</div>
        <label style="margin-top:6px;display:inline-block">
          <input type="file" id="stm32File" accept=".bin" style="display:none" onchange="selectFw('stm32')">
          <span class="btn btn-sm" style="margin:0;background:#2d2008;border-color:#e3b341;color:#e3b341" onclick="document.getElementById('stm32File').click()">Browse</span>
        </label>
        <div id="stm32FileName" style="font-size:.68rem;color:#8b949e;margin-top:4px">No file selected</div>
      </div>
      <div style="margin-top:8px">
        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px">
          <span style="font-size:.68rem;color:#8b949e" id="stm32Pct">0%</span>
          <button class="btn btn-sm" onclick="flashSTM32()" id="stm32Btn" disabled style="background:#2d2008;border-color:#e3b341;color:#e3b341">Flash STM32</button>
        </div>
        <div style="height:6px;background:#21262d;border-radius:3px;overflow:hidden">
          <div id="stm32Bar" style="height:6px;width:0%;background:#e3b341;border-radius:3px;transition:width .2s"></div>
        </div>
        <div id="stm32Msg" style="font-size:.65rem;color:#8b949e;margin-top:4px;min-height:1em"></div>
      </div>
    </div>

  </div>
</div>

<script>
/* ═══════════════════════════════════════════════
   FIRMWARE UPDATE
   ═══════════════════════════════════════════════ */
const fwFiles={esp32:null,stm32:null};

function selectFw(target){
  const f=document.getElementById(target+'File').files[0];
  if(!f) return;
  fwFiles[target]=f;
  document.getElementById(target+'FileName').textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';
  document.getElementById(target+'Btn').disabled=false;
}
function dropFw(ev,target){
  event.preventDefault();
  document.getElementById(target+'DropZone').style.borderColor='#30363d';
  const f=ev.dataTransfer.files[0];
  if(!f||!f.name.endsWith('.bin')){
    document.getElementById(target+'Msg').textContent='Only .bin files accepted.'; return;
  }
  fwFiles[target]=f;
  document.getElementById(target+'FileName').textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';
  document.getElementById(target+'Btn').disabled=false;
  document.getElementById(target+'Msg').textContent='';
}

function flashESP32(){
  const f=fwFiles.esp32; if(!f) return;
  document.getElementById('esp32Btn').disabled=true;
  document.getElementById('esp32Msg').textContent='Uploading…';
  const xhr=new XMLHttpRequest();
  xhr.open('POST','/update',true);
  xhr.upload.onprogress=e=>{
    if(e.lengthComputable){
      const pct=Math.round(e.loaded/e.total*100);
      document.getElementById('esp32Bar').style.width=pct+'%';
      document.getElementById('esp32Pct').textContent=pct+'%';
    }
  };
  xhr.onload=()=>{
    if(xhr.status===200){
      document.getElementById('esp32Msg').style.color='#56d364';
      document.getElementById('esp32Msg').textContent='✓ Flash OK — ESP32 rebooting…';
      document.getElementById('esp32Bar').style.background='#56d364';
    } else {
      document.getElementById('esp32Msg').style.color='#f85149';
      document.getElementById('esp32Msg').textContent='✗ Error: '+xhr.responseText;
      document.getElementById('esp32Btn').disabled=false;
    }
  };
  xhr.onerror=()=>{
    document.getElementById('esp32Msg').style.color='#f85149';
    document.getElementById('esp32Msg').textContent='✗ Upload failed — connection lost (normal after reboot)';
    document.getElementById('esp32Btn').disabled=false;
  };
  xhr.send(f);
}

function flashSTM32(){
  const f=fwFiles.stm32; if(!f) return;
  document.getElementById('stm32Btn').disabled=true;
  document.getElementById('stm32Msg').textContent='Uploading binary to ESP32…';
  document.getElementById('stm32Bar').style.width='5%';
  const xhr=new XMLHttpRequest();
  xhr.open('POST','/update-stm32',true);
  xhr.upload.onprogress=e=>{
    if(e.lengthComputable){
      const pct=Math.round(e.loaded/e.total*50); // upload = 0-50%
      document.getElementById('stm32Bar').style.width=pct+'%';
      document.getElementById('stm32Pct').textContent=pct+'%';
    }
  };
  xhr.onload=()=>{
    if(xhr.status===200){
      document.getElementById('stm32Msg').style.color='#56d364';
      document.getElementById('stm32Msg').textContent='✓ '+xhr.responseText;
      document.getElementById('stm32Bar').style.width='100%';
      document.getElementById('stm32Bar').style.background='#56d364';
      document.getElementById('stm32Pct').textContent='100%';
    } else {
      document.getElementById('stm32Msg').style.color='#f85149';
      document.getElementById('stm32Msg').textContent='✗ Error: '+xhr.responseText;
      document.getElementById('stm32Btn').disabled=false;
    }
  };
  xhr.onerror=()=>{
    document.getElementById('stm32Msg').style.color='#f85149';
    document.getElementById('stm32Msg').textContent='✗ Connection lost during upload.';
    document.getElementById('stm32Btn').disabled=false;
  };
  // Poll for STM32 flash progress via WebSocket messages
  xhr.send(f);
}

/* Listen for stm32_progress messages from WebSocket */
function handleFwProgress(m){
  if(m.type!=='stm32_progress') return;
  const pct=50+Math.round((m.pct||0)/2); // flashing = 50-100%
  document.getElementById('stm32Bar').style.width=pct+'%';
  document.getElementById('stm32Pct').textContent=pct+'%';
  document.getElementById('stm32Msg').textContent=m.msg||'';
}
</script>
</body></html>)HTML";



/* ═══════════════════════════════════════════════════════════════════
   C++ HELPERS
   ═══════════════════════════════════════════════════════════════════ */
static String csvField(const char *s, int idx) {
    int start = 0, field = 0;
    while (s[start]) {
        int end = start;
        while (s[end] && s[end] != ',') end++;
        if (field == idx) return String(s + start).substring(0, end - start);
        field++;
        start = end + (s[end] == ',' ? 1 : 0);
        if (!s[end]) break;
    }
    return String("");
}

static int countCommas(const char *s) {
    int n = 0; for (; *s; s++) if (*s == ',') n++; return n;
}

/* ═══════════════════════════════════════════════════════════════════
   STATUS EMITTER
   ═══════════════════════════════════════════════════════════════════ */
static void emitStatus() {
    StaticJsonDocument<256> d;
    d["type"]    = "status";
    d["locked"]  = locked;
    d["bitrate"] = (int)bitrate;
    d["frames"]  = (int)framesSeen;
    d["dwell"]   = (int)dwellMs;
    String out; serializeJson(d, out); ws.broadcastTXT(out);
}

/* ═══════════════════════════════════════════════════════════════════
   LINE PARSER
   ═══════════════════════════════════════════════════════════════════ */
static void parseLine(const char *raw, int len) {
    if (len == 0) return;
    if (strncmp(raw, "TX,", 3) == 0) {
        StaticJsonDocument<192> d; d["type"]="tx"; d["message"]=raw;
        String out; serializeJson(d,out); ws.broadcastTXT(out); return;
    }
    if (strncmp(raw, "STATUS,LOCK,", 12) == 0)     { bitrate=(uint32_t)atol(raw+12); locked=true;  emitStatus(); return; }
    if (strncmp(raw, "STATUS,SCAN,", 12) == 0)     { bitrate=(uint32_t)atol(raw+12); locked=false; emitStatus(); return; }
    if (strncmp(raw, "STATUS,REACQUIRE", 16) == 0) { locked=false; bitrate=0;        emitStatus(); return; }
    if (strncmp(raw, "STATUS,DWELL,", 13) == 0)    { dwellMs=(uint32_t)atol(raw+13); emitStatus(); return; }

    int commas = countCommas(raw);

    if (strncmp(raw, "PROFILE,", 8) == 0 && commas >= 5) {
        String id=csvField(raw,1); id.toUpperCase();
        String vm=csvField(raw,4); vm.toUpperCase();
        int dlc = (int)csvField(raw,2).toInt();
        float rate = atof(csvField(raw,3).c_str()) / 10.0f;
        String lbl = csvField(raw,5);
        /* P1.6: static buffer */
        static char pbuf[256];
        int pn = snprintf(pbuf, sizeof pbuf,
            "{\"type\":\"profile\",\"id\":\"%s\",\"dlc\":%d,"
            "\"rate\":%.1f,\"var_mask\":\"%s\",\"label\":\"%s\"}",
            id.c_str(), dlc, (double)rate, vm.c_str(), lbl.c_str());
        if (pn > 0 && pn < (int)sizeof pbuf) ws.broadcastTXT(pbuf, pn);
        return;
    }
    if (strncmp(raw, "SIG,", 4) == 0 && commas >= 7) {
        String id=csvField(raw,1); id.toUpperCase();
        String l=csvField(raw,3);  l.toUpperCase();
        String mn=csvField(raw,4); mn.toUpperCase();
        String mx=csvField(raw,5); mx.toUpperCase();
        int    byt  = (int)csvField(raw,2).toInt();
        float  rate = atof(csvField(raw,6).c_str()) / 10.0f;
        String lbl  = csvField(raw,7);
        /* P1.6: static buffer */
        static char sbuf[256];
        int sn = snprintf(sbuf, sizeof sbuf,
            "{\"type\":\"sig\",\"id\":\"%s\",\"byte\":%d,"
            "\"last\":\"%s\",\"min\":\"%s\",\"max\":\"%s\","
            "\"rate\":%.1f,\"label\":\"%s\"}",
            id.c_str(), byt, l.c_str(), mn.c_str(), mx.c_str(),
            (double)rate, lbl.c_str());
        if (sn > 0 && sn < (int)sizeof sbuf) ws.broadcastTXT(sbuf, sn);
        return;
    }
    if (strncmp(raw, "PID,", 4) == 0 && commas >= 4) {
        String pid=csvField(raw,1); pid.toUpperCase();
        float val = atof(csvField(raw,4).c_str());
        String lbl = csvField(raw,2);
        String unt = csvField(raw,3);
        /* P1.6: static buffer */
        static char pidbuf[256];
        int pn = snprintf(pidbuf, sizeof pidbuf,
            "{\"type\":\"pid\",\"pid\":\"%s\",\"label\":\"%s\","
            "\"unit\":\"%s\",\"value\":%.4f}",
            pid.c_str(), lbl.c_str(), unt.c_str(), (double)val);
        if (pn > 0 && pn < (int)sizeof pidbuf) ws.broadcastTXT(pidbuf, pn);
        return;
    }
    if (commas >= 5) {
        String ts=csvField(raw,0);
        String fmt=csvField(raw,1); fmt.toUpperCase();
        String id=csvField(raw,2);  id.toUpperCase();
        String dlcS=csvField(raw,3), rtrS=csvField(raw,4);
        String pay=csvField(raw,5); pay.toUpperCase();
        if(!ts.length()||!id.length()) return;
        if(fmt!="STD"&&fmt!="EXT") return;
        int dlc=dlcS.toInt(); if(dlc<0||dlc>8) return;
        framesSeen++;
        /* P1.6: static buffer replaces StaticJsonDocument heap allocation */
        static char fbuf[200];
        const char *fmtS = fmt.c_str();
        int fn = snprintf(fbuf, sizeof fbuf,
            "{\"type\":\"frame\",\"ts\":\"%s\",\"id\":\"%s\",\"format\":\"%s\","
            "\"rtr\":%s,\"dlc\":%d,\"data\":\"%s\",\"frames\":%d,"
            "\"locked\":%s,\"bitrate\":%d}",
            ts.c_str(), id.c_str(), fmtS,
            (rtrS.toInt()!=0)?"true":"false", dlc, pay.c_str(),
            (int)framesSeen, locked?"true":"false", (int)bitrate);
        if (fn > 0 && fn < (int)sizeof fbuf) ws.broadcastTXT(fbuf, fn);
        return;
    }
}

/* ═══════════════════════════════════════════════════════════════════
   WS EVENT + HTTP
   ═══════════════════════════════════════════════════════════════════ */
static void handleWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_CONNECTED) { emitStatus(); return; }
    if (type != WStype_TEXT || length == 0 || length > 200) return;
    StaticJsonDocument<256> cmd;
    if (deserializeJson(cmd, payload, length)) return;
    if (strcmp(cmd["op"]|"", "tx") != 0) return;
    String fmt(cmd["format"]|""); fmt.toUpperCase();
    String id(cmd["id"]|"");     id.toUpperCase();
    String data(cmd["data"]|""); data.toUpperCase();
    int dlc=cmd["dlc"]|-1, rtr=cmd["rtr"]|-1;
    uint32_t maxId=(fmt=="S")?0x7FFu:(fmt=="E"?0x1FFFFFFFu:0u);
    bool ok=(fmt=="S"||fmt=="E")&&id.length()&&id.length()<=8&&dlc>=0&&dlc<=8
            &&(rtr==0||rtr==1)&&strtoul(id.c_str(),nullptr,16)<=maxId;
    if(ok&&rtr==0) ok=(data.length()==(size_t)(dlc*2));
    if(!ok){
        StaticJsonDocument<128> e; e["type"]="tx"; e["message"]="TX,ERR,ESP32_VALIDATION_FAILED";
        String out; serializeJson(e,out); ws.broadcastTXT(out); return;
    }
    char buf[90];
    snprintf(buf,sizeof buf,"TX,%s,%s,%d,%d,%s\n",fmt.c_str(),id.c_str(),rtr,dlc,data.c_str());
    Stm32.print(buf);
    Serial.printf("[TX→STM32] %s",buf);
}
static void handleRoot()     { http.send_P(200,"text/html",PAGE); }
static void handleNotFound() { http.send(404,"text/plain","Not found"); }

/* ═══════════════════════════════════════════════════════════════════
   ESP32 OTA — receives .bin via HTTP POST /update
   ═══════════════════════════════════════════════════════════════════ */
static void handleESP32Update() {
    /* Called once after upload completes */
    if (Update.hasError()) {
        http.send(500, "text/plain", "Flash failed: " + String(Update.errorString()));
    } else {
        http.send(200, "text/plain", "OK — rebooting");
        delay(200);
        ESP.restart();
    }
}

static void handleESP32UpdateUpload() {
    HTTPUpload &upload = http.upload();
    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA-ESP32] Start: %s  size=%u\n", upload.filename.c_str(), upload.totalSize);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Serial.printf("[OTA-ESP32] Begin error: %s\n", Update.errorString());
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Serial.printf("[OTA-ESP32] Write error: %s\n", Update.errorString());
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.printf("[OTA-ESP32] Done: %u bytes\n", upload.totalSize);
        } else {
            Serial.printf("[OTA-ESP32] End error: %s\n", Update.errorString());
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
   STM32 UART bootloader flash — receives .bin via HTTP POST /update-stm32
   Implements ST AN3155 UART bootloader protocol manually.
   ═══════════════════════════════════════════════════════════════════ */

/* STM32 bootloader commands */
static constexpr uint8_t BL_GET       = 0x00;
static constexpr uint8_t BL_GET_VER   = 0x01;
/* STM32F407 ROM bootloader requires Extended Erase (0x44), not Standard (0x43).
 * AN3155 §3.3: Standard Erase is NOT supported on STM32F2/F4/F7/L4/H7 devices.
 * The payload (0xFF,0xFF = global mass erase + checksum 0x00) is correct for
 * Extended Erase.  Only this command byte needed to change. */
static constexpr uint8_t BL_ERASE     = 0x44; /* Extended Erase — F407 required */
static constexpr uint8_t BL_WRITE_MEM = 0x31;
static constexpr uint8_t BL_GO        = 0x21;
static constexpr uint8_t BL_ACK       = 0x79;
static constexpr uint8_t BL_NACK      = 0x1F;
static constexpr uint32_t BL_TIMEOUT_MS = 5000;

static uint8_t *stm32BinBuf     = nullptr;
static size_t   stm32BinAlloc   = 0;   /* actual allocated size */
static size_t   stm32BinSize    = 0;   /* bytes received so far */
static bool     stm32FlashRunning = false;

static bool blWaitAck(uint32_t timeoutMs = BL_TIMEOUT_MS) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (Stm32.available()) {
            uint8_t b = Stm32.read();
            if (b == BL_ACK) return true;
            if (b == BL_NACK) { Serial.println("[BL] NACK"); return false; }
        }
        delay(1);
        esp_task_wdt_reset(); /* feed WDT — flash takes up to ~60 s */
    }
    Serial.println("[BL] Timeout waiting for ACK");
    return false;
}

static bool blSendCmd(uint8_t cmd) {
    Stm32.write(cmd);
    Stm32.write((uint8_t)(cmd ^ 0xFF)); /* checksum = XOR complement */
    return blWaitAck();
}

static bool blSendAddr(uint32_t addr) {
    uint8_t b[5];
    b[0] = (addr >> 24) & 0xFF;
    b[1] = (addr >> 16) & 0xFF;
    b[2] = (addr >>  8) & 0xFF;
    b[3] = (addr      ) & 0xFF;
    b[4] = b[0] ^ b[1] ^ b[2] ^ b[3];
    Stm32.write(b, 5);
    return blWaitAck();
}

/* Enter STM32 bootloader: BOOT0=HIGH, pulse NRST */
static void blEnter() {
    Serial.println("[BL] Entering bootloader mode");
    Stm32.end(); /* release UART briefly */
    delay(10);
    digitalWrite(STM32_BOOT0, HIGH);
    delay(10);
    /* pulse NRST: LOW ~10ms then HIGH */
    digitalWrite(STM32_NRST, LOW);
    delay(10);
    digitalWrite(STM32_NRST, HIGH);
    delay(100); /* STM32 ROM starts */
    /* Re-open at 57600 — STM32 ROM bootloader default */
    Stm32.begin(57600, SERIAL_8E1, STM32_RX, STM32_TX);
    delay(50);
    /* Send init byte 0x7F — bootloader auto-detects baud */
    Stm32.write((uint8_t)0x7F);
    blWaitAck(3000);
    Serial.println("[BL] Init sent");
}

/* Exit bootloader: BOOT0=LOW, pulse NRST, restore normal UART */
static void blExit() {
    Serial.println("[BL] Exiting bootloader, resetting STM32");
    Stm32.end();
    delay(10);
    digitalWrite(STM32_BOOT0, LOW);
    digitalWrite(STM32_NRST, LOW);
    delay(10);
    digitalWrite(STM32_NRST, HIGH);
    delay(200);
    Stm32.begin(STM32_BAUD, SERIAL_8N1, STM32_RX, STM32_TX);
    Serial.println("[BL] STM32 running application firmware");
}

/* Notify dashboard clients of STM32 flash progress */
static void blProgress(int pct, const char *msg) {
    StaticJsonDocument<128> d;
    d["type"] = "stm32_progress";
    d["pct"]  = pct;
    d["msg"]  = msg;
    String out; serializeJson(d, out);
    ws.broadcastTXT(out);
    Serial.printf("[BL] %d%% %s\n", pct, msg);
}

/* Full STM32 flash sequence */
static String doFlashSTM32(const uint8_t *bin, size_t size) {
    const uint32_t FLASH_START = 0x08000000;
    const size_t   PAGE_SIZE   = 256; /* STM32 bootloader write page */

    blEnter();
    blProgress(2, "Bootloader initialised");

    /* Mass erase */
    if (!blSendCmd(BL_ERASE)) { blExit(); return "Mass erase command failed"; }
    /* Extended erase: 0xFF 0xFF = global mass erase */
    Stm32.write((uint8_t)0xFF);
    Stm32.write((uint8_t)0xFF);
    Stm32.write((uint8_t)0x00); /* checksum 0xFF^0xFF = 0x00 */
    if (!blWaitAck(30000)) { blExit(); return "Mass erase timeout"; }
    blProgress(10, "Flash erased");

    /* Write pages */
    size_t written = 0;
    while (written < size) {
        uint32_t addr = FLASH_START + written;
        size_t chunk = (size - written < PAGE_SIZE) ? (size - written) : PAGE_SIZE;
        /* Pad to multiple of 4 */
        size_t padded = (chunk + 3) & ~3u;

        if (!blSendCmd(BL_WRITE_MEM)) { blExit(); return "Write command failed at " + String(addr, HEX); }
        if (!blSendAddr(addr))         { blExit(); return "Address failed at " + String(addr, HEX); }

        /* Send: N-1, data[0..N-1], checksum */
        uint8_t ck = (uint8_t)(padded - 1);
        Stm32.write((uint8_t)(padded - 1));
        for (size_t i = 0; i < padded; i++) {
            uint8_t b = (i < chunk) ? bin[written + i] : 0xFF;
            Stm32.write(b);
            ck ^= b;
        }
        Stm32.write(ck);
        if (!blWaitAck(5000)) { blExit(); return "Write ACK failed at " + String(addr, HEX); }

        written += chunk;
        int pct = 10 + (int)((written * 88) / size);
        if (written % 4096 < PAGE_SIZE) { /* report every ~4KB */
            blProgress(pct, ("Writing... " + String(written / 1024) + "/" + String(size / 1024) + " KB").c_str());
        }
    }
    blProgress(98, "Write complete, starting application");

    /* GO command — jump to flash start */
    if (blSendCmd(BL_GO)) {
        blSendAddr(FLASH_START);
    }

    blExit();
    blProgress(100, "STM32 flashed successfully");
    return "OK — STM32 flashed " + String(size) + " bytes";
}

/* HTTP handler: receive .bin, flash STM32 */
static void handleSTM32UpdateUpload() {
    HTTPUpload &upload = http.upload();
    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA-STM32] Start: %s  content-length=%u\n",
                      upload.filename.c_str(), upload.totalSize);
        if (stm32BinBuf) { free(stm32BinBuf); stm32BinBuf = nullptr; }
        /* Allocate: prefer Content-Length if valid, otherwise cap at 1 MB */
        stm32BinAlloc = (upload.totalSize > 0 && upload.totalSize <= 1024*1024u)
                        ? upload.totalSize : 1024*1024u;
        stm32BinBuf = (uint8_t*)malloc(stm32BinAlloc);
        if (!stm32BinBuf) {
            Serial.printf("[OTA-STM32] malloc %u bytes failed\n", stm32BinAlloc);
            stm32BinAlloc = 0;
        }
        stm32BinSize = 0;
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (stm32BinBuf) {
            /* Bounds check — never write past allocated buffer */
            if (stm32BinSize + upload.currentSize > stm32BinAlloc) {
                Serial.println("[OTA-STM32] Binary exceeds allocated buffer — aborting");
                free(stm32BinBuf); stm32BinBuf = nullptr;
                stm32BinSize = stm32BinAlloc = 0;
                return;
            }
            memcpy(stm32BinBuf + stm32BinSize, upload.buf, upload.currentSize);
            stm32BinSize += upload.currentSize;
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        Serial.printf("[OTA-STM32] Received %u bytes\n", stm32BinSize);
    }
}

static void handleSTM32Update() {
    if (!stm32BinBuf || stm32BinSize == 0) {
        http.send(400, "text/plain", "No binary received");
        return;
    }
    if (stm32FlashRunning) {
        http.send(503, "text/plain", "Flash already in progress");
        return;
    }
    stm32FlashRunning = true;
    /* Send HTTP response first so browser doesn't time out */
    String result = doFlashSTM32(stm32BinBuf, stm32BinSize);
    stm32FlashRunning = false;
    free(stm32BinBuf);
    stm32BinBuf  = nullptr;
    stm32BinSize = 0;
    if (result.startsWith("OK")) {
        http.send(200, "text/plain", result);
    } else {
        http.send(500, "text/plain", result);
    }
}

/* ═══════════════════════════════════════════════════════════════════
   SETUP / LOOP
   ═══════════════════════════════════════════════════════════════════ */
void setup() {
    Serial.begin(115200); delay(200);
    Serial.println("\n=== Civic CAN Dashboard ===");

    /* STM32 bootloader control pins */
    pinMode(STM32_BOOT0, OUTPUT); digitalWrite(STM32_BOOT0, LOW);
    pinMode(STM32_NRST,  OUTPUT); digitalWrite(STM32_NRST,  HIGH);

    Stm32.setRxBufferSize(4096);
    Stm32.begin(STM32_BAUD,SERIAL_8N1,STM32_RX,STM32_TX);
    Serial.printf("STM32 UART2: RX=GPIO%d TX=GPIO%d baud=%u\n",STM32_RX,STM32_TX,STM32_BAUD);
    Serial.printf("STM32 BOOT0: GPIO%d  NRST: GPIO%d\n",STM32_BOOT0,STM32_NRST);

    WiFi.mode(WIFI_AP);
    bool ok=WiFi.softAP(AP_SSID,AP_PASS,AP_CHANNEL,0,AP_MAX_CONN);
    Serial.printf("Wi-Fi AP '%s' %s  IP=%s\n",AP_SSID,ok?"UP":"FAILED",WiFi.softAPIP().toString().c_str());

    http.on("/",          HTTP_GET,  handleRoot);
    http.on("/update",    HTTP_POST, handleESP32Update,    handleESP32UpdateUpload);
    http.on("/update-stm32", HTTP_POST, handleSTM32Update, handleSTM32UpdateUpload);
    http.onNotFound(handleNotFound);
    http.begin();

    ws.begin(); ws.onEvent(handleWsEvent);
    Serial.println("HTTP:80  WS:81  OTA:/update  STM32-OTA:/update-stm32  ready.");
}

void loop() {
    http.handleClient(); ws.loop();
    while (Stm32.available()) {
        char c=(char)Stm32.read();
        if(c=='\n'){ if(lineLen>0){lineBuf[lineLen]='\0';parseLine(lineBuf,lineLen);} lineLen=0; }
        else if(c!='\r'){ if(lineLen<(uint16_t)(sizeof(lineBuf)-1)) lineBuf[lineLen++]=c; else lineLen=0; }
    }
    static uint32_t last=0; uint32_t now=millis();
    if(now-last>=1000){ last=now; emitStatus(); }
}
