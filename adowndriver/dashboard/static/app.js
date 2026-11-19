"use strict";
/* AdownDriver operator console - vanilla JS, no external dependencies (works offline on the travel router). */
const $ = (id) => document.getElementById(id);
const STATE_COLOR = {STANDBY:"#3b82f6",NORMAL_CRUISE:"#22c55e",WARNING:"#eab308",SPEED_REDUCTION:"#f97316",
  CONTROLLED_STOP:"#ef4444",EMERGENCY_STOP:"#dc2626",OFFLINE:"#64748b"};
const READY_TEXT = {NO_DRIVER_LINK:"No packets from Node 1 (cockpit)", DRIVER_NOT_VISIBLE:"Driver face not visible / low C_driver",
  LANE_NOT_VISIBLE:"Lane tape not visible to the road camera", PATH_BLOCKED:"Obstacle in front of the robot",
  TOF_INVALID:"Lidar sends no data", ACTIVE_FAULTS:"Faults active - press RESET",
  DRIVER_STOP_LATCHED:"Controlled stop latched - driver must be alert, then RESET"};
const SERIES = [
  {k:"ear", label:"EAR", color:"#38bdf8", min:0, max:0.45, on:true},
  {k:"ear_th", label:"TH_EAR", color:"#38bdf8", dash:[4,4], min:0, max:0.45, on:true},
  {k:"mar", label:"MAR", color:"#f59e0b", min:0, max:1.0, on:true},
  {k:"v", label:"v_cmd", color:"#22c55e", min:0, max:0.22, on:true},
  {k:"c_lane", label:"C_lane", color:"#a78bfa", min:0, max:1, on:false},
  {k:"c_drv", label:"C_driver", color:"#94a3b8", min:0, max:1, on:false},
  {k:"e", label:"e_y", color:"#f87171", min:-10, max:10, on:false},
];
let snap = null, hist = [], lastEventKey = "", toggles = {};

// ---------------------------------------------------------------- utils
function toast(msg, err=false){ const d=document.createElement("div"); d.textContent=msg; if(err) d.className="err";
  $("toast").appendChild(d); setTimeout(()=>d.remove(), 3500); }
async function api(path, opts){ const r = await fetch(path, opts); if(!r.ok){ const t=await r.text(); throw new Error(t); } return r.json(); }
async function cmd(target, c, extra={}){
  try { const r = await api("/api/command",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({target, cmd:c, ...extra})});
    if(!r.sent) toast(`Could not send ${c} to ${target}`, true); }
  catch(e){ toast(`${c}: ${e.message}`, true); }
}
const fmt = (v, d=2) => (v===null || v===undefined || Number.isNaN(v)) ? "—" : Number(v).toFixed(d);
const clamp = (x,a,b)=>Math.max(a,Math.min(b,x));
function setMeter(tile, frac, mark){ const m=tile.querySelector(".meter div"); if(m) m.style.width = `${clamp(frac,0,1)*100}%`;
  if(mark!==undefined){ const i=tile.querySelector(".meter i"); if(i) i.style.left=`${clamp(mark,0,1)*100}%`; } }

// ---------------------------------------------------------------- tabs
document.querySelectorAll("#tabs button").forEach(b => b.onclick = () => {
  document.querySelectorAll("#tabs button").forEach(x=>x.classList.toggle("active", x===b));
  document.querySelectorAll(".tab").forEach(t=>t.classList.toggle("active", t.id===`tab-${b.dataset.tab}`));
  if(b.dataset.tab==="trials") loadRuns();
  if(b.dataset.tab==="validation") loadValidation();
  if(b.dataset.tab==="settings") loadParams();
  if(window.onTabChange) window.onTabChange(b.dataset.tab);
});

// ---------------------------------------------------------------- controls
$("btnStart").onclick = () => cmd("node2","start");
$("btnStop").onclick = () => cmd("node2","stop");
$("btnReset").onclick = () => cmd("node2","reset");
$("btnEstop").onclick = $("fabEstop").onclick = () => { cmd("node2","estop"); toast("E-STOP sent"); };
document.addEventListener("keydown", e => {
  if(e.code==="Space" && !["INPUT","SELECT","TEXTAREA"].includes(document.activeElement.tagName)){
    e.preventDefault(); cmd("node2","estop"); toast("E-STOP (space)"); }
});
$("btnCalib").onclick = () => { cmd("node1","calibrate",{driver:$("driverName").value.trim()||"default"}); toast("Calibration started - look at the camera"); };
$("btnProto").onclick = () => cmd("node1","protocol_start",{name:$("protocolSel").value});
$("btnProtoStop").onclick = () => cmd("node1","protocol_stop");
$("btnAck").onclick = () => cmd("node1","ack");
$("mute").onchange = (e) => cmd("node1","mute",{value:e.target.checked});
document.querySelectorAll("#simTools [data-sim]").forEach(b => b.onclick = () => {
  let v = b.dataset.v;
  if(b.hasAttribute("data-toggle")){ b.classList.toggle("on"); v = b.classList.contains("on") ? 1 : 0; }
  cmd("node2","sim",{action:b.dataset.sim, value: v===undefined?null:Number(v)});
});
$("bright").onchange = (e) => cmd("node2","sim",{action:"brightness", value:Number(e.target.value)});
["alert","blink_fast","talk","yawn","closed","look_away","occluded","glare","dim"].forEach(b => {
  const el=document.createElement("button"); el.textContent=b.replace("_"," "); el.dataset.b=b;
  el.onclick=()=>cmd("node1","sim_behaviour",{value:b}); $("behaviours").appendChild(el); });

// legend toggles
SERIES.forEach(s => { toggles[s.k]=s.on; const l=document.createElement("label");
  l.innerHTML=`<input type="checkbox" ${s.on?"checked":""}><i style="background:${s.color}"></i>${s.label}`;
  l.querySelector("input").onchange=(e)=>{toggles[s.k]=e.target.checked;}; $("legend").appendChild(l); });

// ---------------------------------------------------------------- websocket
function connect(){
  const ws = new WebSocket(`${location.protocol==="https:"?"wss":"ws"}://${location.host}/ws`);
  ws.onopen = () => { $("pill-ws").className="pill ok"; };
  ws.onclose = () => { $("pill-ws").className="pill bad"; setTimeout(connect, 1000); };
  ws.onmessage = (ev) => { snap = JSON.parse(ev.data); try { if(window.renderDrive) window.renderDrive(snap); } catch(e){ console.error(e); } render(snap); };
}
connect();
// video streams are attached only for the visible tab (drive.js) to save Pi bandwidth
api("/api/history").then(h => { hist = h.map(r => ({...r, c_drv:r.c_drv})).concat(hist); }).catch(()=>{});

function pill(id, ok, text){ const p=$(id); p.className = "pill " + (ok===null?"":ok?"ok":"bad"); p.textContent=text; }

function render(s){
  const n1=s.node1, n2=s.node2, on=s.online;
  pill("pill-n1", on.node1, on.node1 ? `Node 1 · ${fmt(n1.fps,0)} fps` : "Node 1 offline");
  pill("pill-n2", on.node2, on.node2 ? `Node 2 · ${fmt(n2.loop_hz,0)} Hz${n2.sim?" · SIM":""}` : "Node 2 offline");
  $("ns1").style.display = on.node1 ? "none" : "grid"; $("ns2").style.display = on.node2 ? "none" : "grid";
  if(n1) $("n1fps").textContent = `${n1.backend} · ${fmt(n1.proc_ms,0)} ms`;

  // ---- Node 1 panel
  if(n1){
    const p=n1.profile||{}; $("profile").textContent = `profile "${p.name}" · TH_EAR ${fmt(p.ear_th,3)}${p.ear_open?` (open ${fmt(p.ear_open,3)})`:" (default)"}`;
    $("calibBar").firstElementChild.style.width = `${(n1.calibrating||0)*100}%`;
    $("calibMsg").textContent = n1.calib_msg || "";
    const sel=$("protocolSel"); if(sel.options.length!==(n1.protocols||[]).length){
      sel.innerHTML = (n1.protocols||[]).map(x=>`<option>${x}</option>`).join(""); }
    const cue=$("cue"); if(n1.protocol){ cue.hidden=false; cue.textContent=`Step ${n1.protocol.step}/${n1.protocol.of} · ${n1.protocol.cue} · ${fmt(n1.protocol.remaining,1)} s`; } else cue.hidden=true;
    $("simDriver").hidden = !n1.sim;
    document.querySelectorAll("#behaviours button").forEach(b=>b.classList.toggle("on", b.dataset.b===n1.sim_behaviour));
  }

  if(!n2 || !on.node2){ setState("OFFLINE","Node 2 not reachable - check power / Wi-Fi"); return; }
  setState(n2.state, n2.reason);
  pill("pill-rec", n2.run_id ? true : null, n2.run_id ? `● REC #${n2.run_id} ${n2.run?.scenario||""}` : "not recording");
  if(n2.run_id) $("pill-rec").className="pill rec";
  const L=n2.link||{};
  pill("pill-link", L.age_ms!==null && L.age_ms < 150, L.age_ms===null ? "Link down" : `Link ${fmt(L.age_ms,0)} ms · RTT ${fmt(L.rtt_ms,0)}`);

  $("vcmd").textContent = fmt(n2.v_cmd,3); $("omega").textContent = fmt(n2.omega,2); $("vtarget").textContent=fmt(n2.v_target,2);
  $("vbar").style.width = `${clamp(n2.v_cmd/0.22,0,1)*100}%`;
  $("vbar").style.background = STATE_COLOR[n2.state];
  $("capReason").textContent = n2.cap ? `speed capped: ${n2.cap}` : "";

  const f=n2.fsm||{}, P=n2.params||{};
  const ready=$("ready");
  if(n2.state==="STANDBY"){ ready.innerHTML = (f.ready_issues||[]).length ? f.ready_issues.map(r=>`<li>✗ ${READY_TEXT[r]||r}</li>`).join("")
      : `<li class="ok">✓ Ready - press START</li>`; } else ready.innerHTML="";
  if(n2.ack && n2.ack.cmd){ $("ack").textContent = `last command: ${n2.ack.cmd} ${n2.ack.ok?"✓":"✗"} ${n2.ack.info||""}`; }

  // fatigue ladder
  const tstop=P.T_STOP||4; $("ladderFill").style.width = `${clamp((f.t_closed||0)/tstop,0,1)*100}%`;
  $("lw").textContent=`WARN ${P.T_WARN}`; $("ls").textContent=`SLOW ${P.T_SLOW}`; $("lst").textContent=`STOP ${P.T_STOP}`;
  document.querySelectorAll(".ladder i")[0].style.left=`${P.T_WARN/tstop*100}%`;
  document.querySelectorAll(".ladder i")[1].style.left=`${P.T_SLOW/tstop*100}%`;
  $("tClosed").textContent = `${fmt(f.t_closed,2)} s`; $("tAlert").textContent=`${fmt(f.t_alert,1)} s`;
  $("perclos").textContent = `${fmt((f.perclos||0)*100,0)} %`; $("yawns").textContent = f.yawns_window ?? 0;
  $("tLow").textContent = `${fmt(f.t_low_conf,1)} s`; $("tDist").textContent=`${fmt(f.t_distract,1)} s`;

  // tiles
  const d=n2.driver||{};
  tile("tEar", fmt(d.ear,3), d.ear/0.45, (d.ear_th||0.21)/0.45, d.ear!==undefined && d.ear < d.ear_th);
  $("tEar").querySelector("small").textContent = `threshold ${fmt(d.ear_th,3)}`;
  tile("tMar", fmt(d.mar,2), d.mar, P.TH_MAR||0.6, d.mar > (P.TH_MAR||0.6));
  tile("tCd", fmt(d.c,2), d.c, undefined, d.c < 0.65);
  const ln=n2.lane||{};
  tile("tCl", fmt(ln.c,2), ln.c, undefined, ln.c < 0.65);
  $("laneWidth").textContent = `${ln.mode||"—"} · width ${fmt(ln.width_cm,0)} cm`;
  $("laneMode").textContent = `${fmt(ln.proc_ms,1)} ms`;
  const dd=n2.tof?.d_cm; const dt=$("tD");
  dt.querySelector("b").textContent = n2.tof?.enabled===false ? "disabled" : dd===null||dd===undefined ? "invalid" : `${fmt(dd,1)} cm`;
  setMeter(dt, (dd??0)/100); dt.classList.toggle("alarm", dd!==null && dd < (P.D_SLOW_CM||45));
  const e=ln.e_axle_cm; const et=$("tE"); et.querySelector("b").textContent = `${fmt(e,1)} cm`;
  const bar=et.querySelector(".meter div"); const frac=clamp((e||0)/20,-0.5,0.5);
  bar.style.left = frac<0 ? `${(0.5+frac)*100}%` : "50%"; bar.style.width = `${Math.abs(frac)*100}%`;
  et.classList.toggle("alarm", Math.abs(e||0)>5);
  $("tLag").querySelector("b").textContent = `${fmt(Math.max(d.proc_ms||0, n2.proc_ms||0),0)} ms`;
  $("lagDetail").textContent = `N1 ${fmt(d.proc_ms,0)} · N2 ${fmt(n2.proc_ms,1)} ms · ${n2.cpu_temp?fmt(n2.cpu_temp,0)+" °C":""}`;
  $("tLink").querySelector("b").textContent = L.age_ms===null ? "lost" : `${fmt(L.age_ms,0)} ms`;
  $("linkDetail").textContent = `RTT ${fmt(L.rtt_ms,1)} ms · timeout 150`;
  $("tLink").classList.toggle("alarm", L.age_ms===null || L.age_ms>150);

  $("simTools").hidden = !n2.sim;

  // events
  const evs=n2.events||[]; const key=evs.length?evs[evs.length-1].t_ms+evs[evs.length-1].kind:"";
  if(key!==lastEventKey){ lastEventKey=key;
    $("events").innerHTML = evs.slice().reverse().map(e=>`<li class="k-${e.kind}"><time>${new Date(e.t_ms).toLocaleTimeString()}</time><b>${e.kind}</b><span>${e.detail}</span></li>`).join(""); }

  // chart buffer
  hist.push({t:n2.t_ms, ear:d.ear, ear_th:d.ear_th, mar:d.mar, v:n2.v_cmd, c_lane:ln.c, c_drv:d.c, e:ln.e_axle_cm, state:n2.state});
  const cutoff = n2.t_ms - 60000; while(hist.length && hist[0].t < cutoff) hist.shift();
}

function tile(id, text, frac, mark, alarm){ const t=$(id); t.querySelector("b").textContent=text; setMeter(t, frac||0, mark); t.classList.toggle("alarm", !!alarm); }
function setState(st, reason){
  const c=STATE_COLOR[st]||"#64748b"; $("stateName").textContent = st.replace(/_/g," "); $("stateName").style.color=c;
  $("stateCard").style.borderColor=c; $("stateReason").textContent = reason || "";
}

// ---------------------------------------------------------------- chart
function drawChart(canvas, rows, series, tKey="t", span=null, states=true){
  const dpr=window.devicePixelRatio||1, W=canvas.clientWidth, H=canvas.height;
  if(canvas.width!==W*dpr){ canvas.width=W*dpr; canvas.style.height=H+"px"; }
  const g=canvas.getContext("2d"); g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,H*1);
  const Hh = canvas.height/dpr;
  if(!rows.length){ g.fillStyle="#8b9bab"; g.fillText("waiting for data…", 10, 20); return; }
  const t1=rows[rows.length-1][tKey], t0= span ? t1-span : rows[0][tKey];
  const x=(t)=> (t-t0)/(t1-t0||1)*(W-8)+4;
  if(states){ // state band
    for(let i=1;i<rows.length;i++){ g.fillStyle=(STATE_COLOR[rows[i].state]||"#333")+"55"; g.fillRect(x(rows[i-1][tKey]), Hh-8, x(rows[i][tKey])-x(rows[i-1][tKey])+1, 8); } }
  g.strokeStyle="#243241"; g.lineWidth=1; for(let k=1;k<4;k++){ g.beginPath(); g.moveTo(0,(Hh-10)*k/4); g.lineTo(W,(Hh-10)*k/4); g.stroke(); }
  for(const s of series){ if(toggles[s.k]===false && canvas.id==="chart") continue;
    g.strokeStyle=s.color; g.lineWidth=1.6; g.setLineDash(s.dash||[]); g.beginPath(); let started=false;
    for(const r of rows){ const v=r[s.k]; if(v===null||v===undefined){started=false;continue;}
      const y=(Hh-12)-(clamp((v-s.min)/(s.max-s.min),0,1))*(Hh-16);
      if(!started){g.moveTo(x(r[tKey]),y);started=true;} else g.lineTo(x(r[tKey]),y); }
    g.stroke(); g.setLineDash([]); }
}
(function loop(){ drawChart($("chart"), hist, SERIES, "t", 60000); requestAnimationFrame(()=>setTimeout(loop, 100)); })();

// ---------------------------------------------------------------- trials
let scenarios=[];
async function loadScenarios(){
  scenarios = await api("/api/scenarios");
  $("trialScenario").innerHTML = `<option value="FREE_DRIVE">FREE_DRIVE — unscripted</option>` +
    scenarios.map(s=>`<option value="${s.id}">${s.id} — ${s.name}</option>`).join("");
  $("trialScenario").onchange = () => { const s=scenarios.find(x=>x.id===$("trialScenario").value);
    $("trialProc").textContent = s ? `Procedure: ${s.procedure}  Expected: ${s.expect_reach.join(" → ")||"no escalation"}; ends in ${s.expect_final}.` : ""; };
}
loadScenarios().catch(()=>{});
$("btnRunStart").onclick = () => { cmd("node2","run_start",{scenario:$("trialScenario").value, driver:$("trialDriver").value,
  operator:$("trialOperator").value, notes:$("trialNotes").value}); toast("Recording started"); };
$("btnRunStop").onclick = () => { cmd("node2","run_stop"); toast("Run saved - scoring…"); setTimeout(loadRuns, 1200); };
$("btnRefreshRuns").onclick = loadRuns;

const badge = (p) => p===true?'<span class="badge pass">PASS</span>':p===false?'<span class="badge fail">FAIL</span>':'<span class="badge na">n/a</span>';
async function loadRuns(){
  const runs = await api("/api/runs");
  $("runsBody").innerHTML = runs.map(r=>{ const m=(r.summary||{}).metrics||{}; const ps=(r.summary||{}).pass||{};
    const dur = r.ended_ms ? ((r.ended_ms-r.started_ms)/1000).toFixed(0)+" s" : "running";
    const cell=(k,d)=> m[k]===undefined||m[k]===null ? "—" : `${fmt(m[k],d)}`;
    return `<tr class="click" data-id="${r.id}"><td>${r.id}</td><td>${new Date(r.started_ms).toLocaleString()}</td><td>${r.scenario}</td>
      <td>${r.source}</td><td>${dur}</td><td>${cell("f1",2)}</td><td>${cell("blink_false_alarms",0)}</td><td>${cell("lag_p95_ms",0)}</td>
      <td>${cell("lane_dev_stop_cm",1)}</td><td>${cell("brake_latency_ms",0)}</td>
      <td><a href="/api/runs/${r.id}/csv" onclick="event.stopPropagation()">CSV</a></td></tr>`; }).join("") ||
    `<tr><td colspan="11" class="muted">No runs yet.</td></tr>`;
  document.querySelectorAll("#runsBody tr.click").forEach(tr=>tr.onclick=()=>showRun(tr.dataset.id));
}
const MNAMES={f1:"Drowsiness F1",blink_false_alarms:"Blink false alarms",lag_p95_ms:"Lag p95 (ms)",lane_dev_stop_cm:"Lane dev. in stop (cm)",brake_latency_ms:"Brake latency (ms)"};
function metricCards(el, metrics){
  el.innerHTML = Object.entries(metrics).filter(([k])=>!k.startsWith("_")).map(([k,v])=>
    `<div class="${v.pass===true?"pass":v.pass===false?"fail":""}"><span class="small muted">${MNAMES[k]||k}</span><b>${v.value===null?"n/a":v.value}</b><span class="small muted">target ${v.target}</span></div>`).join("");
}
async function showRun(id){
  const d = await api(`/api/runs/${id}`);
  $("runDetail").hidden=false;
  $("runTitle").textContent = `Run #${id} · ${d.run.scenario} · ${d.run.source} · ${d.metrics.n_ticks} ticks · ${d.metrics.n_events} events`;
  metricCards($("runMetrics"), d.metrics.metrics);
  const rows = d.ticks.map(t=>({t:t.t_ms, ear:t.ear, ear_th:t.ear_th, mar:t.mar, v:t.v_cmd, c_lane:t.c_lane, state:t.state}));
  drawChart($("runChart"), rows, SERIES.slice(0,5), "t");
}

// ---------------------------------------------------------------- validation
$("btnValidate").onclick = async () => { const r = await api("/api/validate",{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"});
  toast(r.started ? "Validation started (~2 min)" : r.reason); pollValidation(); };
async function pollValidation(){ const v = await loadValidation(); if(v.running) setTimeout(pollValidation, 2000); }
async function loadValidation(){
  const v = await api("/api/validation");
  $("valStatus").textContent = v.running ? "Running… results appear when finished." : v.report ? `Last run ${v.report.generated}: ${v.report.passed}/${v.report.total} scenarios passed.` : "No validation run yet.";
  if(v.report){ metricCards($("valMetrics"), v.report.aggregate);
    $("valBody").innerHTML = v.report.scenarios.map(s=>`<tr><td>${s.id}</td><td>${s.name}</td><td class="small">${s.states.join(" → ")}</td>
      <td>${s.lane_dev_true_cm??"—"}</td><td>${badge(s.passed)}</td>
      <td><a href="/api/validation/plot/${s.id}.png" target="_blank"><img class="thumb" src="/api/validation/plot/${s.id}.png" onerror="this.remove()"></a></td></tr>`).join(""); }
  return v;
}

// ---------------------------------------------------------------- settings
async function loadParams(){
  const {config, tunable} = await api("/api/config");
  const box=$("params"); box.innerHTML="";
  for(const [sec, names] of Object.entries(tunable)){
    const h=document.createElement("h4"); h.textContent=sec; box.appendChild(h);
    for(const n of names){ const l=document.createElement("label"); const val=config[sec][n];
      l.innerHTML=`${n}<input type="number" step="any" value="${val}">`;
      l.querySelector("input").onchange=(e)=>{ cmd("node2","set_param",{path:`${sec}.${n}`, value:Number(e.target.value)}); toast(`${sec}.${n} = ${e.target.value}`); };
      box.appendChild(l); }
  }
}
