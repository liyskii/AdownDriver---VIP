"use strict";
/* Drive tab: the simple operator view. Reads the same WebSocket snapshot as the engineering view (app.js). */
(function(){
const q = (id) => document.getElementById(id);
const ORDER = ["STANDBY","NORMAL_CRUISE","WARNING","SPEED_REDUCTION","CONTROLLED_STOP","EMERGENCY_STOP"];
const TITLE = {STANDBY:"Ready", NORMAL_CRUISE:"Driving", WARNING:"Warning", SPEED_REDUCTION:"Slowing down",
  CONTROLLED_STOP:"Stopping safely", EMERGENCY_STOP:"Emergency stop", OFFLINE:"Robot offline"};
const REASON = {
  NONE:"", EYE_CLOSURE:"The driver's eyes are closing.", YAWN:"The driver is yawning.",
  HIGH_PERCLOS:"The driver's eyes are closed too often.", DRIVER_DISTRACTED:"The driver is looking away from the road.",
  SUSTAINED_EYE_CLOSURE:"The driver's eyes stayed closed, so the robot is slowing down.",
  REPEATED_YAWNING:"The driver keeps yawning, so the robot is slowing down.",
  LOW_DRIVER_CONFIDENCE:"The driver's face is hard to see, so the robot is slowing down.",
  PERSISTENT_DROWSINESS:"The driver fell asleep. The robot is coming to a gentle stop.",
  DRIVER_NOT_MONITORABLE:"The driver can't be seen. The robot is coming to a gentle stop.",
  OBSTACLE:"Something is in front of the robot.", HEARTBEAT_LOSS:"The connection to the cockpit was lost.",
  TOF_FAULT:"The lidar stopped sending data.", LANE_LOST:"The road camera lost the lane.",
  MANUAL_ESTOP:"Emergency stop was pressed.",
};
const IDLE_SUB = {
  STANDBY:"The robot is parked. Calibrate the driver, then start driving.",
  NORMAL_CRUISE:"The driver is alert. Close your eyes to see the safe stop.",
  CONTROLLED_STOP:"Open your eyes, stay alert for a moment, then press Reset.",
  EMERGENCY_STOP:"Clear the problem, then press Reset.",
};
const EVENT_TEXT = (k, d) => {
  if(k==="TRANSITION"){ const [ft, why] = String(d).split(" "); const to=(ft||"").split("->")[1];
    const r = REASON[why] ? " " + REASON[why] : "";
    return to ? `${TITLE[to]||to}.${r}` : d; }
  if(k==="RESET_REJECTED") return d==="DRIVER_NOT_ALERT" ? "Reset refused: open your eyes and look at the camera for 2 s, then press Reset again."
                                 : `Reset refused: ${String(d).toLowerCase().replace(/_/g," ")} is still active.`;
  if(k==="START_REJECTED") return "Start refused: " + String(d).split(",").map(x=>x.toLowerCase().replace(/_/g," ")).join(", ") + ".";
  return {YAWN:"Yawn detected.", RECOVERED:"Driver is alert again.", CALIBRATED:"Driver calibrated.",
          BRAKE:"Robot braked.", COMMAND:`Command: ${d}`}[k] || `${k.toLowerCase().replace(/_/g," ")} ${d||""}`;
};

let calibrated = false, lastKey = "";
const send = (t, c, x={}) => fetch("/api/command",{method:"POST",headers:{"Content-Type":"application/json"},
  body:JSON.stringify({target:t, cmd:c, ...x})}).catch(()=>{});

q("dvCalib").onclick = () => { send("node1","calibrate",{driver:(q("driverName")?.value||"default").trim()||"default"}); };
q("dvStart").onclick = () => send("node2","start");
q("dvStop").onclick  = () => send("node2","stop");
q("dvReset").onclick = () => send("node2","reset");
q("dvEstop").onclick = () => send("node2","estop");
// volume: label follows the slider live, the cockpit gets the value when you let go
let volDragging = false;
q("dvVol").oninput  = (e) => { volDragging = true; q("dvVolVal").textContent = e.target.value + "%"; };
q("dvVol").onchange = (e) => { volDragging = false; send("node1","volume",{value: Number(e.target.value)/100}); };

// --- video: only the visible tab streams (each MJPEG stream costs CPU on the Pi)
const VIDS = {drive:[["dvVid1","/video/1.mjpg"],["dvVid2","/video/2.mjpg"]], live:[["vid1","/video/1.mjpg"],["vid2","/video/2.mjpg"]]};
window.onTabChange = (tab) => {
  for(const [name, list] of Object.entries(VIDS)) for(const [id, url] of list){
    const el = q(id); if(!el) continue;
    if(name===tab){ if(!el.getAttribute("src")) el.src = url + "?t=" + Date.now(); } else el.removeAttribute("src");
  }
  document.body.classList.toggle("drive-mode", tab==="drive");
};
window.onTabChange("drive");

const fmt = (v,d=1) => (v===null||v===undefined||Number.isNaN(v)) ? "—" : Number(v).toFixed(d);
const pct = (x) => `${Math.max(0,Math.min(1,x))*100}%`;
function mark(id, ok, pending){ const el=q(id); el.className = ok ? "ok" : pending ? "wait" : "no"; }

window.renderDrive = (s) => {
  const on = s.online||{}, n1 = s.node1, n2 = s.node2;
  const vol = q("dvVol");
  vol.disabled = !on.node1 || n1?.volume === null || n1?.volume === undefined;
  if (!volDragging && n1?.volume !== null && n1?.volume !== undefined && document.activeElement !== vol) {
    vol.value = Math.round(n1.volume*100); q("dvVolVal").textContent = vol.value + "%";
  }
  if (vol.disabled) q("dvVolVal").textContent = on.node1 ? "no speaker" : "offline";
  const st = (n2 && on.node2) ? n2.state : "OFFLINE";
  document.body.dataset.st = st;

  // stage road
  const idx = ORDER.indexOf(st);
  q("dvStages").querySelectorAll("li").forEach((li,i)=>{
    li.classList.toggle("now", li.dataset.st===st);
    li.classList.toggle("past", idx>0 && i>0 && i<idx && st!=="EMERGENCY_STOP"); });
  q("dvTitle").textContent = TITLE[st] || st;
  const why = n2 && REASON[n2.reason];
  q("dvSub").textContent = st==="OFFLINE" ? "Turn the robot on and connect it to the adown-net Wi-Fi."
    : (why || IDLE_SUB[st] || "");

  // cameras
  q("dvNs1").hidden = !!on.node1; q("dvNs2").hidden = !!on.node2;

  // checklist
  const d = n2?.driver || {}, f = n2?.fsm || {}, P = n2?.params || {};
  const prof = n1?.profile || {};
  calibrated = calibrated || !!prof.ear_open;
  mark("ckN1", !!on.node1); mark("ckN2", !!on.node2);
  mark("ckFace", on.node2 && d.c >= 0.65);
  mark("ckCal", calibrated, true);
  const dist = n2?.tof?.d_cm;
  mark("ckPath", on.node2 && dist !== null && dist !== undefined && dist >= (P.D_SAFE_CM||20));
  q("dvCheck").hidden = st !== "STANDBY" && st !== "OFFLINE";

  // calibration progress
  const cal = n1?.calibrating || 0; q("dvCalBar").hidden = !(cal > 0 && cal < 1);
  q("dvCalBar").firstElementChild.style.width = pct(cal);
  q("dvCalibMsg").textContent = n1?.calib_msg || (calibrated ? `Done for "${prof.name||"default"}". Repeat for a new driver.` : "Look at the camera with eyes open for 5 s");

  // eyes gauge
  const tStop = P.T_STOP || 4, tc = f.t_closed || 0;
  const faceOk = d.c >= 0.65, closed = faceOk && d.ear !== undefined && d.ear < d.ear_th;
  q("dvEyesVal").textContent = !on.node2 ? "—" : !faceOk ? "Not visible" : closed ? "Closed" : "Open";
  q("dvEyes").dataset.v = !faceOk ? "warn" : closed ? "bad" : "good";
  q("dvEyesFill").style.width = pct(tc / tStop);
  q("dvMkW").style.left = pct((P.T_WARN||1.2)/tStop); q("dvMkS").style.left = pct((P.T_SLOW||2.5)/tStop);
  q("dvEyesNote").textContent = `Closed for ${fmt(tc,1)} s · stops at ${fmt(tStop,1)} s`;

  // obstacle gauge
  const okDist = dist !== null && dist !== undefined;
  q("dvObsVal").textContent = n2?.tof?.enabled===false ? "Off" : okDist ? (dist >= 299 ? "Clear" : `${fmt(dist,0)} cm`) : "Lidar: no data";
  q("dvObs").dataset.v = !okDist ? "warn" : dist < (P.D_SAFE_CM||20) ? "bad" : dist < (P.D_SLOW_CM||45) ? "warn" : "good";
  q("dvObsFill").style.width = pct(okDist ? dist/100 : 0);

  // speed gauge (cm/s reads better than 0.16 m/s)
  const v = n2?.v_cmd || 0;
  q("dvSpdVal").textContent = on.node2 ? `${fmt(v*100,0)} cm/s` : "—";
  q("dvSpdFill").style.width = pct(v/0.22);
  const L = n2?.link || {};
  q("dvLink").textContent = !on.node1 ? "Cockpit not connected" : (L.age_ms===null||L.age_ms===undefined) ? "Cockpit link lost"
    : `Cockpit link ${fmt(L.age_ms,0)} ms`;

  // buttons + the reason why START is refused (the FSM checks these before it moves)
  const WHY = {NO_DRIVER_LINK:"Cockpit is not connected", DRIVER_NOT_VISIBLE:"Driver's face is not visible",
    LANE_NOT_VISIBLE:"Road camera does not see the lane", PATH_BLOCKED:"Something is in front of the robot",
    TOF_INVALID:"Lidar sends no data (robot base not running or battery low)", ACTIVE_FAULTS:"an earlier stop is still active",
    DRIVER_STOP_LATCHED:"driver looks drowsy or is not in front of the camera"};
  const issues = (st==="STANDBY") ? (f.ready_issues||[]) : [];
  const startNote = q("dvStart").querySelector("small");
  if(!on.node2) startNote.textContent = "Robot is not connected";
  else if(st==="CONTROLLED_STOP" || st==="EMERGENCY_STOP") startNote.textContent = "Press Reset first";
  else if(st!=="STANDBY") startNote.textContent = "Already driving";
  else if(issues.length) startNote.textContent = "Can't start: " + issues.map(i=>WHY[i]||i).join(", ") +
    (issues.some(i => i === "ACTIVE_FAULTS") ? " (press Reset)" : "");
  else startNote.textContent = "Robot drives at 0.16 m/s";
  q("dvStart").classList.toggle("blocked", issues.length>0);
  q("dvStart").disabled = !on.node2 || st !== "STANDBY";
  // Reset is needed after a stop, and also in STANDBY when a latched fault or a drowsiness stop blocks START
  const needsReset = ["CONTROLLED_STOP","EMERGENCY_STOP"].includes(st) ||
    (st === "STANDBY" && issues.some(i => i === "ACTIVE_FAULTS"));
  q("dvReset").disabled = !on.node2 || !needsReset;
  q("dvReset").classList.toggle("go", !!on.node2 && st === "STANDBY" && needsReset);
  q("dvStop").disabled  = !on.node2 || st === "STANDBY";
  q("dvCalib").disabled = !on.node1;

  // plain-language log
  const evs = (n2?.events || []).filter(e => ["TRANSITION","YAWN","RECOVERED","CALIBRATED","RESET_REJECTED","START_REJECTED"].includes(e.kind));
  const key = evs.length ? evs[evs.length-1].t_ms + evs[evs.length-1].kind : "";
  if(key && key !== lastKey){ lastKey = key;
    q("dvLog").innerHTML = evs.slice(-6).reverse().map(e => {
      const to = e.kind==="TRANSITION" ? (String(e.detail).split(" ")[0].split("->")[1]||"") : e.kind;
      return `<li data-st="${to}"><time>${new Date(e.t_ms).toLocaleTimeString([], {hour:"2-digit",minute:"2-digit",second:"2-digit"})}</time><span>${EVENT_TEXT(e.kind, e.detail)}</span></li>`; }).join(""); }
};
})();
