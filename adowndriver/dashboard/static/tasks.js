"use strict";
/* Task launcher tab - pick any task / sub-task of the project and run it here. */
(() => {
const el = (id) => document.getElementById(id);
const esc = (s) => String(s ?? "").replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const KIND = {process:["job",""], parallel:["services","svc"], sequence:["pipeline","seq"], command:["node command","cmd"]};
const WHERE = {laptop:"laptop / any PC", node1:"Node 1 Pi", node2:"TurtleBot Pi", any:"any machine"};
/* The mission flow: the order in which the team actually uses the system. */
const FLOW = [
  {t:"Check hardware", d:"self-tests of both Pis", ids:["T6.1","T6.2"]},
  {t:"Calibrate", d:"road camera + driver EAR", ids:["T2.3","T1.3"]},
  {t:"Validate in SIL", d:"tests → 16 scenarios", ids:["T8.3","T8.1","T3.2"]},
  {t:"Drive", d:"demo or real robot", ids:["T5.1","T5.2","T1.1","T2.1"]},
  {t:"Score & report", d:"metrics, CSV, evidence", ids:["T4.2","T9.1"]},
];
let TASKS = [], JOBS = [], selJob = null, logSeq = 0, flowOn = null;

async function api(path, opts){ const r = await fetch(path, opts); const t = await r.text();
  let j; try { j = JSON.parse(t); } catch { j = t; }
  if(!r.ok) throw new Error((j && j.detail) || t); return j; }
function note(msg, err){ if(typeof toast === "function") toast(msg, err); }

// ------------------------------------------------------------------ render catalogue
function paramHtml(p){
  if(p.kind === "select")
    return `<label>${esc(p.label)}<select data-p="${p.name}">${p.options.map(o =>
      `<option value="${esc(o.value)}" ${String(o.value)===String(p.default)?"selected":""}>${esc(o.label)}</option>`).join("")}</select></label>`;
  const type = p.kind === "int" ? `type="number" min="${p.lo}" max="${p.hi}"` : `type="text" maxlength="32"`;
  return `<label>${esc(p.label)}<input ${type} data-p="${p.name}" value="${esc(p.default)}"></label>`;
}
function subHtml(s){
  const [kname, kcls] = KIND[s.kind] || ["job",""];
  const tags = [`<span class="tag ${s.service?"svc":kcls}">${s.service?"service":kname}</span>`,
    `<span class="tag">${WHERE[s.where]||s.where}</span>`,
    s.missing.length ? `<span class="tag miss" title="pip install ${esc(s.missing.join(" "))}">missing ${esc(s.missing.join(", "))}</span>`
                     : `<span class="tag ok">ready here</span>`,
    s.est_s ? `<span class="tag">≈ ${s.est_s < 60 ? s.est_s+" s" : Math.round(s.est_s/60)+" min"}</span>` : "",
    `<span class="tag run" data-running="${s.id}" hidden>running</span>`].join("");
  const label = s.kind === "command" ? "Send" : s.service ? "Start" : "Run";
  return `<div class="sub" data-sub="${s.id}" data-text="${esc((s.id+" "+s.title+" "+s.description+" "+s.preview.join(" ")).toLowerCase())}">
    <span class="sid">${s.id}</span>
    <div class="st"><b>${esc(s.title)}</b><p>${esc(s.description)}</p><div class="meta">${tags}</div></div>
    <button class="btn run ${s.service?"svc":""} go" data-run="${s.id}" ${s.missing.length?"disabled":""}
      title="${esc(s.preview.join("  &&  ") || "send command to the running node")}">${label}</button>
    ${s.params.length ? `<div class="params">${s.params.map(paramHtml).join("")}</div>` : ""}
  </div>`;
}
function renderCatalogue(){
  el("taskList").innerHTML = TASKS.map(t => `
    <article class="task" data-task="${t.id}" data-text="${esc((t.id+" "+t.title+" "+t.goal+" "+t.module).toLowerCase())}">
      <header><span class="tid">${t.id}</span><h3>${esc(t.title)}</h3>
        </header>
      <div class="goal">${esc(t.goal)} <code>${esc(t.module)}</code></div>
      <div class="subs">${t.subtasks.map(subHtml).join("")}</div>
    </article>`).join("");
  el("taskList").querySelectorAll(".task>header").forEach(h => h.onclick = () => h.parentElement.classList.toggle("collapsed"));
  el("taskList").querySelectorAll("[data-run]").forEach(b => b.onclick = () => run(b.dataset.run));
  el("flow").innerHTML = FLOW.map((f,i) => `<li data-i="${i}"><b>${f.t}</b><small>${f.d} · ${f.ids.join(", ")}</small></li>`).join("");
  el("flow").querySelectorAll("li").forEach(li => li.onclick = () => {
    const i = Number(li.dataset.i); flowOn = flowOn === i ? null : i;
    el("taskSearch").value = ""; filter(); });
}

// ------------------------------------------------------------------ search / flow filter
function filter(){
  const q = el("taskSearch").value.trim().toLowerCase();
  const ids = flowOn === null ? null : FLOW[flowOn].ids;
  el("flow").querySelectorAll("li").forEach(li => li.classList.toggle("on", Number(li.dataset.i) === flowOn));
  let first = null;
  el("taskList").querySelectorAll(".task").forEach(t => {
    const taskHit = q && t.dataset.text.includes(q);
    let any = false;
    t.querySelectorAll(".sub").forEach(s => {
      const hit = ids ? ids.includes(s.dataset.sub) : (!q || taskHit || s.dataset.text.includes(q));
      s.hidden = !hit; s.classList.toggle("hit", !!(hit && (q || ids)));
      if(hit){ any = true; first = first || s; }
    });
    t.hidden = !any;
    if(q || ids) t.classList.remove("collapsed");
  });
  if((q || ids) && first) first.scrollIntoView({block:"nearest", behavior:"smooth"});
}
el("taskSearch").addEventListener("input", () => { flowOn = null; filter(); });
el("taskSearch").addEventListener("keydown", e => {
  if(e.key === "Enter"){ const s = [...document.querySelectorAll(".sub")].find(x => !x.hidden);
    if(s && !s.querySelector("[data-run]").disabled) run(s.dataset.sub); }
  if(e.key === "Escape"){ e.target.value = ""; flowOn = null; filter(); e.target.blur(); }
});
document.addEventListener("keydown", e => {
  if(e.key === "/" && !["INPUT","SELECT","TEXTAREA"].includes(document.activeElement.tagName)){
    e.preventDefault(); showTab("tasks"); el("taskSearch").focus(); }
});

// ------------------------------------------------------------------ run / stop
async function run(id){
  const row = document.querySelector(`.sub[data-sub="${id}"]`);
  const params = {}; row.querySelectorAll("[data-p]").forEach(i => params[i.dataset.p] = i.value);
  try {
    const j = await api("/api/tasks/run", {method:"POST", headers:{"Content-Type":"application/json"},
      body: JSON.stringify({id, params})});
    note(`${id} ${j.status === "ok" ? "sent" : "started"} (job #${j.id})`);
    await refreshJobs(); select(j.id);
  } catch(e){ note(`${id}: ${e.message}`, true); }
}
el("btnJobStop").onclick = async () => { if(selJob){ await api(`/api/jobs/${selJob}/stop`, {method:"POST"}); refreshJobs(); } };
el("btnGoLive").onclick = () => showTab("live");

// ------------------------------------------------------------------ jobs list + console
const fmtT = (s) => s < 60 ? `${s.toFixed(0)} s` : `${Math.floor(s/60)} min ${Math.round(s%60)} s`;
async function refreshJobs(){
  try { JOBS = await api("/api/jobs"); } catch { return; }
  const running = JOBS.filter(j => j.status === "running" || j.status === "starting");
  const pj = el("pill-jobs"); pj.textContent = `Jobs ${running.length}`; pj.className = "pill" + (running.length ? " ok" : "");
  el("jobsCount").textContent = JOBS.length ? `${running.length} running · ${JOBS.length} total` : "";
  document.querySelectorAll("[data-running]").forEach(t => t.hidden = !running.some(j => j.sub_id === t.dataset.running));
  el("jobList").innerHTML = JOBS.length ? JOBS.map(j => `<li data-job="${j.id}" class="${j.id===selJob?"sel":""}">
      <span class="dot ${j.status}"></span>
      <div><b>${j.sub_id}</b> ${esc(j.title)}<small>#${j.id} · ${j.status}${j.rc!==null && j.status!=="ok" ? " (exit "+j.rc+")" : ""} · ${fmtT(j.elapsed_s)}</small></div>
      <small>${new Date(j.started).toLocaleTimeString()}</small></li>`).join("")
    : `<li class="muted small">Nothing has been run yet.</li>`;
  el("jobList").querySelectorAll("[data-job]").forEach(li => li.onclick = () => select(Number(li.dataset.job)));
}
function lineHtml(l){
  let cls = l.src ? `l-${l.src}` : "";
  if(/\[PASS\]|passed|=== OK/.test(l.text)) cls += " pass";
  if(/\[FAIL\]|FAILED|Error|Traceback|=== FAILED/.test(l.text)) cls += " fail";
  return `<span class="${cls}">${l.src && l.src.startsWith("p") ? `[${l.src}] ` : ""}${esc(l.text)}</span>\n`;
}
function select(id){ selJob = id; logSeq = 0; el("console").innerHTML = ""; el("artifacts").innerHTML = "";
  el("consoleCard").hidden = false; pollLog(); refreshJobs(); }
async function pollLog(){
  if(!selJob) return;
  let j; try { j = await api(`/api/jobs/${selJob}?since=${logSeq}`); } catch { return; }
  const c = el("console"), atBottom = c.scrollTop + c.clientHeight >= c.scrollHeight - 30;
  c.insertAdjacentHTML("beforeend", j.log.map(lineHtml).join(""));
  if(j.log.length) logSeq = j.log[j.log.length-1].n;
  if(atBottom) c.scrollTop = c.scrollHeight;
  el("consoleTitle").textContent = `#${j.id} ${j.sub_id} · ${j.title} — ${j.status}`;
  el("consoleCmd").textContent = j.commands.map(x => "python " + x).join("  &&  ");
  const live = j.status === "running" || j.status === "starting";
  el("btnJobStop").hidden = !live;
  el("btnGoLive").hidden = !(live && j.service && /node|demo|T5|T1\.[12]|T2\.[12]/.test(j.sub_id + j.commands.join(" ")));
  el("artifacts").innerHTML = (j.artifacts || []).map(a => {
    const u = `/api/artifact?path=${encodeURIComponent(a)}&t=${Date.now()}`;
    return /\.png$/i.test(a) ? `<a href="${u}" target="_blank"><img src="${u}" alt="">${esc(a)}</a>` : `<a href="${u}" target="_blank">📄 ${esc(a)}</a>`;
  }).join("");
}

// ------------------------------------------------------------------ tabs / boot
function showTab(name){ const b = document.querySelector(`#tabs button[data-tab="${name}"]`); if(b) b.click(); }
document.querySelectorAll("#tabs button").forEach(b => b.addEventListener("click", () => {
  history.replaceState(null, "", "#" + b.dataset.tab); }));
async function boot(){
  const d = await api("/api/tasks"); TASKS = d.tasks;
  const nSub = TASKS.reduce((a,t) => a + t.subtasks.length, 0);
  const ready = TASKS.reduce((a,t) => a + t.subtasks.filter(s => !s.missing.length).length, 0);
  el("machine").innerHTML = `<b>${TASKS.length}</b> tasks · <b>${nSub}</b> sub-tasks<br><b>${ready}</b> runnable on this machine
    <br><span class="muted">host ${esc(location.host)}</span>`;
  renderCatalogue(); refreshJobs();
  const h = location.hash.slice(1); if(h && h !== "tasks") showTab(h);
}
boot().catch(e => note("task catalogue: " + e.message, true));
setInterval(refreshJobs, 1500);
setInterval(() => { const j = JOBS.find(x => x.id === selJob);
  if(selJob && (!j || j.status === "running" || j.status === "starting" || logSeq < j.lines)) pollLog(); }, 700);
})();
