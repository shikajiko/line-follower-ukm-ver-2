#include "web_server.h"
#include "mission.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <string.h>
#include "display.h"

static WebServer webServer(80);
static bool web_server_started = false;
static bool is_mission_loaded = false;

#define MISSION_PREF_NAMESPACE "missions"
#define MISSION_PREF_KEY "data"
#define MISSION_PREF_COUNT_KEY "count"

static size_t packMissions(uint8_t *out, size_t maxBytes) {
    size_t needed = (size_t)NUM_STATES * MISSION_RECORD_SIZE;
    if (needed > maxBytes) return 0;

    for (int i = 0; i < NUM_STATES; i++) {
        uint8_t *p = out + i * MISSION_RECORD_SIZE;
        const MissionState &m = missionStates[i];
        p[0] = (uint8_t)m.lineMode;
        p[1] = (uint8_t)m.driveMode;
        memcpy(p + 2, &m.leftSpeed, 4);
        memcpy(p + 6, &m.rightSpeed, 4);
        p[10] = (uint8_t)m.condition;
        memcpy(p + 11, &m.condition_threshold, 4);
        p[15] = m.sensorRight;
        p[16] = m.sensorLeft;
        p[17] = (uint8_t)m.maskMode;
        p[18] = (uint8_t)m.stopMode;
    }
    return needed;
}

static bool unpackMissions(const uint8_t *in, size_t len) {
    if (len == 0 || len % MISSION_RECORD_SIZE != 0) return false;

    int count = len / MISSION_RECORD_SIZE;
    if (count > MAX_MISSIONS) count = MAX_MISSIONS;

    for (int i = 0; i < count; i++) {
        const uint8_t *p = in + i * MISSION_RECORD_SIZE;
        MissionState &m = missionStates[i];
        m.lineMode = (LineMode)p[0];
        m.driveMode = (DriveMode)p[1];
        memcpy(&m.leftSpeed, p + 2, 4);
        memcpy(&m.rightSpeed, p + 6, 4);
        m.condition = (ConditionType)p[10];
        memcpy(&m.condition_threshold, p + 11, 4);
        m.sensorRight = p[15];
        m.sensorLeft = p[16];
        m.maskMode = (MaskMode)p[17];
        m.stopMode = (StopMode)p[18];
    }
    NUM_STATES = count;
    return true;
}

static void persistMissionsToNVS() {
    uint8_t buf[MAX_MISSIONS * MISSION_RECORD_SIZE];
    size_t n = packMissions(buf, sizeof(buf));
    if (n == 0 && NUM_STATES > 0) {
        Serial.println("[WEB] persistMissionsToNVS: pack failed, not writing NVS");
        return;
    }

    Preferences prefs;
    prefs.begin(MISSION_PREF_NAMESPACE, false);
    prefs.putBytes(MISSION_PREF_KEY, buf, n);
    prefs.putInt(MISSION_PREF_COUNT_KEY, NUM_STATES);
    prefs.end();

    Serial.printf("[WEB] Persisted %d mission(s) (%u bytes) to NVS\n", NUM_STATES, (unsigned)n);
}

void loadMissionsFromNVS() {
    Preferences prefs;
    prefs.begin(MISSION_PREF_NAMESPACE, true);
    int count = prefs.getInt(MISSION_PREF_COUNT_KEY, 0);
    if (count > MAX_MISSIONS) count = MAX_MISSIONS;

    if (count <= 0) {
        prefs.end();
        Serial.println("[WEB] No saved missions in NVS, keeping compiled-in defaults");
        is_mission_loaded = true;
        return;
    }

    size_t needed = (size_t)count * MISSION_RECORD_SIZE;
    uint8_t buf[MAX_MISSIONS * MISSION_RECORD_SIZE];
    size_t got = prefs.getBytes(MISSION_PREF_KEY, buf, needed);
    prefs.end();

    if (got != needed || !unpackMissions(buf, needed)) {
        Serial.println("[WEB] NVS mission blob was malformed, keeping compiled-in defaults");
        return;
    }

    is_mission_loaded = true;
    Serial.printf("[WEB] Loaded %d mission(s) from NVS\n", NUM_STATES);
}

static const char MISSION_EDITOR_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Mission Sequence Editor</title>
<style>
  :root{
    --bg: #14171c;
    --panel: #1a1e25;
    --panel-2: #1f242c;
    --border: #2a2f38;
    --text: #e6e8eb;
    --muted: #8b93a1;
    --dim: #565e6b;
    --amber: #ffb454;
    --amber-dim: #4a3a20;
    --teal: #5ec8c0;
    --red: #e8685f;
    --mono: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
  }
  *{box-sizing:border-box;}
  body{
    margin:0;
    background: radial-gradient(ellipse at top left, #1a2028 0%, var(--bg) 45%);
    color:var(--text);
    font-family:var(--mono);
    font-size:13px;
    min-height:100vh;
  }
  ::selection{background:var(--amber-dim); color:var(--amber);}

  header{
    display:flex; align-items:center; justify-content:space-between;
    padding:16px 24px; border-bottom:1px solid var(--border);
    background:linear-gradient(180deg, #171b21 0%, #14171c 100%);
    flex-wrap:wrap; gap:10px;
  }
  .brand{display:flex; align-items:baseline; gap:10px;}
  .brand .dot{width:8px;height:8px;border-radius:50%;background:var(--amber); box-shadow:0 0 8px var(--amber); display:inline-block;}
  .brand h1{font-size:14px; letter-spacing:.12em; text-transform:uppercase; font-weight:600; margin:0; color:var(--text);}
  .brand .sub{color:var(--dim); font-size:11px; letter-spacing:.05em;}

  .toolbar{display:flex; gap:8px; flex-wrap:wrap;}
  button{
    font-family:var(--mono); font-size:11px; letter-spacing:.04em; text-transform:uppercase;
    background:var(--panel-2); color:var(--text); border:1px solid var(--border);
    padding:8px 12px; border-radius:3px; cursor:pointer;
    transition:border-color .15s, color .15s, background .15s;
  }
  button:hover{border-color:var(--amber); color:var(--amber);}
  button:focus-visible{outline:2px solid var(--amber); outline-offset:1px;}
  button.primary{background:var(--amber); color:#1a1305; border-color:var(--amber); font-weight:600;}
  button.primary:hover{background:#ffc574; color:#1a1305;}
  button.danger:hover{border-color:var(--red); color:var(--red);}
  button.ghost{background:transparent;}
  button:disabled{opacity:.35; cursor:not-allowed;}
  button:disabled:hover{border-color:var(--border); color:var(--text);}
  button.tiny{padding:4px 7px; font-size:10px;}

  main{
    display:grid; grid-template-columns:340px 1fr; gap:1px;
    background:var(--border); min-height:calc(100vh - 61px);
  }
  @media (max-width:900px){ main{grid-template-columns:1fr;} }

  .panel{background:var(--bg); padding:20px 22px; overflow:auto;}
  .panel-title{
    font-size:11px; text-transform:uppercase; letter-spacing:.12em; color:var(--dim);
    margin:0 0 14px 0; display:flex; justify-content:space-between; align-items:center;
  }
  .count{color:var(--teal);}

  .seq{list-style:none; margin:0; padding:0;}
  .seq-item{
    position:relative; margin:0; padding:10px 10px 10px 34px;
    border:1px solid transparent; border-radius:4px; cursor:pointer; background:var(--panel);
  }
  .seq-item::before{content:""; position:absolute; left:16px; top:-2px; bottom:50%; width:1px; background:var(--border);}
  .seq-item:first-child::before{display:none;}
  .seq-item::after{content:""; position:absolute; left:16px; top:50%; bottom:-2px; width:1px; background:var(--border);}
  .seq-item:last-child::after{display:none;}
  .seq-item .idx{
    position:absolute; left:10px; top:10px; width:13px; height:13px; border-radius:50%;
    background:var(--panel-2); border:1px solid var(--dim); font-size:9px; line-height:12px;
    text-align:center; color:var(--dim);
  }
  .seq-item.active{border-color:var(--amber); background:#221c10;}
  .seq-item.active .idx{border-color:var(--amber); color:var(--amber); background:var(--amber-dim);}
  .seq-item:hover:not(.active){border-color:var(--border); background:var(--panel-2);}
  .seq-item .label{font-weight:600; font-size:12.5px; color:var(--text);}
  .seq-item .meta{margin-top:4px; display:flex; gap:6px; flex-wrap:wrap;}
  .badge{
    font-size:9.5px; padding:2px 6px; border-radius:2px; background:var(--panel-2);
    color:var(--muted); border:1px solid var(--border); letter-spacing:.03em; white-space:nowrap;
  }
  .badge.drive{color:var(--teal); border-color:#28504c;}
  .badge.stop-STOP, .badge.stop-BRAKE{color:var(--red); border-color:#5a3230;}

  .seq-controls{position:absolute; right:8px; top:8px; display:none; gap:2px;}
  .seq-item:hover .seq-controls, .seq-item.active .seq-controls{display:flex;}
  .seq-controls button{padding:3px 6px; font-size:10px;}

  .empty-state{
    color:var(--dim); font-size:12px; padding:24px 10px; text-align:center;
    border:1px dashed var(--border); border-radius:4px; line-height:1.6;
  }

  .form-grid{display:grid; grid-template-columns:1fr 1fr; gap:14px 18px;}
  .field{display:flex; flex-direction:column; gap:6px;}
  .field.span2{grid-column:1 / -1;}
  .field label{
    font-size:10.5px; text-transform:uppercase; letter-spacing:.08em; color:var(--muted);
    display:flex; align-items:center; gap:6px;
  }
  .field .hint{font-size:10px; color:var(--dim); margin-top:-2px; line-height:1.5;}
  input[type=text], input[type=number], select{
    font-family:var(--mono); font-size:13px; background:var(--panel-2); border:1px solid var(--border);
    color:var(--text); padding:9px 10px; border-radius:3px; width:100%;
  }
  input:focus, select:focus{outline:none; border-color:var(--amber);}
  select{
    appearance:none; -webkit-appearance:none;
    background-image:linear-gradient(45deg, transparent 50%, var(--muted) 50%), linear-gradient(135deg, var(--muted) 50%, transparent 50%);
    background-position: calc(100% - 16px) 14px, calc(100% - 11px) 14px; background-size:5px 5px; background-repeat:no-repeat;
  }

  fieldset{border:1px solid var(--border); border-radius:5px; padding:14px 16px 16px; margin:0; transition:opacity .15s;}
  fieldset.na{opacity:.4;}
  fieldset.na::after{content:"tidak berpengaruh untuk mode ini"; display:block; font-size:9.5px; color:var(--dim); margin-top:8px; letter-spacing:.05em;}
  legend{font-size:10.5px; text-transform:uppercase; letter-spacing:.1em; color:var(--amber); padding:0 6px; display:flex; align-items:center; gap:6px;}
  .info-btn{
    width:14px; height:14px; border-radius:50%; border:1px solid var(--dim); color:var(--dim);
    font-size:9px; line-height:12px; text-align:center; cursor:pointer; background:transparent;
    padding:0; text-transform:none; letter-spacing:0;
  }
  .info-btn:hover{border-color:var(--amber); color:var(--amber);}

  .form-actions{display:flex; gap:10px; margin-top:20px; padding-top:16px; border-top:1px solid var(--border); flex-wrap:wrap;}

  /* Sensor bit grid */
  .bitfield{display:flex; flex-direction:column; gap:6px;}
  .bitfield-row{display:flex; gap:2px;}
  .bit-toggle{
    flex:1; min-width:0; font-family:var(--mono); font-size:10.5px; font-weight:600;
    background:var(--panel-2); border:1px solid var(--border); color:var(--dim);
    padding:6px 0; border-radius:3px; cursor:pointer; text-align:center; transition:all .1s;
  }
  .bit-toggle .n{display:block; font-size:7px; font-weight:400; color:var(--dim); margin-top:1px;}
  .bit-toggle.on{background:var(--amber-dim); border-color:var(--amber); color:var(--amber);}
  .bit-toggle.on .n{color:var(--amber);}
  .bit-toggle:hover{border-color:var(--amber);}
  .bit-readout{
    font-size:10.5px; color:var(--teal); background:#0f1216; border:1px solid var(--border);
    border-radius:3px; padding:6px 8px; display:flex; justify-content:space-between; flex-wrap:wrap; gap:2px;
  }
  .bit-readout .dec{color:var(--muted);}

  dialog{
    background:var(--panel); color:var(--text); border:1px solid var(--border);
    border-radius:6px; width:min(720px, 92vw); padding:0;
  }
  dialog::backdrop{background:rgba(0,0,0,.6);}
  .dialog-head{display:flex; justify-content:space-between; align-items:center; padding:14px 18px; border-bottom:1px solid var(--border); gap:12px;}
  .dialog-head h2{font-size:12px; text-transform:uppercase; letter-spacing:.1em; margin:0; color:var(--dim); white-space:nowrap;}
  .dialog-head select{width:auto; padding:6px 26px 6px 10px; font-size:11px;}
  dialog textarea{
    width:100%; height:380px; margin:0; padding:16px 18px; background:#0f1216; color:var(--teal);
    border:none; resize:vertical; font-family:var(--mono); font-size:11.5px; line-height:1.6;
  }
  .dialog-foot{display:flex; justify-content:flex-end; gap:8px; padding:12px 18px; border-top:1px solid var(--border);}

  #guideDialog .dialog-body{padding:6px 20px 20px; max-height:70vh; overflow:auto;}
  #guideDialog h3{font-size:12px; color:var(--amber); margin:18px 0 4px; letter-spacing:.03em;}
  #guideDialog h3:first-child{margin-top:14px;}
  #guideDialog p{margin:0 0 4px; color:var(--muted); line-height:1.7; font-size:12.5px;}
  #guideDialog code{color:var(--teal); background:#0f1216; padding:1px 5px; border-radius:2px; font-size:11.5px;}

  #fileInput{display:none;}
  #toast{
    position:fixed; bottom:20px; right:20px; background:var(--panel-2); border:1px solid var(--amber);
    color:var(--amber); padding:10px 16px; border-radius:4px; font-size:12px;
    opacity:0; transform:translateY(8px); transition:opacity .2s, transform .2s; pointer-events:none;
  }
  #toast.show{opacity:1; transform:translateY(0);}
</style>
</head>
<body>

<header>
  <div class="brand">
    <span class="dot"></span>
    <h1>Mission Sequence Editor</h1>
    <span class="sub">MissionState[] · runMission()</span>
  </div>
  <div class="toolbar">
    <button class="ghost" id="btnGuide">Field Guide</button>
    <button class="ghost" id="btnNew">+ New Mission</button>
    <button id="btnLoad">Load from robot</button>
    <button id="btnSave">Save to robot</button>
    <button id="btnExportCpp">Export Code</button>
  </div>
</header>

<input type="file" id="fileInput" accept=".bin,application/octet-stream">

<main>
  <section class="panel">
    <p class="panel-title">Sequence <span class="count" id="missionCount">0</span></p>
    <ul class="seq" id="seqList"></ul>
    <div class="empty-state" id="emptyState">
      No missions yet.<br>Click "+ New Mission" to add the first state.
    </div>
  </section>

  <section class="panel">
    <p class="panel-title" id="formTitle">Select or create a mission</p>
    <form id="missionForm" autocomplete="off">
      <div class="field span2">
        <label for="f_label">Label <span class="hint">(editor-only, not part of the struct)</span></label>
        <input type="text" id="f_label" placeholder="e.g. Approach ramp">
      </div>

      <div class="form-grid" style="margin-top:18px;">
        <fieldset class="span2" style="grid-column:1/-1;" id="fs_drive">
          <legend>Drive <button type="button" class="info-btn" data-guide="drive">i</button></legend>
          <div class="form-grid">
            <div class="field">
              <label for="f_driveMode">driveMode</label>
              <select id="f_driveMode">
                <option value="DIRECT_MOVE">DIRECT_MOVE</option>
                <option value="PID_STRAIGHT">PID_STRAIGHT</option>
                <option value="PID">PID</option>
              </select>
            </div>
            <div class="field">
              <label for="f_lineMode">lineMode</label>
              <select id="f_lineMode">
                <option value="LINE_BLACK">LINE_BLACK</option>
                <option value="LINE_WHITE">LINE_WHITE</option>
              </select>
            </div>
            <div class="field" id="fld_leftSpeed">
              <label for="f_leftSpeed">leftSpeed</label>
              <input type="number" id="f_leftSpeed" value="0" step="1">
            </div>
            <div class="field" id="fld_rightSpeed">
              <label for="f_rightSpeed">rightSpeed</label>
              <input type="number" id="f_rightSpeed" value="0" step="1">
            </div>
          </div>
        </fieldset>

        <fieldset style="grid-column:1/-1;">
          <legend>Exit condition <button type="button" class="info-btn" data-guide="condition">i</button></legend>
          <div class="form-grid">
            <div class="field">
              <label for="f_condition">condition</label>
              <select id="f_condition">
                <option value="COND_ENCODER1_GT">COND_ENCODER1_GT</option>
                <option value="COND_ENCODER2_GT">COND_ENCODER2_GT</option>
                <option value="COND_DIST_GT">COND_DIST_GT</option>
                <option value="COND_SENSOR_MASK">COND_SENSOR_MASK</option>
                <option value="COND_IMMEDIATE">COND_IMMEDIATE</option>
                <option value="COND_TIMER">COND_TIMER</option>
              </select>
            </div>
            <div class="field" id="fld_threshold">
              <label for="f_threshold">condition_threshold <span id="thresholdUnit" class="hint" style="margin:0;">int32</span></label>
              <input type="number" id="f_threshold" value="0" step="1">
            </div>
          </div>
        </fieldset>

        <fieldset style="grid-column:1/-1;" id="fs_sensor">
          <legend>Sensor mask <button type="button" class="info-btn" data-guide="sensor">i</button></legend>
          <p class="hint" style="margin:-4px 0 12px;">Tiap bit mewakili satu sensor fisik pada sisi tersebut. Nyalakan bit sensor yang harus (AND) atau boleh (OR) mendeteksi garis agar misi dianggap selesai.</p>
          <div class="form-grid">
            <div class="field">
              <label>sensorLeft <span class="hint" id="leftPolarity">1 = mendeteksi garis</span></label>
              <div class="bitfield" id="bitsLeft"></div>
            </div>
            <div class="field">
              <label>sensorRight <span class="hint" id="rightPolarity">1 = mendeteksi garis</span></label>
              <div class="bitfield" id="bitsRight"></div>
            </div>
            <div class="field span2">
              <label for="f_maskMode">maskMode</label>
              <select id="f_maskMode">
                <option value="MASK_AND">MASK_AND</option>
                <option value="MASK_OR">MASK_OR</option>
              </select>
            </div>
          </div>
        </fieldset>

        <fieldset style="grid-column:1/-1;">
          <legend>Stop mode <button type="button" class="info-btn" data-guide="stop">i</button></legend>
          <div class="form-grid">
            <div class="field">
              <label for="f_stopMode">stopMode</label>
              <select id="f_stopMode">
                <option value="NONE">NONE</option>
                <option value="STOP">STOP</option>
                <option value="BRAKE">BRAKE</option>
              </select>
            </div>
          </div>
        </fieldset>
      </div>

      <div class="form-actions">
        <button type="submit" class="primary" id="btnSaveMission">Add to sequence</button>
        <button type="button" id="btnDuplicate" disabled>Duplicate</button>
        <button type="button" class="danger" id="btnDelete" disabled>Delete</button>
        <button type="button" class="ghost" id="btnClear">Clear form</button>
      </div>
    </form>
  </section>
</main>

<dialog id="exportDialog">
  <div class="dialog-head">
    <h2>Export</h2>
    <select id="exportFormat">
      <option value="array">C++ array literal (missions.h)</option>
      <option value="esp32">ESP32 Preferences backend (.ino)</option>
    </select>
    <button class="ghost" id="btnCloseDialog">Close</button>
  </div>
  <textarea id="exportText" readonly spellcheck="false"></textarea>
  <div class="dialog-foot">
    <button id="btnCopyExport">Copy to clipboard</button>
    <button id="btnDownloadExport" class="primary">Download</button>
  </div>
</dialog>

<dialog id="guideDialog">
  <div class="dialog-head">
    <h2>Field Guide</h2>
    <button class="ghost" id="btnCloseGuide">Close</button>
  </div>
  <div class="dialog-body">
    <h3 data-section="drive">1. DRIVE_MODE</h3>
    <p>mode pengendalian. PID berarti robot akan mengikuti garis menggunakan koreksi PID, DIRECT_MOVE berarti menggerakkan motor secara langsung</p>
    <h3 data-section="drive">2. LINE_MODE</h3>
    <p>LINE_BLACK untuk mengikuti garis hitam, LINE_WHITE untuk mengikuti garis putih</p>
    <h3 data-section="drive">3. LEFTSPEED</h3>
    <p>kecepatan motor kiri. nilai negatif berarti berputar ke belakang. tidak berpengaruh saat mode PID</p>
    <h3 data-section="drive">4. RIGHTSPEED</h3>
    <p>kecepatan motor kanan. nilai negatif berarti berputar ke belakang. tidak berpengaruh saat mode PID</p>
    <h3 data-section="condition">5. CONDITION</h3>
    <p>COND_ENCODER1_GT: misi selesai saat roda kiri berputar sebanyak threshold tertentu. COND_ENCODER2_GT: misi selesai saat roda kiri berputar sebanyak threshold tertentu. COND_DIST_GT: misi selesai saat kedua roda berputar sebanyak threshold. COND_SENSOR_MASK: misi selesai saat sensor mendeteksi garis sesuai dengan SENSOR MASK. SENSOR MASK tidak berpegaruh selain mode COND_SENSOR_MASK. COND_TIMER: misi selesai setelah waktu tertentu.</p>
    <h3 data-section="condition">6. CONDITION_THRESHOLD</h3>
    <p>nilai batasan sebelum misi dianggap selesai. jika COND_TIMER, maka nilai ini adalah waktu. jika COND_ENCODER ataupun COND_DIST_GT, nilai ini adalah jarak putaran roda. tidak berpengaruh ketika COND_SENSOR_MARK</p>
    <h3 data-section="sensor">7. SENSORLEFT</h3>
    <p>syarat misi selesai untuk sensor kiri. 0 berarti tidak mendeteksi garis, 1 berarti mendeteksi garis. berlaku sebaliknya untuk mode LINE_WHITE</p>
    <h3 data-section="sensor">8. SENSORLEFT</h3>
    <p>syarat misi selesai untuk sensor kanan. 0 berarti tidak mendeteksi garis, 1 berarti mendeteksi garis. berlaku sebaliknya untuk mode LINE_WHITE</p>
    <h3 data-section="sensor">9. MASK_MODE</h3>
    <p>MASK_AND: kondisi sensor harus sama persis dengan kedua sensor mask sebelum misi dianggap selesai. MASK_OR: misi selesai apabila salah satu sensor sesuai kondisi SENSOR MASK</p>
    <h3 data-section="stop">10. STOPMODE</h3>
    <p>kondisi robot setelah menyelesaikan misi. NONE berarti robot langsung masuk ke misi selanjutnya. STOP berarti robot akan berusaha berhenti. BRAKE berarti robot mengerem untuk pemberhentian yang lebih akurat.</p>
  </div>
</dialog>

<div id="toast"></div>

<script>
(function(){
  "use strict";

  const ENUMS = {
    lineMode:   ['LINE_BLACK','LINE_WHITE'],
    driveMode:  ['DIRECT_MOVE','PID_STRAIGHT','PID'],
    condition:  ['COND_ENCODER1_GT','COND_ENCODER2_GT','COND_DIST_GT','COND_SENSOR_MASK','COND_IMMEDIATE','COND_TIMER'],
    maskMode:   ['MASK_AND','MASK_OR'],
    stopMode:   ['NONE','STOP','BRAKE'],
  };

  let missions = [];
  let selectedId = null;

  const $ = id => document.getElementById(id);
  const els = {
    seqList: $('seqList'), emptyState: $('emptyState'), missionCount: $('missionCount'),
    formTitle: $('formTitle'), form: $('missionForm'),
    btnSaveMission: $('btnSaveMission'), btnDuplicate: $('btnDuplicate'), btnDelete: $('btnDelete'),
    btnClear: $('btnClear'), btnNew: $('btnNew'), btnLoad: $('btnLoad'), btnSave: $('btnSave'),
    btnExportCpp: $('btnExportCpp'), fileInput: $('fileInput'),
    exportDialog: $('exportDialog'), exportText: $('exportText'), exportFormat: $('exportFormat'),
    btnCloseDialog: $('btnCloseDialog'), btnCopyExport: $('btnCopyExport'), btnDownloadExport: $('btnDownloadExport'),
    guideDialog: $('guideDialog'), btnGuide: $('btnGuide'), btnCloseGuide: $('btnCloseGuide'),
    toast: $('toast'),
    bitsLeft: $('bitsLeft'), bitsRight: $('bitsRight'),
  };

  let bitsLeft = new Array(8).fill(0);
  let bitsRight = new Array(8).fill(0);

  function buildBitfield(container, bitsArr, onChange){
    container.innerHTML = '';
    const row = document.createElement('div');
    row.className = 'bitfield-row';
    for(let i = 0; i < 8; i++){
      const bitPos = 7 - i;
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'bit-toggle';
      btn.innerHTML = `${bitsArr[i]}<span class="n">${bitPos}</span>`;
      btn.addEventListener('click', () => {
        bitsArr[i] = bitsArr[i] ? 0 : 1;
        btn.classList.toggle('on', !!bitsArr[i]);
        btn.innerHTML = `${bitsArr[i]}<span class="n">${bitPos}</span>`;
        onChange();
      });
      if(bitsArr[i]) btn.classList.add('on');
      row.appendChild(btn);
    }
    container.appendChild(row);
    const readout = document.createElement('div');
    readout.className = 'bit-readout';
    container.appendChild(readout);
    updateReadout(container, bitsArr);
  }

  function updateReadout(container, bitsArr){
    const readout = container.querySelector('.bit-readout');
    const val = bitsToByte(bitsArr);
    readout.innerHTML = `<span>0b${bitsArr.join('')}</span><span class="dec">= ${val} (0x${val.toString(16).padStart(2,'0').toUpperCase()})</span>`;
  }

  function bitsToByte(bitsArr){
    let v = 0;
    for(let i = 0; i < 8; i++) v |= bitsArr[i] << (7 - i);
    return v;
  }

  function byteToBits(byte){
    const arr = new Array(8).fill(0);
    for(let i = 0; i < 8; i++) arr[i] = (byte >> (7 - i)) & 1;
    return arr;
  }

  function refreshBitfields(){
    buildBitfield(els.bitsLeft, bitsLeft, () => updateReadout(els.bitsLeft, bitsLeft));
    buildBitfield(els.bitsRight, bitsRight, () => updateReadout(els.bitsRight, bitsRight));
  }

  function uid(){ return 'm_' + Date.now().toString(36) + Math.random().toString(36).slice(2,7); }
  function clampInt(v){ const n = parseInt(v, 10); return Number.isFinite(n) ? n : 0; }

  function readForm(){
    return {
      label: $('f_label').value.trim() || 'Untitled mission',
      driveMode: $('f_driveMode').value,
      lineMode: $('f_lineMode').value,
      leftSpeed: clampInt($('f_leftSpeed').value),
      rightSpeed: clampInt($('f_rightSpeed').value),
      condition: $('f_condition').value,
      condition_threshold: clampInt($('f_threshold').value),
      sensorLeft: bitsToByte(bitsLeft),
      sensorRight: bitsToByte(bitsRight),
      maskMode: $('f_maskMode').value,
      stopMode: $('f_stopMode').value,
    };
  }

  function writeForm(m){
    $('f_label').value = m.label || '';
    $('f_driveMode').value = m.driveMode;
    $('f_lineMode').value = m.lineMode;
    $('f_leftSpeed').value = m.leftSpeed;
    $('f_rightSpeed').value = m.rightSpeed;
    $('f_condition').value = m.condition;
    $('f_threshold').value = m.condition_threshold;
    bitsLeft = byteToBits(m.sensorLeft);
    bitsRight = byteToBits(m.sensorRight);
    $('f_maskMode').value = m.maskMode;
    $('f_stopMode').value = m.stopMode;
    refreshBitfields();
    updateApplicability();
    updatePolarityHints();
  }

  function resetForm(){
    els.form.reset();
    writeForm({
      label:'', driveMode:'DIRECT_MOVE', lineMode:'LINE_BLACK',
      leftSpeed:0, rightSpeed:0, condition:'COND_IMMEDIATE', condition_threshold:0,
      sensorLeft:0, sensorRight:0, maskMode:'MASK_AND', stopMode:'NONE'
    });
  }

  function updateApplicability(){
    const driveMode = $('f_driveMode').value;
    const condition = $('f_condition').value;
    $('fld_leftSpeed').style.opacity = driveMode === 'PID' ? .4 : 1;
    $('fld_rightSpeed').style.opacity = driveMode === 'PID' ? .4 : 1;
    $('fs_sensor').classList.toggle('na', condition !== 'COND_SENSOR_MASK');
    $('fld_threshold').style.opacity = condition === 'COND_SENSOR_MASK' ? .4 : 1;
    const unit = $('thresholdUnit');
    unit.textContent = condition === 'COND_TIMER' ? 'ms' :
      (condition === 'COND_ENCODER1_GT' || condition === 'COND_ENCODER2_GT' || condition === 'COND_DIST_GT') ? 'ticks' : 'int32';
  }

  function updatePolarityHints(){
    const white = $('f_lineMode').value === 'LINE_WHITE';
    $('leftPolarity').textContent = white ? '1 = TIDAK mendeteksi garis' : '1 = mendeteksi garis';
    $('rightPolarity').textContent = white ? '1 = TIDAK mendeteksi garis' : '1 = mendeteksi garis';
  }

  ['f_driveMode','f_condition'].forEach(id => $(id).addEventListener('change', updateApplicability));
  $('f_lineMode').addEventListener('change', updatePolarityHints);

  document.querySelectorAll('[data-guide]').forEach(btn => {
    btn.addEventListener('click', () => {
      els.guideDialog.showModal();
      const target = document.querySelector(`#guideDialog h3[data-section="${btn.dataset.guide}"]`);
      if(target) target.scrollIntoView({block:'start'});
    });
  });
  els.btnGuide.addEventListener('click', () => els.guideDialog.showModal());
  els.btnCloseGuide.addEventListener('click', () => els.guideDialog.close());

  function selectMission(id){
    selectedId = id;
    const m = missions.find(x => x.id === id);
    if(m){
      writeForm(m);
      els.formTitle.textContent = 'Editing #' + (missions.indexOf(m)+1) + ' — ' + m.label;
      els.btnSaveMission.textContent = 'Update mission';
      els.btnDuplicate.disabled = false;
      els.btnDelete.disabled = false;
    }
    render();
  }

  function deselect(){
    selectedId = null;
    els.formTitle.textContent = 'Select or create a mission';
    els.btnSaveMission.textContent = 'Add to sequence';
    els.btnDuplicate.disabled = true;
    els.btnDelete.disabled = true;
    render();
  }

  function render(){
    els.missionCount.textContent = missions.length;
    els.emptyState.style.display = missions.length ? 'none' : 'block';
    els.seqList.innerHTML = '';
    missions.forEach((m, i) => {
      const li = document.createElement('li');
      li.className = 'seq-item' + (m.id === selectedId ? ' active' : '');
      const stopBadge = m.stopMode !== 'NONE' ? `<span class="badge stop-${m.stopMode}">${m.stopMode}</span>` : '';
      li.innerHTML = `
        <span class="idx">${i+1}</span>
        <div class="label">${escapeHtml(m.label)}</div>
        <div class="meta">
          <span class="badge drive">${m.driveMode}</span>
          <span class="badge">${m.condition}</span>
          ${stopBadge}
        </div>
        <div class="seq-controls">
          <button type="button" class="tiny" data-act="up" title="Move up">↑</button>
          <button type="button" class="tiny" data-act="down" title="Move down">↓</button>
        </div>`;
      li.addEventListener('click', (e) => { if(!e.target.closest('[data-act]')) selectMission(m.id); });
      li.querySelector('[data-act="up"]').addEventListener('click', e => { e.stopPropagation(); moveMission(i,-1); });
      li.querySelector('[data-act="down"]').addEventListener('click', e => { e.stopPropagation(); moveMission(i,1); });
      els.seqList.appendChild(li);
    });
  }

  function escapeHtml(s){
    return String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  }

  function moveMission(index, dir){
    const target = index + dir;
    if(target < 0 || target >= missions.length) return;
    const [item] = missions.splice(index, 1);
    missions.splice(target, 0, item);
    render();
  }

  function showToast(msg){
    els.toast.textContent = msg;
    els.toast.classList.add('show');
    clearTimeout(showToast._t);
    showToast._t = setTimeout(() => els.toast.classList.remove('show'), 1800);
  }

  els.form.addEventListener('submit', (e) => {
    e.preventDefault();
    const data = readForm();
    if(selectedId){
      Object.assign(missions.find(x => x.id === selectedId), data);
      showToast('Mission updated');
    } else {
      missions.push({ id: uid(), ...data });
      showToast('Mission added');
      selectedId = missions[missions.length-1].id;
      els.formTitle.textContent = 'Editing #' + missions.length + ' — ' + data.label;
      els.btnSaveMission.textContent = 'Update mission';
      els.btnDuplicate.disabled = false;
      els.btnDelete.disabled = false;
    }
    render();
  });

  els.btnNew.addEventListener('click', () => { resetForm(); deselect(); });
  els.btnClear.addEventListener('click', () => { resetForm(); deselect(); });

  els.btnDuplicate.addEventListener('click', () => {
    if(!selectedId) return;
    const src = missions.find(x => x.id === selectedId);
    const idx = missions.indexOf(src);
    const copy = { ...src, id: uid(), label: src.label + ' (copy)' };
    missions.splice(idx+1, 0, copy);
    selectMission(copy.id);
    showToast('Mission duplicated');
  });

  els.btnDelete.addEventListener('click', () => {
    if(!selectedId) return;
    missions = missions.filter(x => x.id !== selectedId);
    resetForm(); deselect();
    showToast('Mission deleted');
  });

  // ---- Binary format: fixed 19-byte record per mission, little-endian ----
  const RECORD_SIZE = 19;

  function packBinary(list){
    const buf = new ArrayBuffer(RECORD_SIZE * list.length);
    const view = new DataView(buf);
    list.forEach((m, i) => {
      const o = i * RECORD_SIZE;
      view.setUint8(o+0, ENUMS.lineMode.indexOf(m.lineMode));
      view.setUint8(o+1, ENUMS.driveMode.indexOf(m.driveMode));
      view.setInt32(o+2, m.leftSpeed, true);
      view.setInt32(o+6, m.rightSpeed, true);
      view.setUint8(o+10, ENUMS.condition.indexOf(m.condition));
      view.setInt32(o+11, m.condition_threshold, true);
      view.setUint8(o+15, m.sensorRight);
      view.setUint8(o+16, m.sensorLeft);
      view.setUint8(o+17, ENUMS.maskMode.indexOf(m.maskMode));
      view.setUint8(o+18, ENUMS.stopMode.indexOf(m.stopMode));
    });
    return new Uint8Array(buf);
  }

  function unpackBinary(bytes){
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const count = Math.floor(bytes.length / RECORD_SIZE);
    const out = [];
    for(let i = 0; i < count; i++){
      const o = i * RECORD_SIZE;
      out.push({
        label: 'Mission ' + (i+1),
        lineMode: ENUMS.lineMode[view.getUint8(o+0)] || 'LINE_BLACK',
        driveMode: ENUMS.driveMode[view.getUint8(o+1)] || 'DIRECT_MOVE',
        leftSpeed: view.getInt32(o+2, true),
        rightSpeed: view.getInt32(o+6, true),
        condition: ENUMS.condition[view.getUint8(o+10)] || 'COND_IMMEDIATE',
        condition_threshold: view.getInt32(o+11, true),
        sensorRight: view.getUint8(o+15),
        sensorLeft: view.getUint8(o+16),
        maskMode: ENUMS.maskMode[view.getUint8(o+17)] || 'MASK_AND',
        stopMode: ENUMS.stopMode[view.getUint8(o+18)] || 'NONE',
      });
    }
    return out;
  }

  function triggerDownload(blob, filename){
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = filename;
    document.body.appendChild(a); a.click(); a.remove();
    URL.revokeObjectURL(url);
  }

  // ---- Load/save now go straight to the robot over HTTP ----
  els.btnSave.addEventListener('click', async () => {
    const bytes = packBinary(missions);
    try{
      const res = await fetch('/save', { method:'POST', body: bytes, headers:{'Content-Type':'application/octet-stream'} });
      if(!res.ok) throw new Error(await res.text());
      showToast(`Saved ${missions.length} mission(s) to robot`);
    }catch(err){
      alert('Save failed: ' + err.message);
    }
  });

  els.btnLoad.addEventListener('click', async () => {
    try{
      const res = await fetch('/load', { cache:'no-store' });
      if(!res.ok) throw new Error(await res.text());
      const bytes = new Uint8Array(await res.arrayBuffer());
      if(bytes.length % RECORD_SIZE !== 0) throw new Error(`Size ${bytes.length} is not a multiple of ${RECORD_SIZE} bytes`);
      const parsed = unpackBinary(bytes);
      missions = parsed.map(m => ({ id: uid(), ...m }));
      deselect();
      showToast(`Loaded ${missions.length} mission(s) from robot`);
    }catch(err){
      alert('Load failed: ' + err.message);
    }
  });

  // ---- Export: C++ array literal, or ESP32 Preferences backend code ----
  els.btnExportCpp.addEventListener('click', () => {
    els.exportText.value = buildExport(els.exportFormat.value);
    els.exportDialog.showModal();
  });
  els.exportFormat.addEventListener('change', () => {
    els.exportText.value = buildExport(els.exportFormat.value);
  });
  els.btnCloseDialog.addEventListener('click', () => els.exportDialog.close());
  els.btnCopyExport.addEventListener('click', async () => {
    await navigator.clipboard.writeText(els.exportText.value);
    showToast('Copied to clipboard');
  });
  els.btnDownloadExport.addEventListener('click', () => {
    const fmt = els.exportFormat.value;
    const filename = fmt === 'array' ? 'missions.h' : 'mission_backend.ino';
    triggerDownload(new Blob([els.exportText.value], {type:'text/plain'}), filename);
  });

  function buildExport(fmt){
    return fmt === 'array' ? buildCppArray() : buildEsp32Backend();
  }

  function buildCppArray(){
    if(missions.length === 0) return '// No missions defined.\n';
    const lines = ['#include "mission.h"', '', `MissionState missions[${missions.length}] = {`];
    missions.forEach((m, i) => {
      lines.push(`    // [${i}] ${m.label}`);
      lines.push('    {');
      lines.push(`        LineMode::${m.lineMode},`);
      lines.push(`        DriveMode::${m.driveMode},`);
      lines.push(`        ${m.leftSpeed}, ${m.rightSpeed},`);
      lines.push(`        ConditionType::${m.condition},`);
      lines.push(`        ${m.condition_threshold},`);
      lines.push(`        ${m.sensorRight}, // sensorRight = 0b${m.sensorRight.toString(2).padStart(8,'0')}`);
      lines.push(`        ${m.sensorLeft}, // sensorLeft  = 0b${m.sensorLeft.toString(2).padStart(8,'0')}`);
      lines.push(`        MaskMode::${m.maskMode},`);
      lines.push(`        StopMode::${m.stopMode}`);
      lines.push(`    }${i < missions.length-1 ? ',' : ''}`);
    });
    lines.push('};', '');
    return lines.join('\n');
  }

  function buildEsp32Backend(){
    return `// See web_server.cpp on the device for the live implementation.
// This export is just a reference copy of the pack/unpack + Preferences logic.
`;
  }

  refreshBitfields();
  resetForm();
  render();
})();
</script>
</body>
</html>
)rawliteral";

void handleIndex() {
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, "text/html", MISSION_EDITOR_HTML);
}


void handleLoadMission() {
    static uint8_t buf[MAX_MISSIONS * MISSION_RECORD_SIZE];
    size_t n = packMissions(buf, sizeof(buf));

    webServer.sendHeader("Cache-Control", "no-store");
    webServer.setContentLength(n);
    webServer.send(200, "application/octet-stream", "");
    webServer.sendContent((const char *)buf, n);

    Serial.printf("[WEB] /load -> %d mission(s), %u bytes\n", NUM_STATES, (unsigned)n);
}

void handleSaveMission() {
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "text/plain", "Missing body");
        return;
    }

    const String &body = webServer.arg("plain");
    size_t len = body.length();

    if (len == 0 || len % MISSION_RECORD_SIZE != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Body size %u is not a multiple of %d bytes", (unsigned)len, MISSION_RECORD_SIZE);
        webServer.send(400, "text/plain", msg);
        return;
    }

    if (!unpackMissions((const uint8_t *)body.c_str(), len)) {
        webServer.send(400, "text/plain", "Unpack failed (too many missions?)");
        return;
    }

    persistMissionsToNVS();

    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "text/plain", "OK");

    Serial.printf("[WEB] /save <- %d mission(s), %u bytes\n", NUM_STATES, (unsigned)len);
}

void handleNotFound() {
    webServer.send(404, "text/plain", "Not found");
}

void startMissionWebServer() {
    if (web_server_started) return;

    loadMissionsFromNVS();

    webServer.on("/", HTTP_GET, handleIndex);
    webServer.on("/load", HTTP_GET, handleLoadMission);
    webServer.on("/save", HTTP_POST, handleSaveMission);
    webServer.onNotFound(handleNotFound);

    webServer.begin();
    web_server_started = true;
    Serial.println("[WEB] Mission editor server started on port 80");
}

void handleMissionWebServer() {
    if (web_server_started) {
        webServer.handleClient();
    }
}

static IPAddress ip;

void enableHotspot() {
    if (!web_server_started) {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        
        ip = WiFi.softAPIP();
    }

    char line1buf[24];
    char line2buf[24];
    char line3buf[24];

    snprintf(line1buf, sizeof(line1buf), "[WEB] Hotspot \"%s\" up", AP_SSID);
    snprintf(line2buf, sizeof(line2buf), "connect and browse to");
    snprintf(line3buf, sizeof(line3buf), "http://%u.%u.%u.%u./\n", ip[0], ip[1], ip[2], ip[3]);

    displayOLED(line1buf, line2buf, line3buf, "");

    Serial.printf("[WEB] Hotspot \"%s\" up, connect and browse to http://%u.%u.%u.%u/\n",
                  AP_SSID, ip[0], ip[1], ip[2], ip[3]);

    startMissionWebServer();
}

bool isMissionLoaded() {
    return is_mission_loaded;
}

bool isWebServerStarted() {
    return web_server_started;
}
