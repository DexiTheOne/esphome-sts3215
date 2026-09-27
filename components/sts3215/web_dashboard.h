#pragma once

// Self-contained device page. The state and actions stay on the ESPHome node.
static const char STS3215_DASHBOARD_HTML[] PROGMEM = R"stsweb(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Blind calibration</title><style>
:root{color-scheme:dark;font:15px/1.4 system-ui,sans-serif;background:#0b1220;color:#e9f1fc}
*{box-sizing:border-box}body{margin:0}main{max-width:1180px;margin:auto;padding:24px 18px 70px}
header{display:flex;justify-content:space-between;align-items:center;gap:16px;margin-bottom:24px}
h1{font-size:clamp(24px,4vw,36px);line-height:1.1;margin:4px 0}h2{margin:0;font-size:22px}
.eyebrow{color:#76adf8;text-transform:uppercase;letter-spacing:.14em;font-size:11px;font-weight:800}
.muted{color:#9babc3}.connection{border:1px solid #344760;border-radius:100px;padding:6px 12px;font-size:13px}
.connection.live{color:#91e2a4;border-color:#36744b}.stack{display:grid;gap:18px}
.card{background:#111d30;border:1px solid #273b56;border-radius:18px;padding:20px;box-shadow:0 12px 30px #0003}
.cardhead{display:flex;justify-content:space-between;gap:12px;align-items:center;margin-bottom:17px}
.badge{font-size:12px;border-radius:100px;padding:5px 10px;background:#24344d;color:#acc5e7}
.badge.ok{background:#123f2e;color:#8ce0aa}.badge.warn{background:#50341d;color:#ffc58b}
.groupControls,.tiltChoices,.legend,.settings,.telemetry{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
button,select,input{font:inherit}button{background:#1d3350;border:1px solid #436184;border-radius:9px;color:#e7f2ff;padding:9px 12px;cursor:pointer}
button:hover:not(:disabled){background:#28507a;border-color:#73aef2}button:disabled{opacity:.45;cursor:not-allowed}
button.primary{background:#2562a5;border-color:#5ba1f0}button.danger{background:#492b32;border-color:#96505b;color:#ffd6db}
button.small{padding:6px 10px;font-size:13px}button.active{border-color:#5db0f8;background:#244b73}
input,select{background:#0b1727;border:1px solid #405873;border-radius:8px;color:#f3f8ff;padding:7px 9px}
input[type=number]{width:82px}label{display:inline-flex;align-items:center;gap:7px;color:#b8c9df}
.calGrid{display:grid;grid-template-columns:minmax(230px,1fr) minmax(210px,.85fr) minmax(310px,1.15fr);gap:14px;align-items:stretch}
.manual,.plotCol,.readout{background:#0c1828;border:1px solid #2b3e56;border-radius:14px;padding:16px;min-width:0}
.panelTitle{font-size:20px;font-weight:800;margin:0 0 15px}.switchRow{display:flex;justify-content:space-between;align-items:center;gap:8px;background:#152438;border:1px solid #314963;border-radius:10px;padding:10px 12px;margin-bottom:13px}.switchRow input{accent-color:#168af5;width:21px;height:21px}.panelSub{font-size:15px;font-weight:800;margin:14px 0 8px;padding-top:12px;border-top:1px solid #32445a}
.manual{display:flex;flex-direction:column;gap:12px}
.manualRow{display:grid;grid-template-columns:44px 1fr;gap:8px}.manualRow button{min-height:42px}.manualRow .arrow{font-size:19px;padding:6px}
.jogBox{margin-top:6px}.jogBox input{width:94px}.reset{margin-top:4px;width:100%}
.stopButton{background:#59343a;border-color:#a85d67}.positionRow{background:#142337;border:1px solid #2e465f;border-radius:11px;padding:12px;margin-bottom:9px}.positionRow header{margin:0 0 8px;align-items:center}.positionRow strong{font-size:18px}.positionRow .kind{font-size:11px;border:1px solid currentColor;border-radius:6px;padding:2px 6px}.positionRow .value{font-size:24px;font-weight:800;font-variant-numeric:tabular-nums}.positionRow input{width:100%;font-size:20px}.positionRow .green{color:#75e5a1}.positionRow .blue{color:#5aaaff}.savePositions{width:100%;margin-bottom:9px}.stateLine{background:#142337;border-radius:9px;padding:8px 11px;display:flex;justify-content:space-between;margin-bottom:7px}
.plotCol{display:flex;flex-direction:column;align-items:stretch;gap:9px}.plotCol>.primary{min-height:42px}
.plotWrap{height:320px;position:relative;margin:4px 12px 0 25px}
.track{position:absolute;left:35px;right:35px;top:0;bottom:0;background:#0a1524;border:1px solid #527091;border-radius:8px;overflow:hidden}
.fill{position:absolute;left:0;right:0;bottom:0;background:linear-gradient(0deg,#2169bd99,#43b5f766);height:0;transition:height .35s}
.mark{position:absolute;left:0;right:0;height:0;border-top:2px solid;z-index:2;transform:translateY(-1px)}
.mark.green{border-color:#75e5a1}.mark.blue{border-color:#5aaaff}.mark.red{border-color:#ff6b72}
.mark.striped{height:6px;border:0;background:repeating-linear-gradient(90deg,#ff6b72 0 8px,#75e5a1 8px 16px)}
.mark.target{border-style:dashed}.mark.current{border-color:#bdeeff;border-width:3px;z-index:4}
.markLabel{position:absolute;font-size:10px;font-weight:800;letter-spacing:.06em;z-index:5;transform:translateY(-50%);background:#111d30;padding:0 3px}
.markLabel.left{left:0}.markLabel.right{right:0}.markLabel.green{color:#75e5a1}.markLabel.blue{color:#5aaaff}.markLabel.red{color:#ff6b72}
.direction{display:flex;flex-direction:column;gap:5px;font-size:12px}.direction select{width:100%;font-size:13px}
.readout{display:flex;flex-direction:column;gap:13px}.liveLine{padding:10px 12px;background:#172a42;border-radius:10px;display:flex;justify-content:space-between;gap:10px}
.livePair{display:grid;grid-template-columns:1fr 1fr;gap:8px}.livePair .liveLine{display:flex;flex-direction:column;gap:3px}.livePair strong{font-size:20px;font-variant-numeric:tabular-nums}
.values{display:grid;grid-template-columns:1fr auto auto;gap:0;border:1px solid #314a68;border-radius:11px;overflow:hidden}
.values>*{padding:11px 10px;border-bottom:1px solid #314a68}.values>*:nth-last-child(-n+3){border-bottom:0}
.values .head{font-size:11px;color:#9eb1ca;text-transform:uppercase;letter-spacing:.05em;background:#18283f}
.values .num{text-align:right;font-variant-numeric:tabular-nums}.values .rowlabel{font-weight:700}
.values .green{color:#80e3a7}.values .blue{color:#77b6ff}
.legend{font-size:12px;color:#b7c8de;gap:12px;margin-top:auto}.key{display:inline-flex;align-items:center;gap:5px}.swatch{width:14px;border-top:3px solid}
.details{margin-top:19px;padding-top:17px;border-top:1px solid #293f5b}.details summary{cursor:pointer;color:#c3d8f2;font-weight:700}
.detailGrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;margin-top:14px}
.metric{background:#0b192b;border-radius:10px;padding:9px 11px}.metric small{display:block;color:#91a8c2}.metric strong{font-variant-numeric:tabular-nums}
.settings{margin-top:16px}.settings label{background:#0b192b;border-radius:9px;padding:7px 9px}.settings button{padding:7px 10px}
.note{font-size:12px;color:#9aacc4;margin-top:12px}.toast{position:fixed;bottom:15px;left:50%;transform:translateX(-50%);background:#244363;border:1px solid #74b5f5;border-radius:9px;padding:9px 14px;z-index:20}
@media(max-width:900px){.calGrid{grid-template-columns:1fr 1fr}.readout{grid-column:1/-1}.card{padding:15px}}
@media(max-width:600px){.calGrid{grid-template-columns:1fr}.readout{grid-column:auto}.plotWrap{height:330px}.manualRow{grid-template-columns:52px 1fr}}
</style></head><body><main>
<header><div><div class="eyebrow">Common Area Blinds</div><h1>Blind calibration</h1><div class="muted">Encoder positions and motor controls</div></div><div id="connection" class="connection">Connecting…</div></header>
<section id="all" class="card" hidden></section><div id="cards" class="stack" style="margin-top:18px"></div>
<p class="note">Blue fill shows the last known blind position. Markers outside the calibrated range are capped at the bar edge.</p>
</main><div id="toast" class="toast" hidden></div><script>
const $=s=>document.querySelector(s), esc=s=>encodeURIComponent(s), fmt=n=>Number.isFinite(n)?Math.round(n).toLocaleString():'—';
const root=$('#cards');let last={}, busy=false, toastTimer;
function toast(message){const el=$('#toast');el.textContent=message;el.hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>el.hidden=true,4000)}
function button(id,label,action,extra=''){return `<button data-id="${id}" data-action="${action}" ${extra}>${label}</button>`}
function makeCard(s){const id=s.id;const article=document.createElement('article');article.className='card';article.id=`blind-${id}`;article.innerHTML=`
  <div class="cardhead"><div><div class="eyebrow">Servo ${id}</div><h2>Blind ${id}</h2></div><span class="badge status">Loading</span></div>
  <div class="calGrid"><div class="manual"><div class="panelTitle">🎮 Manual Control</div><label class="switchRow">Manual Control <input class="manualToggle" type="checkbox" data-id="${id}"></label>
    <div class="manualRow">${button(id,'↑','jog-up','class="arrow" aria-label="Jog up"')}${button(id,'Set Fully Up','set-up','class="set"')}</div>
    <div class="manualRow">${button(id,'■','stop','class="arrow stopButton" aria-label="Stop movement and auto calibration"')}${button(id,'Set Middle','set-middle','class="set"')}</div>
    <div class="manualRow">${button(id,'↓','jog-down','class="arrow" aria-label="Jog down"')}${button(id,'Set Fully Down','set-down','class="set"')}</div>
    <div class="jogBox"><label>Jog step <input class="jog" type="number" min="0.1" max="2520" step="0.1" value="10"> degrees</label><button class="small" data-id="${id}" data-action="save-jog">Save</button></div>
    ${button(id,'Reset calibration','reset','class="danger reset"')}
  </div><div class="plotCol"><div class="panelTitle">⚙ Calibration</div>
    ${button(id,'Auto calibrate','auto','class="primary auto"')}
    <div class="plotWrap"><div class="track"><div class="fill"></div></div><div class="markers"></div></div>
    <label class="direction">Negative direction<select data-id="${id}" class="directionSelect"><option>Negative is down</option><option>Negative is up</option></select></label>
  </div><div class="readout"><div class="panelTitle">▥ Position Data</div><label class="switchRow">Edit Positions <input class="editToggle" type="checkbox" data-id="${id}"></label>
    <div class="livePair"><div class="liveLine"><span>Current encoder</span><strong class="currentValue">—</strong></div>
    <div class="liveLine"><span>Current target</span><strong class="targetValue">—</strong></div></div>
    <div class="positionEditors"><div class="positionRow"><header><strong>↑ Up (Fully Open)</strong><span class="kind upKind green">Calibrated</span></header><div class="value upValue">—</div><input class="upEdit" inputmode="numeric" aria-label="Up encoder position" hidden></div><div class="positionRow"><header><strong>Ⅱ Middle</strong><span class="kind middleKind blue">Calculated</span></header><div class="value middleValue">—</div><input class="middleEdit" inputmode="numeric" aria-label="Middle encoder position" hidden></div><div class="positionRow"><header><strong>↓ Down (Fully Closed)</strong><span class="kind downKind green">Calibrated</span></header><div class="value downValue">—</div><input class="downEdit" inputmode="numeric" aria-label="Down encoder position" hidden></div>${button(id,'Save Positions','save-positions','class="savePositions primary" hidden')}</div>
    <div class="panelSub">Motor Telemetry</div><div class="detailGrid"><div class="metric"><small>Current</small><strong class="currentAmps">—</strong></div><div class="metric"><small>Voltage</small><strong class="voltage">—</strong></div><div class="metric"><small>Output speed</small><strong class="speed">—</strong></div><div class="metric"><small>Load output</small><strong class="load">—</strong></div><div class="metric"><small>Temperature</small><strong class="temperature">—</strong></div><div class="metric"><small>Servo status</small><strong class="servoStatus">—</strong></div></div>
    <div class="legend"><span class="key"><i class="swatch" style="border-color:#75e5a1"></i>Calibrated / Manual</span><span class="key"><i class="swatch" style="border-color:#5aaaff"></i>Calculated</span><span class="key"><i class="swatch" style="border-color:#ff6b72"></i>Servo zero</span></div>
  </div></div>
  <details class="details"><summary>Motor diagnostics and settings</summary><div class="detailGrid">
    <div class="metric"><small>Moving</small><strong class="moving">—</strong></div><div class="metric"><small>Torque enabled</small><strong class="torqueEnabled">—</strong></div>
  </div><div class="settings">
    <label>Speed limit <input class="speedLimit" type="number" min="0" max="360" step="1"> °/s</label>${button(id,'Save','save-speed','class="small"')}
    <label>Acceleration <input class="acceleration" type="number" min="0" max="254" step="1"></label>${button(id,'Save','save-accel','class="small"')}
    <label>Torque limit <input class="torqueLimit" type="number" min="0" max="100" step="1"> %</label>${button(id,'Save','save-torque','class="small"')}
  </div><div class="note">Cover tilt: <span class="tiltValue">—</span></div><div class="tiltChoices">${[0,25,50,75,100].map(p=>button(id,`${p}%`,`tilt-${p}`,'class="small"')).join('')}${button(id,'Stop','stop','class="small"')}</div></details>`;
  root.append(article);return article}
function num(v,unit='',digits=1){return Number.isFinite(v)?`${v.toFixed(digits)}${unit}`:'—'}
function render(s){let card=$(`#blind-${s.id}`)||makeCard(s);last[s.id]=s;const set=(cls,val)=>card.querySelector('.'+cls).textContent=val;
  const valid=Number.isFinite(s.position), full=(s.mask&5)===5&&s.down!==s.up;
  const scale=full?{down:s.down,up:s.up}:(()=>{const pts=[0,s.position,s.target,...[[1,s.down],[2,s.middle],[4,s.up]].filter(([bit])=>s.mask&bit).map(([,v])=>v)].filter(Number.isFinite);let lo=Math.min(...pts),hi=Math.max(...pts);if(hi-lo<4096){const mid=(lo+hi)/2;lo=mid-2048;hi=mid+2048}return s.negative_is_down?{down:lo,up:hi}:{down:hi,up:lo}})();
  const pct=v=>Math.max(0,Math.min(1,(v-scale.down)/(scale.up-scale.down)));const markers=[];
  const add=(v,label,color,side='left',extra='')=>{if(!Number.isFinite(v))return;const top=(100-pct(v)*100).toFixed(2);markers.push(`<div class="mark ${color} ${extra}" style="top:${top}%"></div><span class="markLabel ${side} ${color}" style="top:${top}%">${label}</span>`)};
  if(s.mask&4)add(s.up,'UP','green','left',s.up===0?'striped':'');if(s.mask&2)add(s.middle,'MID',s.middle_calculated?'blue':'green','right');if(s.mask&1)add(s.down,'DOWN','green','left',s.down===0?'striped':'');
  if(!((s.mask&4)&&s.up===0)&&!((s.mask&1)&&s.down===0))add(0,'ZERO','red','left');if(valid)add(s.position,'','current');if(s.moving&&Number.isFinite(s.target))add(s.target,'TARGET','blue','right','target');
  card.querySelector('.markers').innerHTML=markers.join('');card.querySelector('.fill').style.height=`${valid?(pct(s.position)*100).toFixed(2):0}%`;
  set('currentValue',valid?`${fmt(s.position)} counts${full&&(s.position<Math.min(s.down,s.up)||s.position>Math.max(s.down,s.up))?' · capped':''}`:'Unavailable');
  set('targetValue',`${fmt(s.target)} counts`);
  const badge=card.querySelector('.status');badge.textContent=s.status;badge.className='badge status '+(s.status==='Ok'?'ok':s.status==='Error'?'warn':'');
  card.querySelectorAll('.set,.arrow:not(.stopButton)').forEach(el=>el.disabled=!s.manual_control||s.auto_active);card.querySelector('.stopButton').disabled=!s.moving&&!s.auto_active;
  card.querySelector('.manualToggle').checked=s.manual_control;card.querySelector('.manualToggle').disabled=s.auto_active;
  card.querySelector('.editToggle').checked=s.edit_positions;card.querySelector('.editToggle').disabled=s.auto_active;
  card.querySelector('.auto').disabled=s.calibrated||s.auto_active;
  card.querySelector('.directionSelect').disabled=(s.calibrated&&!s.manual_control)||s.auto_active;card.querySelector('.directionSelect').value=s.negative_is_down?'Negative is down':'Negative is up';
  for(const [key,bit] of [['up',4],['middle',2],['down',1]]){const present=!!(s.mask&bit),kind=card.querySelector('.'+key+'Kind'),isCalculated=key==='middle'&&s.middle_calculated;kind.textContent=present?(s.positions_manual?'Manual':isCalculated?'Calculated':'Calibrated'):'Unset';kind.className=`kind ${key}Kind ${isCalculated?'blue':'green'}`;set(key+'Value',present?`${fmt(s[key])} counts`:'—');const input=card.querySelector('.'+key+'Edit');input.hidden=!s.edit_positions;if(document.activeElement!==input)input.value=present?s[key]:''}card.querySelector('.savePositions').hidden=!s.edit_positions;
  const notEditing=cls=>document.activeElement!==card.querySelector('.'+cls);if(notEditing('jog'))card.querySelector('.jog').value=s.jog;
  for(const [cls,value] of [['speedLimit',s.speed_limit],['acceleration',s.acceleration],['torqueLimit',s.torque_limit]])if(notEditing(cls))card.querySelector('.'+cls).value=value;
  set('speed',num(s.speed,' °/s'));set('load',num(s.load,' %'));set('voltage',num(s.voltage,' V'));set('currentAmps',num(s.current,' A',3));
  set('temperature',num(s.temperature,' °C',0));set('servoStatus',Number.isFinite(s.servo_status)?fmt(s.servo_status):'—');
  set('moving',s.moving?'Yes':'No');set('torqueEnabled',s.torque_enabled?'Yes':'No');set('tiltValue',num(s.tilt*100,' %',0));
}
function renderAll(d){const all=$('#all');if(!Number.isFinite(d.all_tilt))return;all.hidden=false;
 all.innerHTML=`<div class="cardhead"><div><div class="eyebrow">Group control</div><h2>All Blinds</h2></div><span class="badge">${Math.round(d.all_tilt*100)}% tilt</span></div><div class="groupControls"><span class="muted">Move all to</span>${[0,25,50,75,100].map(p=>button(0,`${p}%`,`tilt-${p}`,'class="small"')).join('')}${button(0,'Stop','stop','class="small"')}</div>`}
async function poll(){if(busy)return;busy=true;try{const response=await fetch('/sts3215/state',{cache:'no-store'});if(!response.ok)throw Error('Device did not return state');const data=await response.json();data.servos.forEach(render);renderAll(data);const conn=$('#connection');conn.textContent='Live';conn.className='connection live'}catch(e){const conn=$('#connection');conn.textContent='Reconnecting…';conn.className='connection'}finally{busy=false}}
async function post(url){const response=await fetch(url,{method:'POST'});if(!response.ok)throw Error(`Command failed (${response.status})`);setTimeout(poll,200)}
document.addEventListener('click',async e=>{const b=e.target.closest('button[data-action]');if(!b)return;const id=Number(b.dataset.id),action=b.dataset.action,s=last[id],prefix=`Blind ${id}`;
 try{if(action==='reset'){if(!confirm(`Reset calibration for ${prefix}? Its saved positions will be cleared.`))return;await post(`/button/${esc(prefix+' Reset Calibration')}/press`)}
 else if(action==='auto')await post(`/button/${esc(prefix+' Auto Calibrate')}/press`);
 else if(action==='jog-up'||action==='jog-down'){const forward=action==='jog-up'?s.negative_is_down:!s.negative_is_down;await post(`/button/${esc(prefix+(forward?' Jog Forward':' Jog Reverse'))}/press`)}
 else if(action.startsWith('set-')){const suffix={'set-up':' Set Fully Up','set-middle':' Set Middle','set-down':' Set Fully Down'}[action];await post(`/button/${esc(prefix+suffix)}/press`)}
 else if(action==='save-jog'||action==='save-speed'||action==='save-accel'||action==='save-torque'){const card=$(`#blind-${id}`);const item={'save-jog':['jog',' Jog Increment'],'save-speed':['speedLimit',' Speed Limit'],'save-accel':['acceleration',' Acceleration'],'save-torque':['torqueLimit',' Torque Limit']}[action];const value=Number(card.querySelector('.'+item[0]).value);if(!Number.isFinite(value))throw Error('Enter a valid number');await post(`/number/${esc(prefix+item[1])}/set?value=${encodeURIComponent(value)}`)}
 else if(action.startsWith('tilt-'))await post(`/cover/${esc(id?prefix:'All Blinds')}/set?tilt=${Number(action.slice(5))/100}`);
 else if(action==='stop')await post(`/cover/${esc(id?prefix:'All Blinds')}/stop`);
 else if(action==='save-positions'){const card=$(`#blind-${id}`),values={};for(const key of ['down','middle','up']){const raw=card.querySelector('.'+key+'Edit').value.trim();if(!/^[+-]?\d+$/.test(raw))throw Error('Positions must be whole numbers');const n=Number(raw);if(!Number.isSafeInteger(n)||n< -2147483648||n>2147483647)throw Error('Position is outside the supported range');values[key]=n}if(!((values.down<values.middle&&values.middle<values.up)||(values.down>values.middle&&values.middle>values.up)))throw Error('Middle must be between down and up');await control(id,'positions',values);toast('Manual positions saved')}
 }catch(error){toast(error.message)}});
async function control(id,action,values){const body=new URLSearchParams({id:String(id),action,...Object.fromEntries(Object.entries(values).map(([k,v])=>[k,String(v)]))});const response=await fetch('/sts3215/control',{method:'POST',body});if(!response.ok)throw Error(await response.text());await poll()}
document.addEventListener('change',async e=>{const el=e.target,id=Number(el.dataset.id);try{if(el.matches('.directionSelect'))await post(`/select/${esc(`Blind ${id} Negative Direction`)}/set?option=${esc(el.value)}`);else if(el.matches('.manualToggle'))await control(id,'manual',{value:el.checked?1:0});else if(el.matches('.editToggle'))await control(id,'edit',{value:el.checked?1:0})}catch(error){toast(error.message);poll()}});
poll();setInterval(poll,1000);
</script></body></html>)stsweb";
