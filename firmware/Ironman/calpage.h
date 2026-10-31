// The /cal page (2026-10-13, the deadline day - Toby: "make sure everything is ready to just be shoved in"): set on the
// arm, with no recompile, what can only be found on the real arm - each servo's closed / open end, the PCA9685 channel
// map, the battery reading against a meter. Everything it sets is kept in NVS. Served on the feed port: :8080/cal
#pragma once
static const char CAL_HTML[] = R"HTML(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Gauntlet setup</title>
<style>
body{font:15px system-ui,Segoe UI,sans-serif;background:#12171c;color:#e6eaee;margin:0;padding:14px}
h1{font-size:20px;margin:0 0 4px}h2{font-size:15px;margin:18px 0 6px;color:#9aa6b2;text-transform:uppercase;letter-spacing:.06em}
.note{color:#9aa6b2;font-size:13px}.bad{color:#ff8a80}.ok{color:#8be28b}
table{border-collapse:collapse;width:100%}td,th{padding:6px 6px;border-bottom:1px solid #34404b;text-align:left;vertical-align:middle}
th{font-size:12px;color:#9aa6b2;font-weight:700}
button{font:inherit;font-size:13px;font-weight:700;background:#252e37;color:#e6eaee;border:2px solid #34404b;border-radius:8px;padding:5px 9px;margin:2px;cursor:pointer}
button:hover{border-color:#9aa6b2}button.go{border-color:#2a9d3f}button.warn{border-color:#d62828}
input{font:inherit;background:#1c232b;color:#e6eaee;border:2px solid #34404b;border-radius:8px;padding:5px 8px;width:9em}
#msg{position:sticky;top:0;background:#1c232b;border:2px solid #34404b;border-radius:10px;padding:8px 10px;margin:8px 0;min-height:1.3em}
.held{background:#25303a}.us{font-variant-numeric:tabular-nums;font-weight:800}
</style></head><body>
<h1>Gauntlet setup</h1>
<div class="note">Everything here is saved on the arm (NVS) at once and survives a power cycle - no recompile. Servo moves need the mode switch on <b>SHOW</b> (or Force SHOW below). A held servo lets go after 15 s.</div>
<div id="msg">-</div>
<div>Mode: <b id="mode">?</b> <button onclick="go('/mode?force=show')">Force SHOW</button><button onclick="go('/mode?force=switch')">Follow the switch</button>
<button class="warn" onclick="cal('release=1')">Release the servo</button></div>
<div>Plates: <b id="po">?</b> <button onclick="cal('plates=off')">Plates OFF (nothing moves)</button><button onclick="cal('plates=on')">Plates ON</button> <span class="note">OFF: no plate moves - not the cascade, not a gesture, not the lasers' or the camera's plates (they act with the plates where they are); this page still moves one servo.</span></div>
<h2>Servo ends - one servo at a time, on its real plate</h2>
<div><label><input type="checkbox" id="pr" style="width:auto"> <b>Move the pair together</b></label> <span class="note">plate 3 and the hand plate have two servos on one plate: ticked, a move takes its partner along on the same pulse (until both have saved ends, then each by its own ends) and Save saves both - so one never drags the other through the plate.</span></div>
<div class="note">Nudge it until the plate sits closed, press <b>Save as CLOSED</b>; then nudge to open, <b>Save as OPEN</b>. Then <b>Test plate</b> moves the whole plate (both servos of a pair) with the saved ends. Go slowly near a stop - a servo pushing a hard stop strips its gears.</div>
<table><thead><tr><th>Servo</th><th>PCA</th><th>Now</th><th>Move</th><th>Closed</th><th>Open</th><th>Plate</th></tr></thead><tbody id="sv"></tbody></table>
<h2>PCA9685 channels - what each output is wired to</h2>
<div class="note">If you wired something to a different PCA output than the design, enter <b>channel:output</b> pairs, e.g. <b>6:13</b> (the green laser's channel 6 is wired to output 13). Each pair is a swap, so the map stays one to one. Only with everything off.</div>
<div><input id="pm" placeholder="6:13, 9:6"><button class="go" onclick="pmap()">Set</button><button onclick="if(confirm('Every channel back on its own output?'))go('/pcamap?set=reset')">Reset to the design</button></div>
<table><thead><tr><th>Channel</th><th>What</th><th>Wired to PCA output</th></tr></thead><tbody id="pmap"></tbody></table>
<h2>Battery - against your meter</h2>
<div>The arm reads <b class="us" id="bv">?</b> V (factor <span id="bk">?</span>). Meter reads <input id="mv" placeholder="7.62"> V <button class="go" onclick="meter()">Set</button><button onclick="go('/power?meter=0')">Forget</button></div>
<div class="note">Measure the pack (or the battery + bus) with the meter while the arm is on, type the number, Set. It scales the reading to match.</div>
<script>
var S=null;
function $(i){return document.getElementById(i)}
function show(j){if(j&&j.msg)$('msg').textContent=j.msg;}
function go(u){fetch(u).then(function(r){return r.json()}).then(function(j){show(j);load()}).catch(function(e){$('msg').textContent='no answer: '+e});}
function cal(a){if($('pr')&&$('pr').checked){if(/(^|&)(us|step)=/.test(a))a+='&pair=1';if(/(^|&)save=/.test(a))a+='&both=1';}go('/servocal?'+a)}
function pmap(){var v=$('pm').value.replace(/\s+/g,'');if(v)go('/pcamap?set='+v)}
function meter(){var v=parseFloat($('mv').value);if(v>0)go('/power?meter='+v.toFixed(3))}
function btn(t,a,c){return '<button'+(c?' class="'+c+'"':'')+' onclick="cal(\''+a+'\')">'+t+'</button>'}
function load(){
 fetch('/servocal').then(function(r){return r.json()}).then(function(j){S=j;$('po').textContent=j.platesoff?'OFF':'ON';$('mode').textContent=j.mode+(j.canmove?'':' - servo moves refused: '+j.why);
  var h='';j.servos.forEach(function(s,i){var a='s='+i;
   h+='<tr'+(j.held==i?' class="held"':'')+'><td><b>'+s.name+'</b></td><td>'+s.ch+(s.pca!=s.ch?' &rarr; '+s.pca:'')+'</td><td class="us">'+(s.now?s.now+' us':(s.last?'('+s.last+')':'-'))+'</td><td>'
   +btn('&minus;50',a+'&step=-50')+btn('&minus;10',a+'&step=-10')+btn('+10',a+'&step=10')+btn('+50',a+'&step=50')+'</td>'
   +'<td><span class="us">'+s.closed+'</span>'+(s.sc?' <span class="ok">saved</span>':' <span class="note">'+s.src+'</span>')+'<br>'+btn('Go',a+'&us='+s.closed)+btn('Save as CLOSED',a+'&save=closed','go')+'</td>'
   +'<td><span class="us">'+s.open+'</span>'+(s.so?' <span class="ok">saved</span>':' <span class="note">'+s.src+'</span>')+'<br>'+btn('Go',a+'&us='+s.open)+btn('Save as OPEN',a+'&save=open','go')+'</td>'
   +'<td>'+btn('Test open',a+'&test=open')+btn('Test close',a+'&test=close')+(s.sc||s.so?btn('Forget',a+'&clear=1','warn'):'')+'</td></tr>';});
  $('sv').innerHTML=h;}).catch(function(){});
 fetch('/pcamap').then(function(r){return r.json()}).then(function(j){var h='';j.map.forEach(function(p,i){h+='<tr><td>'+i+'</td><td>'+(j.names[i]||'spare')+'</td><td'+(p!=i?' class="bad"':'')+'><b>'+p+'</b>'+(p!=i?' (moved)':'')+'</td></tr>'});$('pmap').innerHTML=h;}).catch(function(){});
 fetch('/power').then(function(r){return r.json()}).then(function(j){$('bv').textContent=j.v.toFixed(2);$('bk').textContent=j.k.toFixed(4)+(j.k==1?' - none':'');}).catch(function(){});
}
load();setInterval(load,1500);
</script></body></html>)HTML";
