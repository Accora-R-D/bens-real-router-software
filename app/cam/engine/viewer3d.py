"""Generate a standalone interactive 3D material-removal viewer (HTML + three.js).

The sheet is a height-field that the real Ø6mm tool carves in real time along the
program toolpath. Browser controls: orbit/zoom/pan, play/pause, speed slider,
scrub bar, reset.

Run:  python viewer3d.py [path-to.NC]   ->  out/viewer3d_<n>.html
"""
import sys, math, json
from pathlib import Path
from simulate import parse_segments
from cam_syntec import R, OUT

RES3D = 1.5      # height-field cell size (mm) — finer = smoother edges, heavier
DEC = 0.25       # toolpath point decimation distance (mm)
DEC_Z = 0.02     # ...or this much change in height. At 0.1 a ball-nose surface
                 # reached the viewer flattened into 0.1 mm steps: grainy.


def decimate(segs):
    """Ordered points [x,y,z,cutflag,tool] where cutflag and tool belong to the
    move INTO that point."""
    pts = []
    if not segs:
        return pts
    s0 = segs[0]
    pts.append([round(s0["x0"], 3), round(s0["y0"], 3), round(s0["z0"], 3), 0, s0.get("t", 0)])
    lk = pts[0]
    for s in segs:
        cut = 0 if s["kind"] == "rapid" else 1
        t = s.get("t", 0)
        x, y, z = s["x1"], s["y1"], s["z1"]
        d = math.hypot(x-lk[0], y-lk[1])
        if (d >= DEC or cut != lk[3] or t != lk[4] or abs(z-lk[2]) > DEC_Z
                or s is segs[-1]):
            p = [round(x, 3), round(y, 3), round(z, 3), cut, t]
            pts.append(p); lk = p
    return pts


HTML = r"""<!doctype html><html><head><meta charset="utf-8">
<title>CNC 3D Material Sim</title>
<style>
 html,body{margin:0;height:100%;background:#1b1d22;color:#ddd;font:13px system-ui,Arial}
 #c{position:fixed;inset:0}
 #ui{position:fixed;left:12px;top:12px;background:#23262dcc;padding:12px 14px;border-radius:8px;
     box-shadow:0 2px 10px #0008;width:330px}
 #ui h1{font-size:14px;margin:0 0 8px;color:#00c2a0}
 .row{display:flex;align-items:center;gap:8px;margin:7px 0}
 .row label{width:54px;color:#9aa}
 input[type=range]{flex:1}
 button{background:#00836a;color:#fff;border:0;border-radius:5px;padding:6px 12px;cursor:pointer}
 button:hover{background:#00a085}
 #val{color:#00c2a0;width:42px;text-align:right}
 #prog{color:#9aa;font-size:12px}
 .leg{font-size:11px;color:#889;margin-top:8px;line-height:1.5}
</style></head><body>
<div id="c"></div>
<div id="err" style="position:fixed;inset:0;display:none;align-items:center;justify-content:center;
  background:#1b1d22;color:#ff8080;font:14px monospace;padding:40px;white-space:pre-wrap"></div>
<div id="ui">
 <h1>Material Removal — Sheet __IDX__ (Ø__DIA__mm tool)</h1>
 <div class="row"><button id="play">▶ Play</button><button id="reset">⟲ Reset</button>
   <span id="prog">0%</span></div>
 <div class="row"><label>Speed</label><input id="speed" type="range" min="0.1" max="10" step="0.1" value="2">
   <span id="val">2.0×</span></div>
 <div class="row"><label>Scrub</label><input id="scrub" type="range" min="0" max="1000" step="1" value="0"></div>
 <div class="leg">Drag = orbit · scroll = zoom · right-drag = pan<br>
   colour = remaining height (light = full board, dark = cut deeper, grey = cut through)</div>
</div>
<script>
function fail(m){var e=document.getElementById('err');e.style.display='flex';e.textContent='3D viewer error:\n\n'+m;}
window.onerror=function(msg,src,l,c){fail(msg+'\n('+src+':'+l+')');};
</script>
<script src="./lib/three.min.js"></script>
<script src="./lib/OrbitControls.js"></script>
<script>
if(typeof THREE==='undefined') fail('three.js failed to load from ./lib/three.min.js — check the lib folder sits next to this HTML.');
else if(typeof THREE.OrbitControls==='undefined') fail('OrbitControls failed to load from ./lib/OrbitControls.js');
(function(){
const GRID = __GRID__;
const PATH = __PATH__;          // {px,py,pz,cut}
const R = __R__, top = GRID.top, res = GRID.res;
const gw = GRID.gw, gh = GRID.gh, ox = GRID.ox, oy = GRID.oy;
const W = gw*res, H = gh*res, cx = ox+W/2, cy = oy+H/2;

const scene = new THREE.Scene(); scene.background = new THREE.Color(0x2b2f36);
const cam = new THREE.PerspectiveCamera(50, innerWidth/innerHeight, 1, 100000);
cam.up.set(0,0,1);
cam.position.set(cx + W*0.25, cy - H*1.0, top + Math.max(W,H)*0.85);
const rnd = new THREE.WebGLRenderer({antialias:true});
rnd.setSize(innerWidth, innerHeight); document.getElementById('c').appendChild(rnd.domElement);
const ctr = new THREE.OrbitControls(cam, rnd.domElement); ctr.target.set(cx, cy, 0); ctr.update();

// ---- height-field ----
const N = gw*gh;
const heights = new Float32Array(N).fill(top);
const pos = new Float32Array(N*3), col = new Float32Array(N*3);
function colorFor(h, out, o){
  if(h<=0.1){ out[o]=0.34;out[o+1]=0.36;out[o+2]=0.40; return; } // cut through -> dark slate (groove)
  const t=Math.max(0,Math.min(1,h/top));
  // bright wood tan at full height, darkening smoothly into the cut
  out[o]=0.42+0.48*t; out[o+1]=0.30+0.36*t; out[o+2]=0.18+0.16*t;
}
for(let iy=0;iy<gh;iy++)for(let ix=0;ix<gw;ix++){
  const i=iy*gw+ix, o=i*3;
  pos[o]=ox+ix*res; pos[o+1]=oy+iy*res; pos[o+2]=top;
  colorFor(top,col,o);
}
const idx=[]; for(let iy=0;iy<gh-1;iy++)for(let ix=0;ix<gw-1;ix++){
  const a=iy*gw+ix,b=a+1,c=a+gw,d=c+1; idx.push(a,b,c, b,d,c);
}
const geo=new THREE.BufferGeometry();
geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
geo.setAttribute('color',new THREE.BufferAttribute(col,3));
geo.setIndex(idx);
const mesh=new THREE.Mesh(geo,new THREE.MeshBasicMaterial({vertexColors:true, side:THREE.DoubleSide}));
scene.add(mesh);
// sheet outline
const eg=new THREE.EdgesGeometry(new THREE.BoxGeometry(W,H,0.1));
const out=new THREE.LineSegments(eg,new THREE.LineBasicMaterial({color:0x444}));
out.position.set(cx,cy,0); scene.add(out);

// ---- tool ----
const tg=new THREE.CylinderGeometry(R,R,30,28); tg.rotateX(Math.PI/2);
const tool=new THREE.Mesh(tg,new THREE.MeshBasicMaterial({color:0x33aaff}));
tool.visible=false; scene.add(tool);

// ---- carving ----
const rr=Math.ceil(R/res);
function stamp(wx,wy,z){
  const ci=(wx-ox)/res, cj=(wy-oy)/res;
  const i0=Math.max(0,Math.floor(ci-rr)), i1=Math.min(gw-1,Math.ceil(ci+rr));
  const j0=Math.max(0,Math.floor(cj-rr)), j1=Math.min(gh-1,Math.ceil(cj+rr));
  for(let jy=j0;jy<=j1;jy++)for(let ix=i0;ix<=i1;ix++){
    const dx=(ix-ci), dy=(jy-cj); if(dx*dx+dy*dy>(R/res)*(R/res)) continue;
    const i=jy*gw+ix; if(heights[i]>z){ heights[i]=z; pos[i*3+2]=z; colorFor(z,col,i*3); }
  }
}
function carveSeg(ax,ay,bx,by,z){
  const len=Math.hypot(bx-ax,by-ay); const n=Math.max(1,Math.ceil(len/(res*0.5)));
  for(let k=0;k<=n;k++){ const t=k/n; stamp(ax+(bx-ax)*t, ay+(by-ay)*t, z); }
}
const NP=PATH.px.length;
let cur=0;                          // current point index (float)
function carveTo(target){
  const from=Math.max(1,Math.floor(cur)+1), to=Math.floor(target);
  for(let i=from;i<=to && i<NP;i++){
    if(PATH.cut[i]){
      carveSeg(PATH.px[i-1],PATH.py[i-1],PATH.px[i],PATH.py[i],Math.min(PATH.pz[i-1],PATH.pz[i]));
    }
  }
  cur=target;
  if(to>=1 && to<NP){ tool.position.set(PATH.px[to],PATH.py[to],PATH.pz[to]+15); }
  geo.attributes.position.needsUpdate=true; geo.attributes.color.needsUpdate=true;
}
function resetAll(){
  for(let iy=0;iy<gh;iy++)for(let ix=0;ix<gw;ix++){const i=iy*gw+ix;heights[i]=top;pos[i*3+2]=top;colorFor(top,col,i*3);}
  cur=0; geo.attributes.position.needsUpdate=true; geo.attributes.color.needsUpdate=true;
}
function setIndex(target){ if(target<cur) resetAll(); carveTo(target); }

// ---- UI ----
let playing=false, speed=2.0;
const RATE=NP/30;                   // points/sec at 1x  (~30s full run)
const playB=document.getElementById('play'), resetB=document.getElementById('reset');
const speedS=document.getElementById('speed'), valE=document.getElementById('val');
const scrubS=document.getElementById('scrub'), progE=document.getElementById('prog');
playB.onclick=()=>{ playing=!playing; tool.visible=true; playB.textContent=playing?'❚❚ Pause':'▶ Play'; };
resetB.onclick=()=>{ playing=false; playB.textContent='▶ Play'; tool.visible=false; resetAll(); scrubS.value=0; progE.textContent='0%'; };
speedS.oninput=()=>{ speed=parseFloat(speedS.value); valE.textContent=speed.toFixed(1)+'×'; };
scrubS.oninput=()=>{ playing=false; playB.textContent='▶ Play'; tool.visible=true;
  setIndex(parseFloat(scrubS.value)/1000*NP); progE.textContent=Math.round(parseFloat(scrubS.value)/10)+'%'; };

const clock=new THREE.Clock();
function loop(){
  requestAnimationFrame(loop);
  const dt=clock.getDelta();
  if(playing){
    let t=cur+RATE*speed*dt;
    if(t>=NP){ t=NP; playing=false; playB.textContent='▶ Play'; }
    carveTo(t);
    scrubS.value=Math.round(t/NP*1000); progE.textContent=Math.round(t/NP*100)+'%';
  }
  ctr.update(); rnd.render(scene,cam);
}
loop();
addEventListener('resize',()=>{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();rnd.setSize(innerWidth,innerHeight);});
})();
</script></body></html>"""


def run(nc):
    segs = parse_segments(nc.read_text())
    mf = nc.with_suffix(".json")
    manifest = json.loads(mf.read_text()) if mf.exists() else {}
    idx = manifest.get("sheet", 1)
    top = manifest.get("thickness_mm", 18.0)

    pts = decimate(segs)
    cutpts = [(s["x0"], s["y0"]) for s in segs if s["kind"] != "rapid"] + \
             [(s["x1"], s["y1"]) for s in segs if s["kind"] != "rapid"]
    pad = R + 8
    minx = min(p[0] for p in cutpts)-pad; maxx = max(p[0] for p in cutpts)+pad
    miny = min(p[1] for p in cutpts)-pad; maxy = max(p[1] for p in cutpts)+pad
    gw = int(math.ceil((maxx-minx)/RES3D)); gh = int(math.ceil((maxy-miny)/RES3D))

    grid = {"gw": gw, "gh": gh, "res": RES3D, "ox": round(minx, 2), "oy": round(miny, 2),
            "top": round(top, 2)}
    path = {"px": [p[0] for p in pts], "py": [p[1] for p in pts],
            "pz": [p[2] for p in pts], "cut": [p[3] for p in pts]}

    html = (HTML.replace("__GRID__", json.dumps(grid))
                .replace("__PATH__", json.dumps(path))
                .replace("__R__", str(R)).replace("__DIA__", str(2*R))
                .replace("__IDX__", str(idx)))
    fn = OUT / f"viewer3d_{idx}.html"
    fn.write_text(html, encoding="utf-8")
    print(f"  {nc.name}: {len(pts)} path pts (from {len(segs)} moves), grid {gw}x{gh} @ {RES3D}mm")
    print(f"  3D viewer -> {fn}")
    return fn


if __name__ == "__main__":
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else OUT / "SHEET_18mm.NC"
    run(target)
