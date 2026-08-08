"""
Quest Orientation Visualizer
----------------------------
An interactive browser-based view of the live controller orientation. This node
subscribes to the Quest pose/inputs/twist topics and serves a self-contained web
page that draws each controller's orientation as a 3D axis triad in real time.

Key Features:
- **Live 3D Orientation**: Renders the quaternion as a rotated axis triad plus a
  controller body, updated as fast as the stream arrives (~72 Hz from the app).
- **Readouts**: Raw quaternion, derived roll/pitch/yaw, position, linear/angular
  speed, quaternion norm (a norm far from 1.0 means the decode is wrong), and the
  measured per-topic publish rate.
- **Interactive**: Drag to orbit the camera, scroll to zoom, pick which hand(s) to
  show, freeze the stream, and re-zero so the current pose becomes the reference.
- **No extra dependencies**: stdlib http.server + Server-Sent Events only; no
  websockets, no CDN, no build step.

How to Run:
    $ ros2 run q2r2_bringup OrientationViz
    then open the URL it prints (default http://0.0.0.0:8080 -> use your host IP).

Options:
    # Serve on a different port / interface:
    $ ros2 run q2r2_bringup OrientationViz --ros-args -p port:=9000 -p host:=127.0.0.1
"""
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

# Import message types
from geometry_msgs.msg import PoseStamped, Twist
from quest2ros.msg import OVR2ROSInputs

HANDS = ("left", "right")


def _quat_to_euler(x, y, z, w):
    """Return (roll, pitch, yaw) in degrees from a quaternion."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    # clamp to keep asin in domain when the quaternion is slightly denormalized
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class _RateMeter:
    """Sliding-window publish-rate estimator."""

    def __init__(self, window=40):
        self.window = window
        self.stamps = []

    def tick(self, now):
        self.stamps.append(now)
        if len(self.stamps) > self.window:
            self.stamps.pop(0)

    def hz(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        if span <= 0.0:
            return 0.0
        return (len(self.stamps) - 1) / span


class OrientationViz(Node):
    def __init__(self):
        super().__init__("quest_orientation_viz")

        self.declare_parameter("port", 8080)
        self.declare_parameter("host", "0.0.0.0")
        self.port = int(self.get_parameter("port").value)
        self.host = str(self.get_parameter("host").value)

        self._lock = threading.Lock()
        self._state = {}
        self._rates = {}
        for hand in HANDS:
            self._state[hand] = {
                "pose": None,
                "inputs": None,
                "twist": None,
                "last_seen": 0.0,
            }
            self._rates[hand] = {
                "pose": _RateMeter(),
                "inputs": _RateMeter(),
                "twist": _RateMeter(),
            }

            self.create_subscription(
                PoseStamped,
                f"/q2r_{hand}_hand_pose",
                lambda msg, h=hand: self._on_pose(h, msg),
                10,
            )
            self.create_subscription(
                OVR2ROSInputs,
                f"/q2r_{hand}_hand_inputs",
                lambda msg, h=hand: self._on_inputs(h, msg),
                10,
            )
            self.create_subscription(
                Twist,
                f"/q2r_{hand}_hand_twist",
                lambda msg, h=hand: self._on_twist(h, msg),
                10,
            )

        self._start_server()

    # ---- subscriptions -------------------------------------------------

    def _on_pose(self, hand, msg):
        p, o = msg.pose.position, msg.pose.orientation
        norm = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
        roll, pitch, yaw = _quat_to_euler(o.x, o.y, o.z, o.w)
        now = time.time()
        with self._lock:
            self._state[hand]["pose"] = {
                "pos": [p.x, p.y, p.z],
                "quat": [o.x, o.y, o.z, o.w],
                "norm": norm,
                "euler": [roll, pitch, yaw],
            }
            self._state[hand]["last_seen"] = now
            self._rates[hand]["pose"].tick(now)

    def _on_inputs(self, hand, msg):
        now = time.time()
        with self._lock:
            self._state[hand]["inputs"] = {
                "button_upper": bool(msg.button_upper),
                "button_lower": bool(msg.button_lower),
                "thumb": [msg.thumb_stick_horizontal, msg.thumb_stick_vertical],
                "press_index": msg.press_index,
                "press_middle": msg.press_middle,
            }
            self._state[hand]["last_seen"] = now
            self._rates[hand]["inputs"].tick(now)

    def _on_twist(self, hand, msg):
        now = time.time()
        lin, ang = msg.linear, msg.angular
        with self._lock:
            self._state[hand]["twist"] = {
                "lin": [lin.x, lin.y, lin.z],
                "ang": [ang.x, ang.y, ang.z],
                "lin_speed": math.sqrt(lin.x**2 + lin.y**2 + lin.z**2),
                "ang_speed": math.sqrt(ang.x**2 + ang.y**2 + ang.z**2),
            }
            self._state[hand]["last_seen"] = now
            self._rates[hand]["twist"].tick(now)

    def snapshot(self):
        now = time.time()
        with self._lock:
            out = {"t": now, "hands": {}}
            for hand in HANDS:
                st = self._state[hand]
                age = now - st["last_seen"] if st["last_seen"] else None
                out["hands"][hand] = {
                    "pose": st["pose"],
                    "inputs": st["inputs"],
                    "twist": st["twist"],
                    "stale": age is None or age > 1.0,
                    "age": age,
                    "hz": {k: round(v.hz(), 1) for k, v in self._rates[hand].items()},
                }
            return out

    # ---- http server ---------------------------------------------------

    def _start_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *args):
                pass  # keep the ROS console clean

            def do_GET(self):
                if self.path.startswith("/stream"):
                    self._serve_stream()
                elif self.path in ("/", "/index.html"):
                    self._serve_page()
                else:
                    self.send_error(404)

            def _serve_page(self):
                body = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                try:
                    while True:
                        payload = json.dumps(node.snapshot())
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        time.sleep(1.0 / 60.0)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # browser tab closed

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._httpd.daemon_threads = True
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

        shown = "localhost" if self.host in ("0.0.0.0", "") else self.host
        self.get_logger().info("--- Quest Orientation Visualizer ---")
        self.get_logger().info(f"Open http://{shown}:{self.port} in a browser")

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        return super().destroy_node()


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Quest Orientation</title>
<style>
  :root { color-scheme: dark; --bg:#0e1116; --panel:#161b22; --line:#2a313c;
          --fg:#e6edf3; --dim:#8b949e; --lx:#ff6b6b; --ly:#51cf66; --lz:#4dabf7; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg); font:14px/1.5
         ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
  header { padding:12px 18px; border-bottom:1px solid var(--line);
           display:flex; gap:16px; align-items:center; flex-wrap:wrap; }
  h1 { font-size:15px; margin:0; font-weight:600; letter-spacing:.02em; }
  .status { font-size:12px; color:var(--dim); }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%;
         background:#f85149; margin-right:6px; vertical-align:1px; }
  .dot.live { background:#3fb950; }
  .controls { margin-left:auto; display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
  button { background:var(--panel); color:var(--fg); border:1px solid var(--line);
           border-radius:6px; padding:6px 12px; font-size:13px; cursor:pointer; }
  button:hover { border-color:#3d4653; }
  button.on { background:#1f6feb; border-color:#1f6feb; }
  main { display:grid; grid-template-columns:1fr 340px; gap:0; height:calc(100vh - 54px); }
  @media (max-width:860px) { main { grid-template-columns:1fr; height:auto; } }
  #stage { position:relative; min-height:420px; }
  canvas { display:block; width:100%; height:100%; touch-action:none; cursor:grab; }
  canvas:active { cursor:grabbing; }
  .hint { position:absolute; left:14px; bottom:12px; font-size:11px; color:var(--dim); }
  aside { border-left:1px solid var(--line); padding:16px; overflow-y:auto; }
  @media (max-width:860px) { aside { border-left:0; border-top:1px solid var(--line); } }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:8px;
          padding:12px 14px; margin-bottom:12px; }
  .card h2 { font-size:12px; margin:0 0 10px; text-transform:uppercase;
             letter-spacing:.08em; color:var(--dim); font-weight:600;
             display:flex; justify-content:space-between; align-items:center; }
  .kv { display:grid; grid-template-columns:auto 1fr; gap:3px 12px;
        font-variant-numeric:tabular-nums; font-size:13px; }
  .kv span:nth-child(odd) { color:var(--dim); }
  .kv span:nth-child(even) { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .bar { height:5px; background:#21262d; border-radius:3px; overflow:hidden; margin-top:3px; }
  .bar > i { display:block; height:100%; background:#1f6feb; width:0; }
  .badge { font-size:10px; padding:2px 7px; border-radius:10px; background:#21262d;
           color:var(--dim); font-weight:500; letter-spacing:.03em; }
  .badge.live { background:#132d1a; color:#3fb950; }
  .btn-state { display:flex; gap:6px; margin-top:8px; }
  .btn-state div { flex:1; text-align:center; font-size:11px; padding:4px;
                   border-radius:5px; background:#21262d; color:var(--dim); }
  .btn-state div.hot { background:#1f6feb; color:#fff; }
  .warn { color:#d29922; font-size:12px; margin-top:8px; }
</style></head><body>
<header>
  <h1>Quest Controller Orientation</h1>
  <span class="status"><span class="dot" id="dot"></span><span id="conn">connecting…</span></span>
  <div class="controls">
    <button id="b-left"  class="on">Left</button>
    <button id="b-right" class="on">Right</button>
    <button id="b-trail" class="on">Trail</button>
    <button id="b-freeze">Freeze</button>
    <button id="b-zero">Re-zero</button>
    <button id="b-reset">Reset view</button>
  </div>
</header>
<main>
  <div id="stage">
    <canvas id="cv"></canvas>
    <div class="hint">drag to orbit · scroll to zoom · axes: <b style="color:var(--lx)">X</b>
      <b style="color:var(--ly)">Y</b> <b style="color:var(--lz)">Z</b></div>
  </div>
  <aside id="panel"></aside>
</main>
<script>
const HANDS = ["left","right"];
const show = {left:true, right:true};
let trail = true, freeze = false;
let cam = {yaw:-0.6, pitch:-0.35, zoom:1};
let zeroQ = {left:null, right:null};
let latest = null, trails = {left:[], right:[]};

const cv = document.getElementById("cv"), ctx = cv.getContext("2d");
function fit(){ const r = cv.getBoundingClientRect(), d = window.devicePixelRatio||1;
  cv.width = r.width*d; cv.height = r.height*d; ctx.setTransform(d,0,0,d,0,0); }
window.addEventListener("resize", fit); fit();

// ---- quaternion helpers ----
function qmul(a,b){ return [
  a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
  a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
  a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
  a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]]; }
function qconj(q){ return [-q[0],-q[1],-q[2],q[3]]; }
function qrot(q,v){
  const t = qmul(qmul(q,[v[0],v[1],v[2],0]), qconj(q));
  return [t[0],t[1],t[2]]; }

// ---- camera projection ----
function project(p){
  const cy=Math.cos(cam.yaw), sy=Math.sin(cam.yaw);
  const cp=Math.cos(cam.pitch), sp=Math.sin(cam.pitch);
  let x = p[0]*cy - p[2]*sy;
  let z = p[0]*sy + p[2]*cy;
  let y = p[1]*cp - z*sp;
  z     = p[1]*sp + z*cp;
  const r = cv.getBoundingClientRect();
  const s = Math.min(r.width, r.height)*0.30*cam.zoom;
  return [r.width/2 + x*s, r.height/2 - y*s, z];
}
function line(a,b,color,w){
  const A=project(a), B=project(b);
  ctx.strokeStyle=color; ctx.lineWidth=w||2;
  ctx.beginPath(); ctx.moveTo(A[0],A[1]); ctx.lineTo(B[0],B[1]); ctx.stroke();
}

function drawGrid(){
  const n=4, s=0.5;
  for(let i=-n;i<=n;i++){
    const a = i===0 ? "#39414d" : "#232a34";
    line([i*s,-0.001,-n*s],[i*s,-0.001,n*s],a,1);
    line([-n*s,-0.001,i*s],[n*s,-0.001,i*s],a,1);
  }
}

function drawTriad(q, origin, scale, label, alpha){
  const ax=[[1,0,0],[0,1,0],[0,0,1]];
  const cols=["#ff6b6b","#51cf66","#4dabf7"], names=["X","Y","Z"];
  // sort by depth so nearer axes draw on top
  const segs = ax.map((a,i)=>{
    const e = qrot(q, [a[0]*scale, a[1]*scale, a[2]*scale]);
    const tip=[origin[0]+e[0], origin[1]+e[1], origin[2]+e[2]];
    return {tip, col:cols[i], name:names[i], depth:project(tip)[2]};
  }).sort((p,r)=>p.depth-r.depth);

  ctx.globalAlpha = alpha;
  // controller body: a small box aligned to the local frame
  const hs=[0.055,0.035,0.10];
  const corners=[];
  for(const sx of [-1,1]) for(const sy of [-1,1]) for(const sz of [-1,1]){
    const e=qrot(q,[sx*hs[0]*scale*2, sy*hs[1]*scale*2, sz*hs[2]*scale*2]);
    corners.push([origin[0]+e[0], origin[1]+e[1], origin[2]+e[2]]);
  }
  const edges=[[0,1],[0,2],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]];
  for(const [i,j] of edges) line(corners[i],corners[j],"#4d5764",1.5);

  for(const s of segs){
    line(origin, s.tip, s.col, 3.5);
    const P=project(s.tip);
    ctx.fillStyle=s.col; ctx.beginPath(); ctx.arc(P[0],P[1],4,0,7); ctx.fill();
    ctx.font="600 11px ui-sans-serif,system-ui"; ctx.fillText(s.name, P[0]+7, P[1]-6);
  }
  if(label){
    const O=project(origin);
    ctx.fillStyle="#8b949e"; ctx.font="600 11px ui-sans-serif,system-ui";
    ctx.fillText(label, O[0]+8, O[1]+16);
  }
  ctx.globalAlpha = 1;
}

function effQuat(hand, pose){
  let q = pose.quat;
  if(zeroQ[hand]) q = qmul(qconj(zeroQ[hand]), q);
  return q;
}

function render(){
  const r = cv.getBoundingClientRect();
  ctx.clearRect(0,0,r.width,r.height);
  drawGrid();
  if(latest){
    const origins = {left:[-0.45,0.25,0], right:[0.45,0.25,0]};
    for(const h of HANDS){
      if(!show[h]) continue;
      const d = latest.hands[h]; if(!d || !d.pose) continue;
      const q = effQuat(h, d.pose);
      if(trail){
        const t = trails[h];
        t.push(qrot(q,[0,0,0.30]));
        if(t.length>90) t.shift();
        ctx.strokeStyle = h==="left" ? "#7c4dff55" : "#00b8d455";
        ctx.lineWidth=2; ctx.beginPath();
        t.forEach((p,i)=>{ const P=project([origins[h][0]+p[0],origins[h][1]+p[1],origins[h][2]+p[2]]);
          i?ctx.lineTo(P[0],P[1]):ctx.moveTo(P[0],P[1]); });
        ctx.stroke();
      }
      drawTriad(q, origins[h], 0.30, h.toUpperCase(), d.stale?0.35:1);
    }
  }
  requestAnimationFrame(render);
}
requestAnimationFrame(render);

// ---- readouts ----
const f=(v,n=3)=>(v===undefined||v===null)?"—":v.toFixed(n);
function panel(){
  if(!latest){ document.getElementById("panel").innerHTML=""; return; }
  let html="";
  for(const h of HANDS){
    const d=latest.hands[h]; if(!d) continue;
    const p=d.pose, tw=d.twist, inp=d.inputs;
    const live = !d.stale;
    html += `<div class="card"><h2>${h} hand
      <span class="badge ${live?'live':''}">${live?f(d.hz.pose,0)+' Hz':'no data'}</span></h2>`;
    if(p){
      const q=effQuat(h,p);
      const nOK = Math.abs(p.norm-1)<1e-4;
      html += `<div class="kv">
        <span>quat x</span><span>${f(q[0],4)}</span>
        <span>quat y</span><span>${f(q[1],4)}</span>
        <span>quat z</span><span>${f(q[2],4)}</span>
        <span>quat w</span><span>${f(q[3],4)}</span>
        <span>norm</span><span style="color:${nOK?'#3fb950':'#f85149'}">${f(p.norm,6)}</span>
        <span>roll</span><span>${f(p.euler[0],1)}°</span>
        <span>pitch</span><span>${f(p.euler[1],1)}°</span>
        <span>yaw</span><span>${f(p.euler[2],1)}°</span>
        <span>pos</span><span>${f(p.pos[0],2)}, ${f(p.pos[1],2)}, ${f(p.pos[2],2)}</span>
      </div>`;
      if(!nOK) html += `<div class="warn">norm ≠ 1 — decode may be wrong</div>`;
    } else { html += `<div class="kv"><span>pose</span><span>—</span></div>`; }
    if(tw){
      html += `<div class="kv" style="margin-top:8px">
        <span>lin speed</span><span>${f(tw.lin_speed,3)} m/s</span>
        <span>ang speed</span><span>${f(tw.ang_speed,3)} rad/s</span></div>`;
    }
    if(inp){
      html += `<div class="btn-state">
        <div class="${inp.button_upper?'hot':''}">upper</div>
        <div class="${inp.button_lower?'hot':''}">lower</div></div>
        <div class="kv" style="margin-top:8px">
        <span>index</span><span>${f(inp.press_index,2)}</span>
        <span>middle</span><span>${f(inp.press_middle,2)}</span>
        <span>thumb</span><span>${f(inp.thumb[0],2)}, ${f(inp.thumb[1],2)}</span></div>
        <div class="bar"><i style="width:${Math.round(Math.max(0,Math.min(1,inp.press_index))*100)}%"></i></div>`;
    }
    html += `</div>`;
  }
  document.getElementById("panel").innerHTML=html;
}
setInterval(panel, 100);

// ---- stream ----
const es = new EventSource("/stream");
es.onmessage = e => {
  if(freeze) return;
  latest = JSON.parse(e.data);
  const any = HANDS.some(h => latest.hands[h] && !latest.hands[h].stale);
  document.getElementById("dot").className = "dot" + (any?" live":"");
  document.getElementById("conn").textContent = any ? "streaming" : "connected — waiting for controller data";
};
es.onerror = () => {
  document.getElementById("dot").className="dot";
  document.getElementById("conn").textContent="stream lost — is the node still running?";
};

// ---- interaction ----
let drag=null;
cv.addEventListener("pointerdown", e=>{ drag={x:e.clientX,y:e.clientY}; cv.setPointerCapture(e.pointerId); });
cv.addEventListener("pointermove", e=>{ if(!drag) return;
  cam.yaw += (e.clientX-drag.x)*0.008;
  cam.pitch = Math.max(-1.5, Math.min(1.5, cam.pitch + (e.clientY-drag.y)*0.008));
  drag={x:e.clientX,y:e.clientY}; });
cv.addEventListener("pointerup", ()=>drag=null);
cv.addEventListener("wheel", e=>{ e.preventDefault();
  cam.zoom = Math.max(0.35, Math.min(3.5, cam.zoom * (e.deltaY>0?0.92:1.08))); }, {passive:false});

const tgl=(id,fn)=>document.getElementById(id).addEventListener("click", e=>fn(e.target));
tgl("b-left",  b=>{ show.left =!show.left;  b.classList.toggle("on"); trails.left=[]; });
tgl("b-right", b=>{ show.right=!show.right; b.classList.toggle("on"); trails.right=[]; });
tgl("b-trail", b=>{ trail=!trail; b.classList.toggle("on"); trails={left:[],right:[]}; });
tgl("b-freeze",b=>{ freeze=!freeze; b.classList.toggle("on");
  b.textContent = freeze?"Resume":"Freeze"; });
tgl("b-zero",  ()=>{ for(const h of HANDS){
  const d=latest&&latest.hands[h]; zeroQ[h] = (d&&d.pose)?d.pose.quat.slice():null; }
  trails={left:[],right:[]}; });
tgl("b-reset", ()=>{ cam={yaw:-0.6,pitch:-0.35,zoom:1}; zeroQ={left:null,right:null};
  trails={left:[],right:[]}; });
</script></body></html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = OrientationViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
