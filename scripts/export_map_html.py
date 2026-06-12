#!/usr/bin/env python3
"""Export a PCD map as an interactive browser viewer (drag to orbit, wheel to
zoom, right-drag to pan).

Writes a folder containing index.html (Three.js) + data.bin (downsampled,
pre-colored points + optional trajectory polyline). View it with any static
file server — browsers block fetch() from file://, so:

    python3 scripts/export_map_html.py --map run_5_map.pcd --out map_web \
        --traj run_5_dump/traj_imu.txt
    python3 -m http.server 8000 -d map_web
    # open http://localhost:8000

The folder is self-contained except for the Three.js CDN import (internet
needed on first load). Coloring matches scripts/render_map.py (height x
intensity). Point count is capped (--max-points, default 8M) by voxel
downsampling so the browser stays responsive.

Deps: numpy, scipy, matplotlib (same as render_map.py).
"""

import argparse
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_map import load_pcd, clean, colorize  # noqa: E402

HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>DLIO++ map viewer</title>
<style>
  body { margin:0; overflow:hidden; background:#0b0b0f; font-family:sans-serif; }
  #hud { position:fixed; top:10px; left:10px; color:#bbb; font-size:13px;
         background:rgba(0,0,0,.45); padding:8px 12px; border-radius:6px; }
  #hud input { vertical-align:middle; }
</style>
<script type="importmap">
{ "imports": {
    "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
    "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
} }
</script>
</head>
<body>
<div id="hud">
  drag: rotate &nbsp;|&nbsp; wheel: zoom &nbsp;|&nbsp; right-drag: pan<br/>
  point size <input id="psize" type="range" min="0.5" max="4" step="0.1" value="1.5"/>
  <button id="topview">top view</button>
  <span id="stats"></span>
</div>
<script type="module">
const statsEl = () => document.getElementById('stats');
addEventListener('error', e => { statsEl().textContent = ' | ERROR: ' + e.message; });
addEventListener('unhandledrejection', e => { statsEl().textContent = ' | ERROR: ' + e.reason; });

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(devicePixelRatio);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0b0f);
const camera = new THREE.PerspectiveCamera(55, innerWidth/innerHeight, 0.5, 50000);
camera.up.set(0,0,1);                       // z-up map frame; must precede OrbitControls
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

statsEl().textContent = ' | loading point cloud ...';
const buf = await (await fetch('data.bin')).arrayBuffer();
const dv = new DataView(buf);
let off = 0;
const n = dv.getUint32(off, true); off += 4;
const pos = new Float32Array(buf, off, n*3); off += n*12;
const col = new Uint8Array(buf, off, n*3); off += n*3;
const tn = dv.getUint32(off, true); off += 4;
// slice(): the offset here is not 4-byte aligned, so a direct
// Float32Array view on `buf` would throw a RangeError
const tpos = new Float32Array(buf.slice(off, off + tn*12));

const geo = new THREE.BufferGeometry();
geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
geo.setAttribute('color', new THREE.BufferAttribute(col, 3, true));
const mat = new THREE.PointsMaterial({size:1.5, vertexColors:true, sizeAttenuation:false});
scene.add(new THREE.Points(geo, mat));

if (tn > 0) {
  const tgeo = new THREE.BufferGeometry();
  tgeo.setAttribute('position', new THREE.BufferAttribute(tpos, 3));
  scene.add(new THREE.Line(tgeo, new THREE.LineBasicMaterial({color:0xff7711})));
}

geo.computeBoundingBox();
const bb = geo.boundingBox, c = new THREE.Vector3();
bb.getCenter(c);
const ext = bb.getSize(new THREE.Vector3()).length();
function home() {
  camera.position.set(c.x - 0.45*ext, c.y - 0.45*ext, c.z + 0.35*ext);
  controls.target.copy(c); controls.update();
}
function top() {
  camera.position.set(c.x, c.y + 1, c.z + 0.9*ext);
  controls.target.copy(c); controls.update();
}
home();
document.getElementById('psize').oninput = e => { mat.size = parseFloat(e.target.value); };
document.getElementById('topview').onclick = top;
document.getElementById('stats').textContent = ` | ${n.toLocaleString()} pts`;
addEventListener('resize', () => {
  camera.aspect = innerWidth/innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script>
</body>
</html>
"""


def voxel_downsample(xyz, colors, max_points):
    if len(xyz) <= max_points:
        return xyz, colors
    res = 0.15
    for _ in range(12):
        key = np.floor(xyz / res).astype(np.int64)
        key = (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)
        _, idx = np.unique(key, return_index=True)
        if len(idx) <= max_points:
            print(f"[export_map_html] voxel {res:.2f} m -> {len(idx):,} points")
            return xyz[idx], colors[idx]
        res *= 1.35
    return xyz[idx], colors[idx]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", required=True, help="output folder (index.html + data.bin)")
    ap.add_argument("--traj", default="", help="optional TUM trajectory overlay")
    ap.add_argument("--max-points", type=int, default=8_000_000)
    ap.add_argument("--cmap", default="turbo")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    print(f"[export_map_html] loading {args.map} ...")
    xyz, inten = load_pcd(args.map)
    print(f"[export_map_html] {len(xyz):,} points")
    xyz, inten = clean(xyz, inten)
    colors = colorize(xyz, inten, args.cmap)
    xyz, colors = voxel_downsample(xyz, colors, args.max_points)

    ctr = (xyz.min(0) + xyz.max(0)) / 2.0
    pos = (xyz - ctr).astype(np.float32)
    col = (np.clip(colors, 0, 1) * 255).astype(np.uint8)

    traj = np.zeros((0, 3), np.float32)
    if args.traj:
        traj = (np.loadtxt(args.traj)[:, 1:4] - ctr + [0, 0, 1.5]).astype(np.float32)

    with open(os.path.join(args.out, "data.bin"), "wb") as f:
        f.write(struct.pack("<I", len(pos)))
        f.write(pos.tobytes())
        f.write(col.tobytes())
        f.write(struct.pack("<I", len(traj)))
        f.write(traj.tobytes())
    with open(os.path.join(args.out, "index.html"), "w") as f:
        f.write(HTML)

    size_mb = os.path.getsize(os.path.join(args.out, "data.bin")) / 1e6
    print(f"[export_map_html] wrote {args.out}/ (data.bin {size_mb:.0f} MB, {len(pos):,} pts)")
    print(f"[export_map_html] view with:  python3 -m http.server 8000 -d {args.out}")
    print("[export_map_html] then open   http://localhost:8000")


if __name__ == "__main__":
    main()
