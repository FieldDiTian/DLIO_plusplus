#!/usr/bin/env python3
"""Headless 3D renderer for the pipeline's PCD maps.

Produces clean perspective renders of a (binary, x/y/z/intensity float32) PCD
— the format written by `glim_dump_to_pcd` — without needing a GUI, GPU
stack, or heavyweight viewer. Points are splatted through a z-buffer,
colored by height blended with return intensity (lane paint / curbs show
up), and shaded with eye-dome lighting so the 3D structure reads clearly.
Optionally overlays the GLIM trajectory (TUM format, e.g. traj_imu.txt).

Examples:
    python3 scripts/render_map.py --map run_5_map.pcd --out renders/
    python3 scripts/render_map.py --map run_5_map.pcd --out renders/ \
        --traj run_5_dump/traj_imu.txt --views overview,low,top,start

Deps: numpy, scipy, matplotlib, pillow  (pip install)
"""

import argparse
import os
import re
import sys

import numpy as np
from PIL import Image
from matplotlib import colormaps
from scipy import ndimage


def load_pcd(path):
    """Minimal binary PCD reader for FIELDS x y z [intensity], TYPE F SIZE 4."""
    with open(path, "rb") as f:
        header = b""
        while not header.endswith(b"DATA binary\n"):
            line = f.readline()
            if not line:
                sys.exit("unsupported PCD (expected 'DATA binary')")
            header += line
        text = header.decode(errors="replace")
        fields = re.search(r"^FIELDS (.+)$", text, re.M).group(1).split()
        npts = int(re.search(r"^POINTS (\d+)$", text, re.M).group(1))
        data = np.fromfile(f, dtype=np.float32, count=npts * len(fields))
    data = data.reshape(-1, len(fields))
    xyz = data[:, [fields.index("x"), fields.index("y"), fields.index("z")]]
    inten = data[:, fields.index("intensity")] if "intensity" in fields else np.zeros(len(data), np.float32)
    return xyz, inten


def clean(xyz, inten, voxel=1.0, min_pts=3):
    """Drop stray speckle: keep points whose 1 m voxel holds >= min_pts points."""
    lo, hi = np.percentile(xyz[:, 2], [0.2, 99.8])
    m = (xyz[:, 2] >= lo) & (xyz[:, 2] <= hi)
    xyz, inten = xyz[m], inten[m]
    key = np.floor(xyz / voxel).astype(np.int64)
    key = (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)
    order = np.argsort(key)
    sk = key[order]
    counts = np.diff(np.flatnonzero(np.concatenate(([True], sk[1:] != sk[:-1], [True]))))
    percount = np.repeat(counts, counts)
    keep = np.empty(len(key), bool)
    keep[order] = percount >= min_pts
    return xyz[keep], inten[keep]


def colorize(xyz, inten, cmap_name="turbo"):
    z = xyz[:, 2]
    # p85 cap: tall structures (trees/fences) saturate the top of the map and
    # would otherwise compress the mostly-flat track surface into one hue
    zlo, zhi = np.percentile(z, [2, 85])
    zn = np.clip((z - zlo) / max(zhi - zlo, 1e-6), 0, 1)
    ilo, ihi = np.percentile(inten, [5, 97])
    inorm = np.clip((inten - ilo) / max(ihi - ilo, 1e-6), 0, 1)
    base = colormaps[cmap_name](zn)[:, :3].astype(np.float32)
    lum = (0.45 + 0.55 * inorm[:, None]).astype(np.float32)
    return np.clip(base * lum, 0, 1)


def look_at(eye, target, up=(0, 0, 1)):
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    dn = np.cross(fwd, right)
    R = np.stack([right, dn, fwd])  # world -> camera
    return R, eye


def render(xyz, colors, R, eye, w, h, fov_deg, traj=None, ss=2):
    """Painter-ordered point splat + eye-dome lighting. Returns HxWx3 uint8."""
    W, H = w * ss, h * ss
    f = 0.5 * W / np.tan(np.radians(fov_deg) / 2)

    def project(pts):
        cam = (pts - eye) @ R.T
        zc = cam[:, 2]
        m = zc > 1.0
        u = f * cam[m, 0] / zc[m] + W / 2
        v = f * cam[m, 1] / zc[m] + H / 2
        inb = (u >= 2) & (u < W - 2) & (v >= 2) & (v < H - 2)
        return u[inb].astype(np.int32), v[inb].astype(np.int32), zc[m][inb], m, inb

    u, v, d, m, inb = project(xyz)
    c = colors[m][inb]
    order = np.argsort(-d)  # far first; near overwrites
    u, v, d, c = u[order], v[order], d[order], c[order]

    img = np.full((H, W, 3), np.array([0.045, 0.045, 0.06], np.float32))
    depth = np.full((H, W), np.inf, np.float32)
    for du in (0, 1):           # 2x2 splat fills pixel gaps
        for dv in (0, 1):
            img[v + dv, u + du] = c
            depth[v + dv, u + du] = d

    # eye-dome lighting on log depth
    logd = np.log(np.where(np.isfinite(depth), depth, 1e6))
    shade = np.zeros((H, W), np.float32)
    for ax, sh in ((0, 1), (0, -1), (1, 1), (1, -1)):
        shade += np.maximum(np.roll(logd, sh, axis=ax) - logd, 0)
    shade = np.exp(-12.0 * np.clip(shade, 0, 0.6))
    valid = np.isfinite(depth)
    img[valid] *= (0.35 + 0.65 * shade[valid])[:, None]

    if traj is not None:
        # densify the polyline, lift it slightly, draw AFTER shading with a
        # depth test against the map so it occludes correctly but stays bright
        seg = np.diff(traj, axis=0)
        steps = np.maximum((np.linalg.norm(seg, axis=1) / 0.3).astype(int), 1)
        dense = [traj[i] + seg[i] * np.linspace(0, 1, s, endpoint=False)[:, None]
                 for i, s in enumerate(steps)]
        tp = (np.concatenate(dense) + np.array([0, 0, 2.0])).astype(np.float32)
        tu, tv, td, _, _ = project(tp)
        col = np.array([1.0, 0.45, 0.0], np.float32)
        for du in (-1, 0, 1):   # 3x3 splat = thick visible ribbon
            for dv in (-1, 0, 1):
                vis = td <= depth[tv + dv, tu + du] + 3.0
                img[tv[vis] + dv, tu[vis] + du] = col

    img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
    out = Image.fromarray(img).resize((w, h), Image.LANCZOS)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", required=True, help="output directory for PNGs")
    ap.add_argument("--traj", default="", help="optional TUM trajectory (traj_imu.txt) overlay")
    ap.add_argument("--views", default="overview,low,top", help="comma list: overview,low,top,start")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--cmap", default="turbo")
    ap.add_argument("--no-clean", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    print(f"[render_map] loading {args.map} ...")
    xyz, inten = load_pcd(args.map)
    print(f"[render_map] {len(xyz):,} points")
    if not args.no_clean:
        xyz, inten = clean(xyz, inten)
        print(f"[render_map] {len(xyz):,} after speckle removal")
    colors = colorize(xyz, inten, args.cmap)

    traj = None
    if args.traj:
        traj = np.loadtxt(args.traj)[:, 1:4]
        print(f"[render_map] trajectory overlay: {len(traj)} poses")

    lo, hi = xyz.min(0), xyz.max(0)
    ctr = (lo + hi) / 2
    ext = float(np.linalg.norm((hi - lo)[:2]))

    def eye_at(az_deg, el_deg, dist, target):
        az, el = np.radians(az_deg), np.radians(el_deg)
        off = dist * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        return target + off

    views = {}
    views["overview"] = (eye_at(215, 38, 0.85 * ext, ctr), ctr, 50)
    views["low"] = (eye_at(120, 14, 0.55 * ext, ctr), ctr, 45)
    views["top"] = (ctr + np.array([0, 1, 1.35 * ext]), ctr, 42)
    if traj is not None:
        s = traj[0] + np.array([0, 0, 0])
        views["start"] = (eye_at(160, 22, 140.0, s), s, 55)

    for name in [v.strip() for v in args.views.split(",") if v.strip()]:
        if name not in views:
            print(f"[render_map] skipping unknown/unavailable view '{name}'")
            continue
        eye, target, fov = views[name]
        R, e = look_at(eye, target)
        print(f"[render_map] rendering '{name}' ...")
        im = render(xyz, colors, R, e, args.width, args.height, fov, traj=traj)
        path = os.path.join(args.out, f"map_{name}.png")
        im.save(path)
        print(f"[render_map] wrote {path}")


if __name__ == "__main__":
    main()
