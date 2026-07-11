#!/usr/bin/env python3
"""Export a GLIM dump directory to a binary PCD map.

This is a fallback for checkouts where the expected ``glim_dump_to_pcd`` ROS
executable is not installed. It reads GLIM submap compact point bins and
composes each submap's ``T_world_origin``.

[P2 FIX 2026-07-10] Frame correctness: GLIM's graph lives in its own WORLD
frame, related to the Atlas local-ENU frame by the ``T_world_utm`` alignment
the gnss_global module fits and saves into the dump. GICP localization
consumes Atlas ENU poses (odom-init seed, GT snap, cross-check) DIRECTLY as
map-frame poses, so the map it loads must be genuinely ENU. This exporter now
defaults to ``--frame enu``: it loads ``<dump>/T_world_utm.txt`` and applies
its inverse, so the written PCD is in the Atlas local-ENU frame. When
exporting an ENU map, leave GICP's ``localization/utm_transform_path`` EMPTY —
the old world-frame workflow only worked to the extent T_world_utm happened
to be near identity.

For the INS-driven local-ENU mapping profile, GLIM world is already the
adapter's local-ENU frame and no ``T_world_utm.txt`` exists. Use
``--frame local-enu`` in that case: it performs no numeric transform but
records the ENU datum and the correct map contract in the manifest.
"""

from __future__ import annotations

import argparse
import datetime
import math
import re
import shutil
import sys
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


def parse_matrix(lines: list[str], key: str) -> np.ndarray:
    for idx, line in enumerate(lines):
        if line.strip() == f"{key}:":
            rows = []
            for row in lines[idx + 1 : idx + 5]:
                values = [float(v) for v in row.strip().split()]
                if len(values) != 4:
                    raise ValueError(f"bad {key} matrix row: {row!r}")
                rows.append(values)
            return np.asarray(rows, dtype=np.float64)
    raise ValueError(f"{key} not found")


def load_world_utm(path: Path) -> np.ndarray:
    """Parse T_world_utm.txt (gnss_global's dump format) and validate it."""
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    mat = parse_matrix(lines, "T_world_utm")
    if not np.all(np.isfinite(mat)):
        raise ValueError(f"{path}: non-finite entries")
    if not np.allclose(mat[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{path}: bottom row is not [0 0 0 1]")
    R = mat[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
        raise ValueError(f"{path}: rotation block is not orthonormal")
    return mat


def invert_se3(mat: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = mat[:3, :3].T
    out[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return out


def submap_dirs(dump_dir: Path) -> list[Path]:
    dirs = []
    for path in dump_dir.iterdir():
        if path.is_dir() and re.fullmatch(r"\d+", path.name):
            if (path / "data.txt").is_file() and (path / "points_compact.bin").is_file():
                dirs.append(path)
    return sorted(dirs, key=lambda p: int(p.name))


def point_count(path: Path) -> int:
    size = path.stat().st_size
    if size % 12 != 0:
        raise ValueError(f"{path} size {size} is not divisible by 12")
    return size // 12


def read_submap(path: Path) -> tuple[np.ndarray, np.ndarray]:
    n = point_count(path / "points_compact.bin")
    points = np.fromfile(path / "points_compact.bin", dtype=np.float32).reshape(n, 3)
    intensity_path = path / "intensities_compact.bin"
    if intensity_path.is_file() and intensity_path.stat().st_size == n * 4:
        intensities = np.fromfile(intensity_path, dtype=np.float32)
    else:
        intensities = np.zeros(n, dtype=np.float32)
    return points, intensities


def transformed_chunks(
    dirs: Iterable[Path],
    voxel_size: float,
    stride: int,
    pre_transform: Optional[np.ndarray] = None,
) -> Iterable[np.ndarray]:
    seen: Optional[set[tuple[int, int, int]]] = set() if voxel_size > 0.0 else None
    for idx, path in enumerate(dirs, start=1):
        lines = (path / "data.txt").read_text(encoding="utf-8", errors="replace").splitlines()
        transform = parse_matrix(lines, "T_world_origin")
        if pre_transform is not None:
            # [P2 FIX 2026-07-10] Compose ONCE per submap: output frame =
            # pre_transform (T_utm_world) applied on top of T_world_origin,
            # i.e. points land in the Atlas local-ENU frame.
            transform = pre_transform @ transform
        points, intensities = read_submap(path)
        if stride > 1:
            points = points[::stride]
            intensities = intensities[::stride]
        if points.size == 0:
            continue

        world = points.astype(np.float64) @ transform[:3, :3].T + transform[:3, 3]
        if seen is not None:
            keys = np.floor(world / voxel_size).astype(np.int64)
            keep_indices = []
            for i, key in enumerate(keys):
                item = (int(key[0]), int(key[1]), int(key[2]))
                if item in seen:
                    continue
                seen.add(item)
                keep_indices.append(i)
            if not keep_indices:
                continue
            keep = np.asarray(keep_indices, dtype=np.int64)
            world = world[keep]
            intensities = intensities[keep]

        out = np.empty(world.shape[0], dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
        out["x"] = world[:, 0].astype(np.float32)
        out["y"] = world[:, 1].astype(np.float32)
        out["z"] = world[:, 2].astype(np.float32)
        out["intensity"] = intensities.astype(np.float32)
        print(f"[export_glim_dump_to_pcd] {idx}: {path.name} -> {len(out)} points", flush=True)
        yield out


def write_header(handle, count: int) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    handle.write(header.encode("ascii"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump_dir", type=Path)
    parser.add_argument("output_pcd", type=Path)
    parser.add_argument("--voxel-size", type=float, default=0.0)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument(
        "--frame",
        choices=("enu", "local-enu", "world"),
        default="enu",
        help="Output frame. 'enu' (default) applies inverse T_world_utm from the dump so the "
        "PCD matches the Atlas local-ENU poses GICP consumes directly; 'local-enu' keeps an "
        "INS-driven GLIM world unchanged because it is already adapter local-ENU; 'world' "
        "writes an otherwise unspecified GLIM world frame (legacy behavior).",
    )
    parser.add_argument(
        "--enu-origin",
        type=str,
        default="",
        help="ENU datum 'lat_deg,lon_deg,alt_m' used by the adapter/prep_bag for this dataset. "
        "REQUIRED for --frame enu/local-enu (recorded in the map manifest): a map and a live adapter "
        "using DIFFERENT origins are numerically valid but mutually incompatible, and nothing "
        "else ties the datum to the map.",
    )
    parser.add_argument(
        "--allow-missing-origin",
        action="store_true",
        help="Escape hatch for legacy dumps whose origin is unknown: export without recording "
        "a datum (the manifest will carry an explicit UNSPECIFIED warning).",
    )
    parser.add_argument(
        "--transform-file",
        type=Path,
        default=None,
        help="Override path to T_world_utm.txt (default: <dump_dir>/T_world_utm.txt)",
    )
    args = parser.parse_args()

    if args.voxel_size < 0.0 or not math.isfinite(args.voxel_size):
        parser.error("--voxel-size must be finite and >= 0")
    # [P3 FIX 2026-07-10] Provenance is ENFORCED, not just recorded: an ENU
    # map without its datum cannot be validated against the live adapter.
    if args.frame in ("enu", "local-enu") and not args.enu_origin and not args.allow_missing_origin:
        parser.error(
            f"--frame {args.frame} requires --enu-origin 'lat_deg,lon_deg,alt_m' (the adapter/prep_bag "
            "datum for this dataset). Use --allow-missing-origin ONLY for legacy dumps whose "
            "origin is unrecoverable."
        )
    if args.stride < 1:
        parser.error("--stride must be >= 1")

    dirs = submap_dirs(args.dump_dir)
    if not dirs:
        raise SystemExit(f"no GLIM submap dirs found under {args.dump_dir}")

    pre_transform: Optional[np.ndarray] = None
    if args.frame == "enu":
        tf_path = args.transform_file or (args.dump_dir / "T_world_utm.txt")
        if not tf_path.is_file():
            # Fail CLOSED: a silently world-framed "ENU" map poisons every
            # GT-anchored mechanism in GICP. A GNSS-less dump must be exported
            # with an explicit --frame world.
            raise SystemExit(
                f"--frame enu but {tf_path} does not exist (was the GNSS module enabled for this "
                "mapping run?). Re-run with --frame world ONLY if the map is genuinely meant to "
                "stay in GLIM's world frame."
            )
        T_world_utm = load_world_utm(tf_path)
        pre_transform = invert_se3(T_world_utm)
        yaw_deg = math.degrees(math.atan2(T_world_utm[1, 0], T_world_utm[0, 0]))
        print(
            f"[export_glim_dump_to_pcd] frame=enu: applying inverse T_world_utm from {tf_path} "
            f"(world-utm offset: t=[{T_world_utm[0,3]:.2f}, {T_world_utm[1,3]:.2f}, "
            f"{T_world_utm[2,3]:.2f}] m, yaw={yaw_deg:.2f} deg). Leave GICP's "
            "localization/utm_transform_path EMPTY for this map.",
            flush=True,
        )
    elif args.frame == "local-enu":
        print(
            "[export_glim_dump_to_pcd] frame=local-enu: keeping GLIM world unchanged because "
            "the INS mapping profile already uses the adapter local-ENU frame. Leave GICP's "
            "localization/utm_transform_path EMPTY for this map.",
            flush=True,
        )
    else:
        print(
            "[export_glim_dump_to_pcd] frame=world (legacy): PCD stays in GLIM's world frame — "
            "GICP's Atlas ENU seeds/GT will be frame-mismatched unless T_world_utm ~= identity.",
            flush=True,
        )

    args.output_pcd.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.output_pcd.with_suffix(args.output_pcd.suffix + ".tmp")
    data_tmp = args.output_pcd.with_suffix(args.output_pcd.suffix + ".data.tmp")

    # The PCD binary header must carry the final POINTS count, which is only
    # known after the whole dump is processed (voxel dedup / stride change it).
    # Rather than buffer every transformed chunk in RAM, stream each chunk to a
    # temp binary file as it is produced (one submap chunk resident at a time,
    # plus the voxel-dedup set when --voxel-size > 0), then prepend the header
    # and copy the data back out. Byte-identical output, bounded peak memory.
    total = 0
    success = False
    try:
        with data_tmp.open("wb") as data_handle:
            for chunk in transformed_chunks(dirs, args.voxel_size, args.stride, pre_transform):
                chunk.tofile(data_handle)
                total += len(chunk)

        if total == 0:
            raise SystemExit("export produced zero points")

        with tmp.open("wb") as handle:
            write_header(handle, total)
            with data_tmp.open("rb") as data_handle:
                shutil.copyfileobj(data_handle, handle, length=8 * 1024 * 1024)
        # [SELF-AUDIT FIX 2026-07-10] Manifest FIRST, then finalize the PCD:
        # provenance is mandatory now, so a manifest-write failure must not
        # leave a finished but unmanifested map behind.
        # [P2 FIX 2026-07-10b] Map manifest: record the frame and transform the
        # PCD was exported with, so the mapping->localization handoff is
        # auditable (reviewer requirement: the ENU conversion must be a
        # GUARANTEED pipeline step, not an operator convention).
        manifest = args.output_pcd.with_suffix(args.output_pcd.suffix + ".manifest.yaml")
        with manifest.open("w", encoding="utf-8") as mh:
            mh.write("# Map provenance manifest (written by export_glim_dump_to_pcd.py)\n")
            mh.write(f"exported_utc: {datetime.datetime.now(datetime.timezone.utc).isoformat()}\n")
            mh.write(f"source_dump: {args.dump_dir.resolve()}\n")
            mh.write(f"frame: {args.frame}\n")
            mh.write(f"points: {total}\n")
            mh.write(f"voxel_size: {args.voxel_size}\n")
            mh.write(f"stride: {args.stride}\n")
            if args.enu_origin:
                mh.write(f"enu_origin: {args.enu_origin}\n")
            else:
                mh.write("enu_origin: UNSPECIFIED  # WARNING: record the adapter's\n")
                mh.write("#   local_enu_origin for this dataset — a live adapter with a\n")
                mh.write("#   different datum is silently incompatible with this map\n")
            if pre_transform is not None:
                mh.write("applied_transform: inverse(T_world_utm)  # PCD is Atlas local-ENU\n")
                mh.write("T_world_utm:\n")
                for row in T_world_utm:
                    mh.write("  - [" + ", ".join(f"{v:.10f}" for v in row) + "]\n")
                mh.write("gicp_note: leave localization/utm_transform_path EMPTY for this map\n")
            elif args.frame == "local-enu":
                mh.write("applied_transform: none  # GLIM world already is adapter local-ENU\n")
                mh.write("gicp_note: leave localization/utm_transform_path EMPTY for this map\n")
            else:
                mh.write("applied_transform: none  # PCD is GLIM WORLD frame — NOT directly\n")
                mh.write("#   compatible with Atlas ENU seeds/GT in gicp localization\n")
        print(f"[export_glim_dump_to_pcd] manifest: {manifest}")
        tmp.replace(args.output_pcd)
        success = True
    finally:
        data_tmp.unlink(missing_ok=True)
        if not success:
            tmp.unlink(missing_ok=True)

    print(f"[export_glim_dump_to_pcd] wrote {total} points to {args.output_pcd}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
