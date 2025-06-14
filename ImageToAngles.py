#!/usr/bin/env python3
"""
@file        ImageToAngles.py
@brief       Convert a black-on-white PNG drawing to SCARA waypoints.

Every foreground (“ink”) pixel in the skeletonized PNG becomes one waypoint,
visited in depth-first order along the pixel-adjacency graph.  The result is:

1. **(x, y, z) path**  
   *z = 1* marks a pen-up jump between disconnected components.

2. **(θ₁, θ₂, z) moves**  
   Inverse-kinematics converts Cartesian waypoints to joint angles, ready to
   stream straight to the robot.

@par Example
@code
python ImageToAngles.py shark.png               # full resolution
python ImageToAngles.py shark.png --px-skip 3   # coarser skip
python ImageToAngles.py shark.png --min-dist 0.1  # ≥0.1 in apart
@endcode
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import List, Sequence, Tuple

import cv2
import numpy as np
from skimage.morphology import skeletonize

# ─────────────────────────── Robot constants ────────────────────────────
#: SCARA link-lengths (inches)
L1, L2 = 5.816, 5.931
#: θ₂ = 0 ° offset (mechanical zero is 147.5 ° above +X)
OFFSET_T2_DEG = 147.5
#: Workspace bounds used to scale/translate drawing (inches)
X_RANGE = (-6, -2)
Y_RANGE = (5.5, 9)


# ────────────────── 1. PNG → skeleton → graph-walk points ───────────────
def parse_png(path: str | Path) -> List[Tuple[float, float, int]]:
    """
    @brief  Skeletonize the PNG and traverse its pixel graph.

    @param path  Path to a *black-on-white* PNG image.
    @return      Ordered list of **(x, y, z)** waypoints in **pixel units**  
                 where *z = 1* designates a pen lift between disconnected
                 strokes.

    @throw FileNotFoundError   If *path* cannot be opened.
    """
    # -- load & binarize --------------------------------------------------
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not open image: {path}")

    _, bw = cv2.threshold(
        img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )  # foreground → 255, background → 0

    bw_bool = bw > 0
    skel_bool = skeletonize(bw_bool)  # 1-pixel-wide skeleton

    # -- build adjacency graph -------------------------------------------
    h, w = img.shape
    ys, xs = np.nonzero(skel_bool)
    pixels = set(zip(xs.tolist(), ys.tolist()))
    if not pixels:
        return []

    nbrs: dict[Tuple[int, int], list[Tuple[int, int]]] = {p: [] for p in pixels}
    for x, y in pixels:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nb = (x + dx, y + dy)
                if nb in pixels:
                    nbrs[(x, y)].append(nb)

    # -- pick a starting pixel (degree 1 endpoint if available) ----------
    endpoints = [p for p, ngh in nbrs.items() if len(ngh) == 1]
    start = endpoints[0] if endpoints else next(iter(pixels))

    # -- depth-first traversal (visit each pixel exactly once) -----------
    visited = {start}
    stack = [start]
    path: list[Tuple[int, int]] = []

    while stack:
        p = stack.pop()
        path.append(p)
        for q in nbrs[p]:
            if q not in visited:
                visited.add(q)
                stack.append(q)

    # -- convert pixel coords → (x, y, z) waypoints ---------------------
    pts: List[Tuple[float, float, int]] = []
    prev = path[0]
    pts.append((float(prev[0]), float(h - 1 - prev[1]), 0))  # first point

    for x, y in path[1:]:
        if (x, y) not in nbrs[prev]:
            # discontinuity → pen up then down
            pts.append((float(prev[0]), float(h - 1 - prev[1]), 1))
            pts.append((float(x), float(h - 1 - y), 0))
        else:
            pts.append((float(x), float(h - 1 - y), 0))
        prev = (x, y)

    return pts


# ───────────────────── 2. Normalize to workspace ────────────────────────
def normalize(pts: Sequence[Tuple[float, float, int]]) -> List[Tuple[float, float, int]]:
    """
    @brief  Scale and center pixel waypoints into the SCARA workspace.

    The drawing is **uniformly** scaled to fit inside *X_RANGE × Y_RANGE*
    while preserving aspect ratio, and then translated so its centroid lies
    at the center of that rectangle.

    @param pts  Waypoints from :pyfunc:`parse_png` in pixel units.
    @return     Same list of points, but in **inch units** relative to the
                robot base.
    """
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]

    sx = (X_RANGE[1] - X_RANGE[0]) / (max(xs) - min(xs))
    sy = (Y_RANGE[1] - Y_RANGE[0]) / (max(ys) - min(ys))
    s = min(sx, sy)  # uniform scale

    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    tx, ty = (X_RANGE[0] + X_RANGE[1]) / 2, (Y_RANGE[0] + Y_RANGE[1]) / 2

    out: List[Tuple[float, float, int]] = []
    for x, y, z in pts:
        out.append(((x - cx) * s + tx, (y - cy) * s + ty, z))
    return out


# ──────────────────── 3. Distance-based down-sampling ───────────────────
def downsample_by_dist(
    pts: Sequence[Tuple[float, float, int]], min_dist: float
) -> List[Tuple[float, float, int]]:
    """
    @brief  Remove redundant waypoints that are closer than *min_dist*.

    @param pts       Input waypoints (inch units).
    @param min_dist  Minimum allowable spacing (inches).  
                     Points with *z = 1* (pen-up) are **never** removed.
    @return          Filtered list with fewer than or equal to *len(pts)*
                     elements.
    """
    if min_dist <= 0 or not pts:
        return list(pts)

    out = [pts[0]]
    last_x, last_y, _ = pts[0]

    for x, y, z in pts[1:]:
        dx, dy = x - last_x, y - last_y
        if math.hypot(dx, dy) >= min_dist or z == 1:
            out.append((x, y, z))
            last_x, last_y = x, y
    return out


# ────────────────────── 4. Inverse kinematics ───────────────────────────
def ik_xy(x: float, y: float) -> Tuple[float, float]:
    """
    @brief  Two-link planar inverse kinematics.

    @param x  X-coordinate in inches.
    @param y  Y-coordinate in inches.
    @return   (θ₁, θ₂) in **degrees**.  Both angles are made negative so that
              positive robot motion matches the SCARA’s CCW convention.

    @throw ValueError  If *(x, y)* lies outside the reachable workspace.
    """
    r2 = x * x + y * y
    c = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c) > 1:
        raise ValueError(f"Point ({x:.2f}, {y:.2f}) out of reach")

    s = math.sqrt(1 - c * c)
    t2 = math.atan2(s, c) - math.radians(OFFSET_T2_DEG)

    k1, k2 = L1 + L2 * c, L2 * s
    t1 = math.atan2(y, x) - math.atan2(k2, k1)

    return -abs(math.degrees(t1)), -abs(math.degrees(t2))


def ik_path(pts: Sequence[Tuple[float, float, int]]) -> List[Tuple[float, float]]:
    """
    @brief  Vectorized helper: apply :pyfunc:`ik_xy` to an entire path.

    @param pts  Waypoints **after** normalization/down-sampling.
    @return     List of *(θ₁, θ₂)* pairs in degrees.
    """
    return [ik_xy(x, y) for x, y, _ in pts]


# ───────────────────── 5. Command-line interface ────────────────────────
def main() -> None:
    """
    @brief  Parse CLI arguments, run full pipeline, and write *image_thetas.txt*.

    The output file contains one line per waypoint in the format  
    `{θ₁, θ₂, z},` – ready for the robot firmware.
    """
    p = argparse.ArgumentParser(
        description="Convert PNG → SCARA waypoints via graph-walk skeleton"
    )
    p.add_argument("png", help="black-on-white PNG input")
    p.add_argument(
        "-o", "--out", default="image_thetas.txt", help="output {θ₁, θ₂, z} file"
    )
    p.add_argument(
        "--px-skip",
        type=int,
        default=1,
        help="keep only every N-th waypoint in traversal order",
    )
    p.add_argument(
        "--min-dist",
        type=float,
        default=0.0,
        help="minimum Cartesian spacing between waypoints (inches)",
    )
    args = p.parse_args()

    # ---------------- Pipeline ------------------------------------------
    pts = parse_png(Path(args.png))
    norm_pts = normalize(pts)

    if args.px_skip > 1:
        norm_pts = [pt for i, pt in enumerate(norm_pts) if i % args.px_skip == 0]
    if args.min_dist > 0:
        norm_pts = downsample_by_dist(norm_pts, args.min_dist)

    angs = ik_path(norm_pts)

    # ---------------- Export --------------------------------------------
    with open(args.out, "w", encoding="utf-8") as fh:
        for (t1, t2), (_, _, z) in zip(angs, norm_pts):
            fh.write(f"{{{t1:.2f}, {t2:.2f}, {z}}},\n")

    print(f"{len(angs)} waypoints → {args.out}")


# ────────────────────────────── Entry-point ─────────────────────────────
if __name__ == "__main__":
    main()
