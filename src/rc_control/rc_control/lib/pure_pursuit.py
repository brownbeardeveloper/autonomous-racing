"""Path and geometry helpers for pure pursuit.

No ROS import, so the maths can be tested without a workspace;
pure_pursuit_node.py is the ROS side.

The path is a closed loop of N points. Segment i runs from point i to point
(i + 1) % N, and a position on the path is a pair (i, t), with t in [0, 1)
along segment i.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np


@dataclass
class Path:
    """A closed racing line.

    xy     (N, 2) positions [m]
    v      (N,)   target speed at each point [m/s]
    kappa  (N,)   signed path curvature [1/m]; zeros if the source had none
    s      (N,)   cumulative arc length from point 0 [m]
    seg    (N,)   length of segment i (point i -> i+1, wrapping) [m]
    length        total loop length [m]
    """

    xy: np.ndarray
    v: np.ndarray
    kappa: np.ndarray
    s: np.ndarray
    seg: np.ndarray
    length: float

    def __len__(self) -> int:
        return len(self.xy)

    @classmethod
    def from_arrays(cls, xy, v=None, kappa=None) -> Path:
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        n = len(xy)
        if n < 3:
            raise ValueError(f'need at least 3 waypoints to form a loop, got {n}')

        # Drop a duplicated closing point - the loop is implicit.
        if np.linalg.norm(xy[0] - xy[-1]) < 1e-9:
            xy = xy[:-1]
            if v is not None:
                v = np.asarray(v, float)[:-1]
            if kappa is not None:
                kappa = np.asarray(kappa, float)[:-1]
            n = len(xy)

        v = np.zeros(n) if v is None else np.asarray(v, float).reshape(-1)
        kappa = np.zeros(n) if kappa is None else np.asarray(kappa, float).reshape(-1)
        if len(v) != n or len(kappa) != n:
            raise ValueError('waypoint columns have inconsistent lengths')
        if not np.all(np.isfinite(xy)):
            raise ValueError('waypoint positions contain NaN or inf')

        seg = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
        if np.any(seg < 1e-9):
            raise ValueError('waypoint file contains coincident consecutive points')
        s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
        return cls(xy=xy, v=v, kappa=kappa, s=s, seg=seg, length=float(seg.sum()))


# Column aliases, lowercase. First match wins.
_X_KEYS = ('x_m', 'x', 'pos_x', 'x_ref_m')
_Y_KEYS = ('y_m', 'y', 'pos_y', 'y_ref_m')
_V_KEYS = ('v_mps', 'vx_mps', 'v', 'speed', 'vel', 'velocity', 'v_ref_mps')
_K_KEYS = ('kappa_radpm', 'kappa', 'curvature', 'k')


def _pick(header: Sequence[str], keys: Sequence[str]) -> Optional[int]:
    lowered = [h.strip().lower().lstrip('# ') for h in header]
    for key in keys:
        if key in lowered:
            return lowered.index(key)
    return None


def _sniff_delimiter(sample: str) -> str:
    counts = {d: sample.count(d) for d in (';', ',', '\t', ' ')}
    return max(counts, key=counts.get) if max(counts.values()) else ','


def load_waypoints(path: str) -> Path:
    """Load a closed racing line from CSV.

    Understands, in order of preference:

    1. A header row naming its columns: the map pipeline's own export
       (s_m,x_m,y_m,psi_rad,kappa_radpm,v_mps,v_crit_mps) or TUM
       global_racetrajectory_optimization output (# s_m; x_m; y_m; psi_rad;
       kappa_radpm; vx_mps; ax_mps2 - ';'-delimited, header behind a '#').
    2. A headerless numeric file of 2 columns (x, y) or 3+ (x, y, v, ...).

    Raises OSError if the file is unreadable and ValueError if the contents
    cannot be read as a loop.
    """
    with open(path, 'r', newline='') as fh:
        raw = fh.read()
    if not raw.strip():
        raise ValueError('waypoints file is empty')

    lines = [ln for ln in raw.splitlines() if ln.strip()]
    delim = _sniff_delimiter(lines[0] if len(lines) < 2 else lines[1])

    header = None
    body_start = 0
    first = lines[0].lstrip()
    probe = list(csv.reader([lines[0].lstrip('#').strip()], delimiter=delim))[0]
    if first.startswith('#') or not _all_numeric(probe):
        header = probe
        body_start = 1

    rows = []
    for ln in lines[body_start:]:
        if ln.lstrip().startswith('#'):
            continue
        fields = [f for f in next(csv.reader([ln], delimiter=delim)) if f.strip() != '']
        if not fields:
            continue
        try:
            rows.append([float(f) for f in fields])
        except ValueError as e:
            raise ValueError(f'non-numeric row in {path!r}: {ln!r}') from e

    if not rows:
        raise ValueError('waypoints file has no data rows')
    width = min(len(r) for r in rows)
    if width < 2:
        raise ValueError('waypoints file needs at least x and y columns')
    data = np.array([r[:width] for r in rows], dtype=float)

    if header is not None:
        ix, iy = _pick(header, _X_KEYS), _pick(header, _Y_KEYS)
        iv, ik = _pick(header, _V_KEYS), _pick(header, _K_KEYS)
        if ix is None or iy is None:
            raise ValueError(f'header in {path!r} has no recognisable x/y columns: {header}')
    else:
        ix, iy = 0, 1
        iv = 2 if width >= 3 else None
        ik = None

    def col(i):
        return data[:, i] if (i is not None and i < width) else None

    return Path.from_arrays(np.column_stack([data[:, ix], data[:, iy]]), col(iv), col(ik))


def _all_numeric(fields: Sequence[str]) -> bool:
    ok = False
    for f in fields:
        f = f.strip()
        if not f:
            continue
        try:
            float(f)
            ok = True
        except ValueError:
            return False
    return ok


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw [rad] from a quaternion, in (-pi, pi]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _project_on_segment(p, a, b):
    """Return (t, closest_point, distance) for point p onto segment a->b."""
    ab = b - a
    denom = float(ab @ ab)
    t = 0.0 if denom < 1e-12 else float(np.clip((p - a) @ ab / denom, 0.0, 1.0))
    q = a + t * ab
    return t, q, float(np.linalg.norm(p - q))


def nearest_point_on_path(
    position, path: Path, seed: Optional[int] = None, window: Optional[int] = None
):
    """Closest point on the loop to `position`, as (idx, t, point, distance).

    `seed` and `window` restrict the search to segments within `window`
    indices of `seed`, wrapping round the loop. A hairpin puts two distant
    parts of the line within a metre of each other, and an unrestricted search
    snaps to the wrong one. seed=None searches the whole loop: the first call,
    or after a reset.
    """
    xy = path.xy
    n = len(xy)
    if seed is None or window is None or window <= 0 or 2 * window + 1 >= n:
        candidates = range(n)
    else:
        candidates = ((seed + k) % n for k in range(-window, window + 1))

    best = (0, 0.0, xy[0], float('inf'))
    for i in candidates:
        t, q, d = _project_on_segment(np.asarray(position, float), xy[i], xy[(i + 1) % n])
        if d < best[3]:
            best = (i, t, q, d)
    return best


def _circle_segment_hit(position, a, b, radius):
    """Largest t in [0, 1] where segment a->b crosses the circle, else None."""
    d = b - a
    f = a - position
    aa = float(d @ d)
    if aa < 1e-12:
        return None
    bb = 2.0 * float(f @ d)
    cc = float(f @ f) - radius * radius
    disc = bb * bb - 4.0 * aa * cc
    if disc < 0.0:
        return None
    disc = math.sqrt(disc)
    for t in ((-bb + disc) / (2.0 * aa), (-bb - disc) / (2.0 * aa)):
        if 0.0 <= t <= 1.0:
            return t
    return None


def lookahead_point_on_path(position, path: Path, idx: int, t: float, radius: float):
    """First forward crossing of the lookahead circle, as (idx, t, point).

    Walks forward from (idx, t) to the first segment that leaves the circle of
    `radius` around `position`. If none does - the radius is larger than the
    loop, or the car is far off the line - it falls back to the point `radius`
    of arc length ahead, which a closed loop always has.
    """
    xy = path.xy
    n = len(xy)
    travelled = 0.0
    for k in range(n + 1):
        i = (idx + k) % n
        a, b = xy[i], xy[(i + 1) % n]
        start = t if k == 0 else 0.0
        seg_a = a + start * (b - a)
        hit = _circle_segment_hit(np.asarray(position, float), a, b, radius)
        if hit is not None and hit >= start:
            return i, hit, a + hit * (b - a)
        travelled += float(np.linalg.norm(b - seg_a))
        if travelled > 2.0 * radius + path.length:
            break
    return advance_by_arclength(path, idx, t, radius)


def advance_by_arclength(path: Path, idx: int, t: float, distance: float):
    """Move `distance` metres forward along the loop from (idx, t)."""
    n = len(path.xy)
    remaining = distance % path.length if path.length > 0 else 0.0
    i = idx % n
    remaining += t * path.seg[i]
    while remaining > path.seg[i]:
        remaining -= path.seg[i]
        i = (i + 1) % n
    tt = remaining / path.seg[i] if path.seg[i] > 1e-9 else 0.0
    a, b = path.xy[i], path.xy[(i + 1) % n]
    return i, tt, a + tt * (b - a)


def sample_at(values: np.ndarray, idx: int, t: float) -> float:
    """Linearly interpolate a per-waypoint quantity at (idx, t)."""
    n = len(values)
    return float((1.0 - t) * values[idx % n] + t * values[(idx + 1) % n])


def compute_steering(position, yaw: float, target, wheelbase: float) -> float:
    """Pure pursuit steering angle [rad].

    delta = atan2(2 * wheelbase * sin(alpha), Ld), with alpha the bearing to
    the target in the vehicle frame and Ld the distance to it. A target on top
    of the rear axle gives 0.
    """
    d = np.asarray(target, float) - np.asarray(position, float)
    ld = float(np.linalg.norm(d))
    if ld < 1e-6:
        return 0.0
    alpha = math.atan2(d[1], d[0]) - yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # wrap to (-pi, pi]
    return math.atan2(2.0 * wheelbase * math.sin(alpha), ld)


def pursuit_arc(target, n: int = 20) -> np.ndarray:
    """The circle compute_steering drives, from the rear axle to `target`.

    Vehicle frame: rear axle at the origin, heading +x. Returns (n, 2) points.
    """
    x, y = (float(v) for v in target)
    s = np.linspace(0.0, 1.0, n)
    if abs(y) < 1e-9:
        return np.column_stack([s * x, np.zeros(n)])  # dead ahead or behind: a line
    radius = (x * x + y * y) / (2.0 * y)  # signed, positive to the left
    heading = s * 2.0 * math.atan2(y, x)  # turned through by the time it gets there
    return radius * np.column_stack([np.sin(heading), 1.0 - np.cos(heading)])


def cross_track_error(position, yaw: float, nearest) -> float:
    """Signed lateral offset [m]; positive when the path lies to the left."""
    d = np.asarray(nearest, float) - np.asarray(position, float)
    return float(-math.sin(yaw) * d[0] + math.cos(yaw) * d[1])


def curvature_speed_limit(kappa, a_lat_max: float, v_cap):
    """v <= sqrt(a_lat_max / |kappa|), capped at v_cap. Scalars or arrays.

    A net under whatever the file asked for. It matters most for a
    hand-recorded lap, whose speed is what the driver did, not what the tyres
    allow.
    """
    if a_lat_max <= 0.0:
        return v_cap
    return np.minimum(v_cap, np.sqrt(a_lat_max / np.maximum(np.abs(kappa), 1e-9)))


def speed_profile(
    path: Path, speed_gain: float, max_speed: float, max_lateral_accel: float, max_decel: float
) -> np.ndarray:
    """Target speed at every waypoint, computed once per path.

    The file's speed times `speed_gain`, capped by `max_speed` and by grip,
    then braked early enough to meet every slower point ahead:
    v_i <= sqrt(v_(i+1)^2 + 2 * max_decel * seg_i). Without it the car arrives
    at a corner still carrying the straight's speed.
    """
    v = curvature_speed_limit(
        path.kappa, max_lateral_accel, np.minimum(path.v * speed_gain, max_speed)
    )
    if max_decel > 0.0:
        n = len(v)
        # Backwards, twice round: a corner just past the start brakes the end of the lap.
        for k in range(2 * n - 1, -1, -1):
            i = k % n
            v[i] = min(v[i], math.sqrt(v[(i + 1) % n] ** 2 + 2.0 * max_decel * path.seg[i]))
    return v
