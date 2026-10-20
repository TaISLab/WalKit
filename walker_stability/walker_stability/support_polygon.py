"""Support polygon of the user's contacts and the margin of a point inside it. Pure python (no ROS, no numpy).

The user touches the walker's frame and the floor in four places: two feet and two handles. The *support polygon* is
the convex hull of the ones that carry load; a point whose vertical projection falls well inside it is held up from
every side, one close to its edge (or outside) can tip over that edge. The *margin* is the signed distance from the
point to the polygon's boundary: positive inside, negative outside, and zero or less when the polygon has no area
(fewer than three contacts, or all on a line), so that there is nothing to stand in.

The point is the centre of pressure for the walker (the barycentre of the contacts weighted by their loads, which
is what walker_centroid_support publishes) and the projection of the centre of mass for the mocap
(mocap_stability); the geometry is the same.
"""
import math
from typing import NamedTuple

FOOT, HANDLE = 'foot', 'handle'


class Contact(NamedTuple):
    kind: str       # FOOT or HANDLE
    x: float        # m
    y: float        # m
    load: float     # kg


def footprint(x, y, heel_m, toe_m, half_width_m):
    """Corners of the sole of a foot tracked at (x, y), pointing along +x (the walker's heading): `heel_m` behind the
    tracked point, `toe_m` in front of it, `half_width_m` to each side. The legs are tracked at the shank, not at
    the floor: a stance foot is a patch of floor a few centimetres around that point, and a polygon made of the
    points alone would have no area while the user stands on one foot."""
    return [(x - heel_m, y - half_width_m), (x - heel_m, y + half_width_m),
            (x + toe_m, y - half_width_m), (x + toe_m, y + half_width_m)]


def centre_of_pressure(contacts):
    """(x, y) barycentre of the contacts weighted by their load, or None if they carry nothing."""
    total = sum(c.load for c in contacts)
    if total <= 0.0:
        return None
    return (sum(c.load * c.x for c in contacts) / total, sum(c.load * c.y for c in contacts) / total)


def convex_hull(points):
    """Vertices of the convex hull of (x, y) points, counter-clockwise, without repeated or collinear ones (Andrew's
    monotone chain). Fewer than three vertices when the points are coincident or on a line."""
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    chains = []
    for sequence in (pts, pts[::-1]):
        chain = []
        for p in sequence:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], p) <= 0.0:
                chain.pop()
            chain.append(p)
        chains.append(chain[:-1])
    return chains[0] + chains[1]


def area(hull):
    """Area of a convex polygon given counter-clockwise (0 for fewer than three vertices)."""
    if len(hull) < 3:
        return 0.0
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(hull, hull[1:] + hull[:1]))


def _distance_to_segment(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def margin(point, hull):
    """Signed distance from `point` to the boundary of the convex polygon `hull` (counter-clockwise):
    positive inside, negative outside. A hull without area (up to two vertices) has no inside: the result is
    minus the distance to it, 0 on it. None if there is no hull at all."""
    if not hull:
        return None
    edges = list(zip(hull, hull[1:] + hull[:1])) if len(hull) > 1 else [(hull[0], hull[0])]
    distance = min(_distance_to_segment(point, a, b) for a, b in edges)
    if len(hull) < 3:
        return -distance
    inside = all((b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0]) >= 0.0 for a, b in edges)
    return distance if inside else -distance


def index(margin_m, reference_m):
    """0 to 1 stability index: the margin over `reference_m`, the margin that counts as fully stable (a scale, not a
    validated threshold), clipped."""
    if margin_m is None or reference_m <= 0.0:
        return 0.0
    return max(0.0, min(1.0, margin_m / reference_m))


class Assessment(NamedTuple):
    area: float         # m2, of the polygon
    margin: float       # m, see margin(); 0 with no polygon at all
    sta: float          # 0..1, see index()


def assess(points, point, reference_m):
    """Margin and index of `point` (x, y) in the polygon of `points` (their (x, y))."""
    hull = convex_hull(points)
    m = margin(point, hull) if point is not None else None
    return Assessment(area(hull), 0.0 if m is None else m, index(m, reference_m))
