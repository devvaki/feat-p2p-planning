# p2p_planning/geo_cyth.pyx
# cython: boundscheck=False, wraparound=False, cdivision=True, language_level=3
cimport cython
from libc.math cimport sqrt, fabs

@cython.inline
cdef double _clamp(double v, double lo, double hi) nogil:
    if v < lo: return lo
    if v > hi: return hi
    return v

@cython.cfunc
cdef double _hypot2(double ax, double ay) nogil:
    return ax*ax + ay*ay

@cython.cfunc
cdef double _hypot(double ax, double ay) nogil:
    return sqrt(ax*ax + ay*ay)

@cython.cfunc
cdef double _dist_point_to_segment_raw(double px, double py,
                                       double x1, double y1,
                                       double x2, double y2) nogil:
    cdef double dx = x2 - x1
    cdef double dy = y2 - y1
    cdef double seg2 = dx*dx + dy*dy

    # Degenerate segment → distance to endpoint
    if seg2 == 0.0:
        return _hypot(px - x1, py - y1)

    # Projection parameter clamped to [0,1]
    cdef double t = ((px - x1)*dx + (py - y1)*dy) / seg2
    t = _clamp(t, 0.0, 1.0)

    cdef double qx = x1 + t*dx
    cdef double qy = y1 + t*dy
    return _hypot(px - qx, py - qy)

def dist_point_to_segment(double px, double py,
                          double x1, double y1,
                          double x2, double y2) -> double:
    """
    Fast distance from point (px,py) to line segment (x1,y1)-(x2,y2).
    """
    return _dist_point_to_segment_raw(px, py, x1, y1, x2, y2)

@cython.boundscheck(False)
@cython.wraparound(False)
def min_dist_to_edges(double px, double py, double[:, :] ring) -> double:
    """
    Minimum distance from point to the polyline edges of `ring` (Nx2).
    `ring` should be a *closed* ring (first vertex == last), but we also
    handle open input by treating the last->first edge implicitly.
    """
    cdef Py_ssize_t n = ring.shape[0]
    if n < 2:
        return 1e300  # empty ring

    cdef Py_ssize_t i
    cdef double x1, y1, x2, y2, d, best
    best = 1e300

    # iterate edges [i] -> [i+1], including last->first
    for i in range(n - 1):
        x1 = ring[i, 0]; y1 = ring[i, 1]
        x2 = ring[i+1, 0]; y2 = ring[i+1, 1]
        d = _dist_point_to_segment_raw(px, py, x1, y1, x2, y2)
        if d < best: best = d

    # if not already closed, include last->first
    if ring[0,0] != ring[n-1,0] or ring[0,1] != ring[n-1,1]:
        x1 = ring[n-1, 0]; y1 = ring[n-1, 1]
        x2 = ring[0,   0]; y2 = ring[0,   1]
        d = _dist_point_to_segment_raw(px, py, x1, y1, x2, y2)
        if d < best: best = d

    return best

@cython.boundscheck(False)
@cython.wraparound(False)
def violates_clearance(double px, double py,
                       double[:, :] ring, double clearance) -> bint:
    """
    Early-exit YES/NO: return True as soon as any edge of `ring` is within
    `clearance` of (px,py). `ring` is (Nx2) double memoryview.
    """
    cdef Py_ssize_t n = ring.shape[0]
    if n < 2:
        return False

    cdef Py_ssize_t i
    cdef double x1, y1, x2, y2
    cdef double thr = clearance

    for i in range(n - 1):
        x1 = ring[i, 0]; y1 = ring[i, 1]
        x2 = ring[i+1, 0]; y2 = ring[i+1, 1]
        if _dist_point_to_segment_raw(px, py, x1, y1, x2, y2) < thr:
            return True

    # include last->first if open
    if ring[0,0] != ring[n-1,0] or ring[0,1] != ring[n-1,1]:
        x1 = ring[n-1, 0]; y1 = ring[n-1, 1]
        x2 = ring[0,   0]; y2 = ring[0,   1]
        if _dist_point_to_segment_raw(px, py, x1, y1, x2, y2) < thr:
            return True

    return False
