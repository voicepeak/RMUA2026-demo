"""FRD body to NED quaternion operations; radians, wxyz convention."""
import numpy as np


def unit(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-8:
        raise ValueError("invalid quaternion")
    return q / np.linalg.norm(q)


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array((w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v))


def rotation(q):
    w, x, y, z = unit(q)
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


def delta_quaternion(angle):
    angle = np.asarray(angle, dtype=float)
    norm = np.linalg.norm(angle)
    if norm < 1e-8:
        return unit(np.r_[1., .5*angle])
    return np.r_[np.cos(norm/2), np.sin(norm/2)*angle/norm]


def skew(v):
    x, y, z = v
    return np.array(((0., -z, y), (z, 0., -x), (-y, x, 0.)))
