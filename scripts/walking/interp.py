from qpsolvers import solve_qp
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from classes import SE3

def _factor_cubic_hermite_curve(p0, n0, p1, n1):
    def H_lambda(s):
        return s * (1 + s * (s - 2)) * n0

    def H_mu(s):
        return s**2 * (s - 1) * n1

    def H_cst(s):
        return p0 + s**2 * (3 - 2 * s) * (p1 - p0)

    return H_lambda, H_mu, H_cst


def _cubic_hermite_pos(p0, v0, p1, v1, s):
    s = float(s)
    h00 = 2*s**3 - 3*s**2 + 1
    h10 = s**3 - 2*s**2 + s
    h01 = -2*s**3 + 3*s**2
    h11 = s**3 - s**2
    return h00*p0 + h10*v0 + h01*p1 + h11*v1

class CubicHermiteInterpolation:
    """
    Time-parameterized cubic Hermite foot trajectory (position only).
    """

    def __init__(
        self,
        start_pose: SE3,
        end_pose: SE3,
        duration,
        *,
        n0=(0, 0, 1),
        n1=(0, 0, 1),
        takeoff_clearance=0.01,
        landing_clearance=0.01,
        s_takeoff=0.25,
        s_landing=0.75,
        qp_solver='proxqp',
    ):
        self.p0 = np.asarray(start_pose.position, dtype=float)
        self.p1 = np.asarray(end_pose.position, dtype=float)
        self.duration = float(duration)

        # normalize normals
        self.n0 = np.asarray(n0, dtype=float)
        self.n1 = np.asarray(n1, dtype=float)
        self.n0 /= max(np.linalg.norm(self.n0), 1e-12)
        self.n1 /= max(np.linalg.norm(self.n1), 1e-12)

        self.takeoff_clearance = takeoff_clearance
        self.landing_clearance = landing_clearance
        self.s_takeoff = s_takeoff
        self.s_landing = s_landing

        # internal time
        self.t = 0.0
        rotations = R.from_quat([start_pose.rotation.as_quat(), end_pose.rotation.as_quat()])
        self.slerp = Slerp([0, 1], rotations)

        # solve tangent magnitudes once
        self.lambda_, self.mu_ = self._solve_tangent_magnitudes(qp_solver)

        # endpoint tangents
        self.v0 = self.lambda_ * self.n0
        self.v1 = self.mu_ * self.n1

    def _solve_tangent_magnitudes(self, qp_solver=None):
        H_lambda, H_mu, H_cst = _factor_cubic_hermite_curve(self.p0, self.n0, self.p1, self.n1)

        s0 = self.s_takeoff
        a0 = np.dot(H_lambda(s0), self.n0)
        b0 = np.dot(H_mu(s0), self.n0)
        c0 = np.dot(H_cst(s0) - self.p0, self.n0)

        s1 = self.s_landing
        a1 = np.dot(H_lambda(s1), self.n1)
        b1 = np.dot(H_mu(s1), self.n1)
        c1 = np.dot(H_cst(s1) - self.p1, self.n1)

        G = np.array([[-a0, -b0],
                      [-a1, -b1]])
        h = np.array([c0 - self.takeoff_clearance,
                      c1 - self.landing_clearance])

        P = np.eye(2)
        q = np.zeros(2)

        x = solve_qp(P, q, G, h, solver=qp_solver)

        if x is None:
            return 0.0, 0.0

        return float(x[0]), float(x[1])

    def pos(self, s):
        return _cubic_hermite_pos(self.p0, self.v0, self.p1, self.v1, s)

    def integrate(self, dt):
        """
        Advance trajectory by dt and return position.
        After duration seconds, it stays at end_pos.
        """
        self.t += dt
        if self.t >= self.duration:
            self.t = self.duration
            pos = self.p1.copy()
            rot = self.slerp(1.0)
        else:
            s = np.clip(self.t / self.duration, 0.0, 1.0)
            pos = self.pos(s)
            rot = self.slerp(s)
        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = pos
        return SE3(T)

    def reset(self):
        """Restart trajectory."""
        self.t = 0.0

def interpolate_linear(start, end, progress):
    return start + (end - start) * progress