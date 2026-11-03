"""Group-local reciprocal acceleration barrier filter, independent of ROS.

UAV pairs share a second-order separation constraint equally. This requires
both agents to run the same filter on fresh, consistently framed states.
Vehicle avoidance uses a 3-D inflated ellipsoid and only changes UAV XY input.
The sampled PX4 implementation still needs tracking-error/delay validation;
continuous double-integrator barrier algebra is not a hardware certificate.
"""
import math
import time
import numpy as np


class SafetyError(ValueError):
    pass


def challenge_acceleration(config, group, follower, tau, velocity):
    """Explicit SSS-only validation input; never used in normal operation."""
    challenge = config.get("simulation", {}).get("safety_challenge", {})
    if not challenge.get("enabled", False):
        return None
    start, duration, speed = [float(challenge.get(k, v)) for k, v in
                              [("start_s", 8.), ("duration_s", 4.), ("speed", .2)]]
    if not np.isfinite([start, duration, speed]).all() or start < 0 or duration <= 0 or speed <= 0:
        raise SafetyError("Invalid SSS safety challenge parameters")
    if config.get("simulation", {}).get("backend") != "sss":
        raise SafetyError("Artificial safety challenge requires the SSS configuration")
    if not start <= tau < start+duration:
        return None
    angle = config["formation"]["angular_rates"][group]*start+config["formation"]["phases"][group]+follower*math.pi
    direction = -np.array([math.cos(angle), math.sin(angle)])
    return 2*(speed*direction-np.asarray(velocity)[:2])


def project_halfplanes(nominal, rows, bounds):
    """Exact Euclidean projection in R2 onto G a >= b; no relaxed safety rows."""
    u = np.asarray(nominal, dtype=float)
    g, b = np.asarray(rows, dtype=float), np.asarray(bounds, dtype=float)
    if u.shape != (2,) or g.ndim != 2 or g.shape[1] != 2 or b.shape != (len(g),):
        raise SafetyError("Invalid safety optimization dimensions")
    if not all(np.isfinite(x).all() for x in (u, g, b)):
        raise SafetyError("Nonfinite safety optimization input")
    norm2 = np.sum(g*g, axis=1)
    if np.any((norm2 < 1e-16) & (b > 1e-9)):
        raise SafetyError("Infeasible degenerate separation constraint")
    valid = norm2 >= 1e-16
    g, b, norm2 = g[valid], b[valid], norm2[valid]
    if np.all(g@u >= b-1e-9):
        return u.copy()
    candidates = [u]
    candidates.extend(u + ((b-g@u)/norm2)[:, None]*g)
    i, j = np.triu_indices(len(g), 1)
    determinant = g[i, 0]*g[j, 1]-g[i, 1]*g[j, 0]
    usable = np.abs(determinant) > 1e-12
    i, j, determinant = i[usable], j[usable], determinant[usable]
    candidates.extend(np.stack(((b[i]*g[j, 1]-g[i, 1]*b[j])/determinant,
                                (g[i, 0]*b[j]-b[i]*g[j, 0])/determinant), axis=1))
    candidates = np.asarray(candidates)
    feasible = candidates[np.all(candidates@g.T >= b[None, :]-1e-8, axis=1)]
    if not len(feasible):
        raise SafetyError("Safety constraints infeasible within acceleration/speed limits")
    return feasible[np.argmin(np.sum((feasible-u)**2, axis=1))].copy()


class GroupSafety:
    def __init__(self, config):
        self.config = config
        self.s = config.get("safety", {})
        self.enabled = self.s.get("enabled", False)
        if not isinstance(self.enabled, bool):
            raise SafetyError("safety.enabled must be boolean")
        self.distance = float(self.s.get("uav_min_distance", .45))
        self.k1 = float(self.s.get("barrier_k1", 2.0))
        self.k2 = float(self.s.get("barrier_k2", 2.0))
        self.error_bound = float(self.s.get("acceleration_error_bound", .15))
        self.ugv_acceleration = float(self.s.get("ugv_acceleration_bound", .5))
        self.rx = float(self.s.get("ugv_horizontal_radius", .48))
        self.rz = float(self.s.get("ugv_vertical_radius", .55))
        self.zc = float(self.s.get("ugv_center_height", .10))
        self.max_age = float(self.s.get("state_max_age", .10))
        self.horizon = float(self.s.get("speed_prediction_horizon", .20))
        self.amax = float(config["limits"]["uav_acceleration"])
        self.vmax = float(config["limits"]["uav_speed"])
        values = [self.distance, self.k1, self.k2, self.rx, self.rz, self.max_age,
                  self.horizon, self.amax, self.vmax]
        if any(not math.isfinite(x) or x <= 0 for x in values):
            raise SafetyError("Safety distances, gains, ages and limits must be positive")
        if any(not math.isfinite(x) or x < 0 for x in [self.error_bound, self.ugv_acceleration, self.zc]):
            raise SafetyError("Safety uncertainty bounds must be nonnegative")
        if self.enabled:
            for r in config["formation"]["radii"]:
                if 2*float(r) <= self.distance:
                    raise SafetyError("Desired formation violates UAV separation")
            # Enclose the inflated car cylinder, including UAV dimensions.
            body_xy = float(self.s.get("ugv_body_radius", math.hypot(.15, .15)))
            body_z = float(self.s.get("ugv_body_half_height", .10))
            aircraft_xy = float(self.s.get("uav_body_radius", .15))
            aircraft_z = float(self.s.get("uav_body_half_height", .08))
            margin = float(self.s.get("body_margin", .05))
            if any(not math.isfinite(x) or x < 0 for x in [body_xy, body_z, aircraft_xy, aircraft_z, margin]):
                raise SafetyError("Invalid physical safety envelope")
            if self.distance < 2*aircraft_xy+margin:
                raise SafetyError("UAV distance does not cover the physical envelope")
            if ((body_xy+aircraft_xy+margin)/self.rx)**2 + ((body_z+aircraft_z+margin)/self.rz)**2 > 1:
                raise SafetyError("UGV ellipsoid does not enclose the inflated physical envelope")
        angles = np.arange(8)*math.pi/4
        self.normals = np.stack((np.cos(angles), np.sin(angles)), axis=1)
        self.polygon_factor = math.cos(math.pi/8)

    def geometry(self, position, peer_position, leader_position):
        p = np.asarray(position)-np.asarray(peer_position)
        car = np.asarray(position)-np.asarray(leader_position)-[0., 0., self.zc]
        return {"uav_distance_xy_m": float(np.linalg.norm(p[:2])),
                "uav_distance_3d_m": float(np.linalg.norm(p)),
                "ugv_distance_3d_m": float(np.linalg.norm(np.asarray(position)-leader_position)),
                "uav_barrier_h": float(p[:2]@p[:2]-self.distance**2),
                "ugv_barrier_h": float(np.sum(car*car/np.array([self.rx**2, self.rx**2, self.rz**2]))-1)}

    def filter(self, nominal, position, velocity, peer_position, peer_velocity,
               leader_position, leader_velocity, vertical_acceleration=0., leader_stationary=False):
        started = time.perf_counter()
        inputs = [np.asarray(x, dtype=float) for x in
                  (position, velocity, peer_position, peer_velocity, leader_position, leader_velocity)]
        if any(x.shape != (3,) or not np.isfinite(x).all() for x in inputs) or not math.isfinite(vertical_acceleration):
            raise SafetyError("Safety requires finite 3-D position/velocity samples")
        position, velocity, peer_position, peer_velocity, leader_position, leader_velocity = inputs
        p, v = (position-peer_position)[:2], (velocity-peer_velocity)[:2]
        geo = self.geometry(position, peer_position, leader_position)
        h, hd = geo["uav_barrier_h"], 2*float(p@v)
        # Each UAV satisfies half of the pair inequality. The error term covers
        # bounded acceleration tracking errors on BOTH aircraft, split equally.
        pair_b = -float(v@v)-.5*(self.k1+self.k2)*hd-.5*self.k1*self.k2*h
        pair_b += 2*np.linalg.norm(p)*self.error_bound
        rows, bounds = [2*p], [pair_b]
        car = position-leader_position-[0., 0., self.zc]
        relative_v = velocity-leader_velocity
        q = 1/np.array([self.rx**2, self.rx**2, self.rz**2])
        car_hd = 2*float((car*q)@relative_v)
        car_b = (-2*float((relative_v*q)@relative_v)-2*car[2]*q[2]*vertical_acceleration
                 -(self.k1+self.k2)*car_hd-self.k1*self.k2*geo["ugv_barrier_h"])
        obstacle_bound = 0. if leader_stationary else self.ugv_acceleration
        car_b += 2*np.linalg.norm(car[:2]*q[:2])*obstacle_bound
        car_b += 2*np.linalg.norm(car*q)*self.error_bound
        rows.append(2*car[:2]*q[:2]); bounds.append(car_b)
        # Inscribed octagons keep Euclidean limits satisfied without post-QP
        # clipping. Speed constraints use a short constant-acceleration horizon.
        rows.extend(-self.normals); bounds.extend([-self.amax*self.polygon_factor]*8)
        rows.extend(-self.horizon*self.normals)
        bounds.extend(self.normals@velocity[:2]-self.vmax*self.polygon_factor)
        output = project_halfplanes(nominal, rows, bounds)
        correction = float(np.linalg.norm(output-np.asarray(nominal)))
        return output, dict(geo, enabled=True, healthy=True, active=correction>1e-7,
                            correction_norm_m_s2=correction,
                            nominal_acceleration=np.asarray(nominal).tolist(),
                            applied_acceleration=output.tolist(),
                            minimum_constraint_margin=float(np.min(np.asarray(rows)@output-bounds)),
                            solve_seconds=time.perf_counter()-started)

    def guard_transfer(self, own, peer, leader, goal, peer_goal):
        """Geometry guard for synchronized straight position-reference paths."""
        own, peer, leader, goal, peer_goal = [np.asarray(x, dtype=float) for x in (own, peer, leader, goal, peer_goal)]
        if any(x.shape != (3,) or not np.isfinite(x).all() for x in (own, peer, leader, goal, peer_goal)):
            raise SafetyError("Invalid position-reference geometry")
        p = (own-peer)[:2]
        delta = ((goal-own)-(peer_goal-peer))[:2]
        fraction = np.clip(-float(p@delta)/max(float(delta@delta), 1e-15), 0., 1.)
        if np.linalg.norm(p+fraction*delta) < self.distance-1e-5:
            raise SafetyError("position transfer intersects the peer UAV envelope")
        center = leader+[0., 0., self.zc]
        axes = np.array([self.rx, self.rx, self.rz])
        p, delta = (own-center)/axes, (goal-own)/axes
        fraction = np.clip(-float(p@delta)/max(float(delta@delta), 1e-15), 0., 1.)
        if np.linalg.norm(p+fraction*delta) < 1.-1e-5:
            raise SafetyError("position transfer intersects the vehicle envelope")
