"""ROS-independent coordinate, setpoint and watchdog helpers.

All vectors exposed by these helpers are ROS ENU. MAVROS itself converts the
PositionTarget payload to MAVLink NED. Applying another ENU/NED swap here would
therefore be incorrect. IMU acceleration is deliberately not gravity-corrected.
"""
import math
import numpy as np

POSITION_MASK = 8 | 16 | 32 | 64 | 128 | 256 | 2048
VELOCITY_MASK = 1 | 2 | 4 | 64 | 128 | 256 | 2048
ACCELERATION_MASK = 1 | 2 | 4 | 8 | 16 | 32 | 2048


def finite_vector(value, size=3):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError("Expected a finite vector of length %d" % size)
    return a


def rotation(yaw):
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap_angle(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def quaternion_yaw(quaternion):
    q = finite_vector(quaternion, 4)
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        raise ValueError("Invalid zero quaternion")
    x, y, z, w = q / norm
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


class PlanarTransform:
    """Rigid yaw + translation mapping a named source ENU frame to world ENU."""
    def __init__(self, specification=None):
        s = specification or {}
        self.yaw = float(s.get("yaw", 0.0))
        if not math.isfinite(self.yaw):
            raise ValueError("Nonfinite transform yaw")
        self.translation = finite_vector(s.get("translation", [0.0, 0.0, 0.0]))
        self.R = rotation(self.yaw)

    def point_to_world(self, point):
        return self.R @ finite_vector(point) + self.translation

    def point_from_world(self, point):
        return self.R.T @ (finite_vector(point) - self.translation)

    def vector_to_world(self, vector):
        return self.R @ finite_vector(vector)

    def vector_from_world(self, vector):
        return self.R.T @ finite_vector(vector)


def limit_norm(vector, limit):
    v = np.asarray(vector, dtype=float)
    if not np.isfinite(v).all() or not math.isfinite(float(limit)) or limit <= 0:
        raise ValueError("Nonfinite command or invalid limit")
    norm = float(np.linalg.norm(v))
    return (v * (limit / norm), True) if norm > limit else (v.copy(), False)


def ugv_command(world_velocity, world_yaw, frame, max_speed):
    v = finite_vector(world_velocity)
    planar, clipped = limit_norm(v[:2], max_speed)
    limited = np.array([planar[0], planar[1], 0.0])
    if frame == "body":
        command = rotation(world_yaw).T @ limited
    elif frame == "world":
        command = limited
    else:
        raise ValueError("UGV command frame must be body or world")
    return command, limited, clipped


def position_target(world_position, world_yaw, transform):
    return _target("position", transform.point_from_world(world_position), world_yaw, transform)


def velocity_target(world_velocity, world_yaw, transform):
    return _target("velocity", transform.vector_from_world(world_velocity), world_yaw, transform)


def acceleration_target(world_acceleration, world_yaw, transform):
    return _target("acceleration", transform.vector_from_world(world_acceleration), world_yaw, transform)


def _target(kind, vector, yaw, transform):
    masks = {"position": POSITION_MASK, "velocity": VELOCITY_MASK, "acceleration": ACCELERATION_MASK}
    result = {"coordinate_frame": 1, "type_mask": masks[kind], "position": [0.0]*3,
              "velocity": [0.0]*3, "acceleration_or_force": [0.0]*3,
              "yaw": wrap_angle(float(yaw)-transform.yaw), "yaw_rate": 0.0, "kind": kind}
    result[{"position": "position", "velocity": "velocity", "acceleration": "acceleration_or_force"}[kind]] = finite_vector(vector).tolist()
    return result


def height_acceleration(z, vz, target, kp, kd, maximum):
    raw = float(kp)*(float(target)-float(z)) - float(kd)*float(vz)
    if not math.isfinite(raw) or maximum <= 0:
        raise ValueError("Invalid height-controller input")
    value = float(np.clip(raw, -maximum, maximum))
    return value, value != raw


def sample_is_fresh(now, receipt_age, source_stamp, timeout, future_tolerance=0.1):
    return (math.isfinite(now) and math.isfinite(receipt_age) and 0 <= receipt_age <= timeout
            and math.isfinite(source_stamp) and source_stamp > 0
            and -future_tolerance <= now-source_stamp <= timeout)


class DerivativeEstimate:
    """Differentiated world velocity, distinct from raw body-frame IMU force."""
    def __init__(self, maximum_gap=0.3):
        self.last_stamp = None
        self.last_velocity = None
        self.maximum_gap = maximum_gap

    def update(self, velocity, stamp):
        velocity = finite_vector(velocity)
        if self.last_stamp is None:
            self.last_stamp, self.last_velocity = float(stamp), velocity.copy()
            return None
        dt = float(stamp)-self.last_stamp
        if dt <= 0:
            return None
        derivative = (velocity-self.last_velocity)/dt if dt <= self.maximum_gap else None
        self.last_stamp, self.last_velocity = float(stamp), velocity.copy()
        return derivative


class StartGate:
    """Two-phase start: SCHEDULED prepares; only RUNNING commits execution."""
    def __init__(self):
        self.session_id = None
        self.t0 = None
        self.prepared = False
        self.committed = False
        self.started = False

    def prepare(self, session_id, t0, now):
        if not session_id or not math.isfinite(float(t0)) or t0 <= now:
            raise ValueError("Schedule must have a nonempty session and future t0")
        if self.prepared and self.session_id == session_id:
            if abs(self.t0-t0) > 1e-9:
                raise ValueError("Scheduled t0 changed within a session")
            return False
        self.session_id, self.t0 = session_id, float(t0)
        self.prepared, self.committed, self.started = True, False, False
        return True

    def commit(self, session_id, t0, now):
        if not self.prepared or self.session_id != session_id or abs(self.t0-t0) > 1e-9:
            raise ValueError("RUNNING without the matching SCHEDULED barrier")
        if not self.committed and now >= self.t0:
            raise ValueError("Late start commit")
        self.committed = True

    def may_run(self, phase, now):
        return phase == "RUNNING" and self.prepared and self.committed and now >= self.t0
