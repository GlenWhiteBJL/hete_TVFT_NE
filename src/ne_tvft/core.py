"""ROS-independent implementation of the paper's first-order leaders and
second-order followers. All online arrays are planar ENU coordinates.

Controller outputs are instantaneous and do not mutate state. ``step`` advances
only controller states, with received neighbor packets held over that period.
Plant integration belongs to the hardware (or to the offline demonstration).
No true equilibrium or non-neighbor physical state is read by a controller.
"""
import math
import numpy as np


def _array(value, shape, label):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError("{} must be finite with shape {}".format(label, shape))
    return result


def _step_size(dt):
    dt = float(dt)
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    return dt


def _rk4(value, dt, rhs):
    k1 = rhs(value)
    k2 = rhs(value + 0.5 * dt * k1)
    k3 = rhs(value + 0.5 * dt * k2)
    k4 = rhs(value + dt * k3)
    result = value + dt * (k1 + 2*k2 + 2*k3 + k4) / 6.0
    if not np.isfinite(result).all():
        raise FloatingPointError("Non-finite controller integration result")
    return result


def validate_topology(config):
    """Validate fixed graph assumptions; A[i,k] means i receives k."""
    m = len(config["groups"])
    if m < 2 or any(len(group["followers"]) != 2 for group in config["groups"]):
        raise ValueError("Require at least two groups with exactly two followers each")
    topology = config["topology"]
    leaders = _array(topology["leaders"], (m,m), "leader topology")
    followers = _array(topology["followers"], (2,2), "follower topology")
    pinning = _array(topology["pinning"], (m,2), "leader pinning")
    for name, matrix in (("leaders", leaders), ("followers", followers)):
        if ((matrix < 0).any() or not np.allclose(matrix, matrix.T)
                or np.any(np.diag(matrix))):
            raise ValueError(name + " graph must be symmetric, nonnegative and without self edges")
    if (pinning < 0).any():
        raise ValueError("Pinning weights must be nonnegative")
    laplacian = np.diag(leaders.sum(axis=1)) - leaders
    if np.linalg.eigvalsh(laplacian)[1] <= 1e-10:
        raise ValueError("Leader graph is disconnected")
    lf = np.diag(followers.sum(axis=1)) - followers
    if any(np.linalg.eigvalsh(lf + np.diag(p)).min() <= 1e-10 for p in pinning):
        raise ValueError("Each follower graph must be rooted at its leader")
    return leaders, followers, pinning


def _gains(config):
    g = config["gains"]
    required = ("k_L", "k_1", "lambda_follower", "kappa_alpha", "sigma_alpha", "delta_alpha",
                "alpha_initial", "alpha_hat_initial", "kappa_beta", "sigma_beta", "delta_beta",
                "beta_initial", "beta_hat_initial")
    if any(not math.isfinite(float(g[name])) or float(g[name]) <= 0 for name in required):
        raise ValueError("All controller gains must be positive and finite")
    for name in ("alpha", "beta"):
        if g[name + "_initial"] < g[name + "_hat_initial"]:
            raise ValueError("Initial adaptive gain must be no smaller than its filtered state")
    return g


def circle(config, group_index, follower_index, tau):
    """Constant radius and angular rate from tau=0; two UAVs differ by pi.

    No angular-rate ramp is introduced: the startup velocity transient is part
    of the experiment. Heading and altitude are handled independently.
    """
    if follower_index not in (0, 1) or not math.isfinite(float(tau)):
        raise ValueError("Invalid follower index or formation time")
    f = config["formation"]
    radius = float(f["radii"][group_index])
    omega = float(f["angular_rates"][group_index])
    phase = float(f["phases"][group_index])
    if not all(math.isfinite(x) for x in (radius,omega,phase)) or radius <= 0:
        raise ValueError("Formation radius must be positive; all parameters must be finite")
    angle = omega * float(tau) + phase + follower_index * math.pi
    h = radius * np.array([math.cos(angle), math.sin(angle)])
    dh = omega * np.array([-h[1], h[0]])
    ddh = -omega**2 * h
    return h, dh, ddh


class MonitoringGame:
    """Task preference + station distance + Gaussian group-footprint overlap.

    ``equilibrium`` is an explicitly separate offline diagnostic method. It is
    not invoked in initialization or by either online controller.
    """
    def __init__(self, config):
        c = config["monitoring"]
        self.m = len(config["groups"])
        self.hotspots = _array(c["hotspots"], (self.m,2), "hotspots")
        self.stations = _array(c["stations"], (self.m,2), "stations")
        self.a = _array(c["task_weights"], (self.m,), "task weights")
        self.eta = _array(c["station_weights"], (self.m,), "station weights")
        self.gamma = _array(c["overlap_weights"], (self.m,self.m), "overlap weights")
        self.sigma = float(c["overlap_sigma"])
        if ((self.a <= 0).any() or (self.eta < 0).any() or (self.gamma < 0).any()
                or not np.allclose(self.gamma,self.gamma.T) or np.any(np.diag(self.gamma))
                or not math.isfinite(self.sigma) or self.sigma <= 0):
            raise ValueError("Invalid monitoring weights: require a>0, eta>=0 and symmetric gamma>=0")
        self.q = self.a + self.eta
        self.anchor = self.a[:,None]*self.hotspots + self.eta[:,None]*self.stations
        self.independent_positions = self.anchor / self.q[:,None]
        self.Lgamma = np.diag(self.gamma.sum(axis=1)) - self.gamma
        self.mu = float(np.linalg.eigvalsh(np.diag(self.q)-self.Lgamma/self.sigma**2).min())
        self.lipschitz = float(np.linalg.eigvalsh(np.diag(self.q)+2*np.exp(-1.5)*self.Lgamma/self.sigma**2).max())
        if self.mu <= 0:
            raise ValueError("Monitoring game is not globally strong-monotonicity certified")

    def pair_overlaps(self, positions):
        p = _array(positions,(self.m,2),"positions")
        diff = p[:,None,:]-p[None,:,:]
        return np.exp(-np.sum(diff**2,axis=-1)/(2*self.sigma**2))

    def components(self, positions):
        p = _array(positions,(self.m,2),"positions")
        return (0.5*self.a*np.sum((p-self.hotspots)**2,axis=-1),
                0.5*self.eta*np.sum((p-self.stations)**2,axis=-1),
                np.sum(self.gamma*self.pair_overlaps(p),axis=1))

    def costs(self, positions):
        task,station,overlap = self.components(positions)
        return task+station+overlap

    def potential(self, positions):
        task,station,overlap = self.components(positions)
        return float(np.sum(task+station+0.5*overlap))

    def gradient(self, positions):
        p = _array(positions,(self.m,2),"positions")
        diff = p[:,None,:]-p[None,:,:]
        weights = self.gamma*self.pair_overlaps(p)/self.sigma**2
        return self.q[:,None]*p-self.anchor-np.einsum("ij,ijd->id",weights,diff)

    def local_gradient(self, group_index, estimates):
        z = _array(estimates,(self.m,2),"local estimates")
        diff = z[group_index]-z
        weights = self.gamma[group_index]*np.exp(-np.sum(diff**2,axis=-1)/(2*self.sigma**2))/self.sigma**2
        return self.q[group_index]*z[group_index]-self.anchor[group_index]-np.einsum("j,jd->d",weights,diff)

    def jacobian(self, positions):
        p = _array(positions,(self.m,2),"positions")
        diff = p[:,None,:]-p[None,:,:]
        pair = (self.gamma*self.pair_overlaps(p))[:,:,None,None]*(
            diff[:,:,:,None]*diff[:,:,None,:]/self.sigma**4-np.eye(2)/self.sigma**2)
        result = -pair.copy()
        for i in range(self.m):
            result[i,i] = self.q[i]*np.eye(2)+pair[i].sum(axis=0)
        return result.transpose(0,2,1,3).reshape(2*self.m,2*self.m)

    def certificate(self):
        return {"strong_monotonicity_lower_bound":self.mu,
                "global_lipschitz_upper_bound":self.lipschitz,
                "individual_strong_convexity_lower_bounds":(self.q-self.gamma.sum(axis=1)/self.sigma**2).tolist(),
                "independent_positions":self.independent_positions.tolist()}

    def equilibrium(self):
        from scipy.optimize import root
        result = root(lambda p:self.gradient(p.reshape(self.m,2)).ravel(),
                      self.independent_positions.ravel(),jac=lambda p:self.jacobian(p.reshape(self.m,2)),tol=1e-11)
        positions = result.x.reshape(self.m,2)
        if not np.isfinite(positions).all() or np.linalg.norm(self.gradient(positions),np.inf)>1e-8:
            raise RuntimeError("Offline NE reference solve failed: " + str(result.message))
        return positions


class LeaderController:
    """Eq. (8), first-order physical leader; estimator row remains complete."""
    def __init__(self, config, group_index):
        self.config = config
        self.i = int(group_index)
        self.A,_,_ = validate_topology(config)
        self.m = len(self.A)
        if not 0 <= self.i < self.m:
            raise ValueError("Invalid group index")
        self.g = _gains(config)
        self.game = MonitoringGame(config)
        self.reset(np.zeros(2))

    def reset(self, position_xy):
        self.y = _array(position_xy,(2,),"leader position").copy()
        # This is only a locally available initializer; it does not copy other
        # leaders' positions or prescribe z_ii = y_i throughout the experiment.
        self.z = np.zeros((self.m,2))
        self.z[self.i] = self.y
        self.alpha = np.full(self.m,float(self.g["alpha_initial"]))
        self.alpha_hat = np.full(self.m,float(self.g["alpha_hat_initial"]))

    def _neighbors(self, packets):
        values = {}
        for k in range(self.m):
            if self.A[self.i,k] > 0:
                if k not in packets:
                    raise ValueError("Missing leader neighbor {}".format(k))
                values[k] = {"y":_array(packets[k]["y"],(2,),"neighbor y").copy(),
                             "z":_array(packets[k]["z"],(self.m,2),"neighbor z").copy()}
        return values

    def _rhs(self, state, neighbors):
        y = state[:2]
        z = state[2:2+2*self.m].reshape(self.m,2)
        alpha = state[2+2*self.m:2+3*self.m]
        hat = state[2+3*self.m:]
        theta = np.zeros_like(z)
        for k,packet in neighbors.items():
            theta += self.A[self.i,k]*(z-packet["z"])
            theta[k] += self.A[self.i,k]*(z[k]-packet["y"])
        dy = -float(self.g["k_L"])*self.game.local_gradient(self.i,z)
        dz = -alpha[:,None]*theta
        da = self.g["kappa_alpha"]*(np.sum(theta**2,axis=1)-self.g["sigma_alpha"]*(alpha-hat))
        dhat = self.g["delta_alpha"]*(alpha-hat)
        return np.concatenate([dy,dz.ravel(),da,dhat])

    def _state(self):
        return np.concatenate([self.y,self.z.ravel(),self.alpha,self.alpha_hat])

    def output(self, position_xy, neighbor_packets):
        position = _array(position_xy,(2,),"leader position")
        neighbors = self._neighbors(neighbor_packets)
        derivative = self._rhs(self._state(),neighbors)
        dy = derivative[:2]
        # Reconstruct theta directly rather than divide by possibly small gains.
        theta = np.zeros_like(self.z)
        for k,packet in neighbors.items():
            theta += self.A[self.i,k]*(self.z-packet["z"])
            theta[k] += self.A[self.i,k]*(self.z[k]-packet["y"])
        return {"u":dy-self.g["k_1"]*(position-self.y),"y":self.y.copy(),"z":self.z.copy(),
                "alpha":self.alpha.copy(),"alpha_hat":self.alpha_hat.copy(),"theta":theta,
                "dy":dy.copy(),"w":position.copy(),"gradient":self.game.local_gradient(self.i,self.z),
                "alpha_dot":derivative[2+2*self.m:2+3*self.m].copy(),
                "alpha_hat_dot":derivative[2+3*self.m:].copy()}

    def step(self, dt, position_xy, neighbor_packets):
        dt = _step_size(dt)
        neighbors = self._neighbors(neighbor_packets)
        state = _rk4(self._state(),dt,lambda x:self._rhs(x,neighbors))
        alpha = state[2+2*self.m:2+3*self.m]
        hat = state[2+3*self.m:]
        if (hat <= 0).any() or (alpha-hat < -1e-8).any():
            raise FloatingPointError("Adaptive alpha invariant lost; reduce controller period")
        self.y = state[:2].copy()
        self.z = state[2:2+2*self.m].reshape(self.m,2).copy()
        self.alpha,self.alpha_hat = alpha.copy(),hat.copy()
        return self.output(position_xy,neighbor_packets)


class FollowerController:
    """Eq. (45)-(48), second-order UAV horizontal acceleration controller."""
    def __init__(self, config, group_index, follower_index):
        self.config = config
        self.i,self.j = int(group_index),int(follower_index)
        _,self.A,self.pinning = validate_topology(config)
        if not 0 <= self.i < len(config["groups"]) or self.j not in (0,1):
            raise ValueError("Invalid group/follower index")
        self.g = _gains(config)
        self.reset()

    def reset(self):
        self.beta = float(self.g["beta_initial"])
        self.beta_hat = float(self.g["beta_hat_initial"])

    def output(self, tau, position_xy, velocity_xy, neighbor_w):
        x = _array(position_xy,(2,),"follower position")
        v = _array(velocity_xy,(2,),"follower velocity")
        h,dh,ddh = circle(self.config,self.i,self.j,tau)
        p,q = x-h,v-dh
        lam = self.g["lambda_follower"]
        w = p+q/lam
        error = np.zeros(2)
        for k in range(2):
            if self.A[self.j,k] > 0:
                index = k+1
                if index not in neighbor_w:
                    raise ValueError("Missing follower neighbor {}".format(index))
                error += self.A[self.j,k]*(w-_array(neighbor_w[index],(2,),"neighbor w"))
        if self.pinning[self.i,self.j] > 0:
            if 0 not in neighbor_w:
                raise ValueError("Missing leader communication signal")
            error += self.pinning[self.i,self.j]*(w-_array(neighbor_w[0],(2,),"leader w"))
        u = ddh-lam*q-lam*self.beta*error
        db = self.g["kappa_beta"]*(float(error@error)-self.g["sigma_beta"]*(self.beta-self.beta_hat))
        dhat = self.g["delta_beta"]*(self.beta-self.beta_hat)
        return {"u":u,"h":h,"dh":dh,"ddh":ddh,"p":p,"q":q,"w":w,"e":error,
                "beta":self.beta,"beta_hat":self.beta_hat,"beta_dot":float(db),"beta_hat_dot":float(dhat)}

    def step(self, dt, tau, position_xy, velocity_xy, neighbor_w):
        dt = _step_size(dt)
        out = self.output(tau,position_xy,velocity_xy,neighbor_w)
        forcing = float(out["e"]@out["e"])
        def rhs(value):
            gap = value[0]-value[1]
            return np.array([self.g["kappa_beta"]*(forcing-self.g["sigma_beta"]*gap),
                             self.g["delta_beta"]*gap])
        beta,hat = _rk4(np.array([self.beta,self.beta_hat]),dt,rhs)
        if hat <= 0 or beta-hat < -1e-8:
            raise FloatingPointError("Adaptive beta invariant lost; reduce controller period")
        self.beta,self.beta_hat = float(beta),float(hat)
        return self.output(tau,position_xy,velocity_xy,neighbor_w)
