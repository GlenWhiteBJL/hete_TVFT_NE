"""Optional SSS backend: explicit output selection and a Python clock client.

Each timer requests its next ROS deadline only after finishing its callback.
Wall-clock freshness watchdogs remain active; pausing a running experiment can
therefore fault on resume rather than silently weakening those checks.
"""
import threading
import time


def outputs_allowed(backend, mock, allow_hardware, hardware, sim_time, sss_time):
    if backend not in ("mock", "sss", "hardware"):
        raise ValueError("backend must be auto, mock, sss or hardware")
    if mock != (backend == "mock"):
        raise ValueError("mock flag and backend disagree")
    if backend == "mock":
        return True
    if backend == "sss":
        if allow_hardware or not (sim_time and sss_time):
            raise ValueError("SSS requires both simulation clocks and allow_hardware=false")
        return True
    return bool(allow_hardware and all(hardware.get(k, False) for k in
        ("enabled", "frames_verified", "acceleration_interface_verified")))


def create_timer(period, callback, backend, reset=False):
    import rospy
    if backend == "sss":
        return SSSTimer(period, callback)
    return rospy.Timer(rospy.Duration(period), callback, reset=reset)


class SSSTimer:
    """One registered SSS time client per callback thread, using integer ROS time."""
    def __init__(self, period, callback):
        import rospy
        from rosgraph_msgs.msg import Clock
        from sss_sim_env.msg import TimeRequest
        from sss_sim_env.srv import ClientRegister, ClientUnregister
        self.rospy, self.callback = rospy, callback
        self.period = rospy.Duration(period)
        self.stop = threading.Event()
        self.wake = threading.Event()
        self.publisher = rospy.Publisher('/sss_time_client/update_clock_request', TimeRequest, queue_size=10, latch=True)
        self.subscriber = rospy.Subscriber('/clock', Clock, lambda _msg: self.wake.set(), queue_size=10)
        rospy.wait_for_service('/sss_timeclient_register', timeout=10)
        self.unregister = rospy.ServiceProxy('/sss_timeclient_unregister', ClientUnregister)
        response = rospy.ServiceProxy('/sss_timeclient_register', ClientRegister)()
        if not response.success:
            raise RuntimeError('SSS time client registration failed')
        self.client_id = response.client_id
        self.message_type = TimeRequest
        deadline = time.monotonic()+5
        while not self.publisher.get_num_connections() and time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(.01)
        if not self.publisher.get_num_connections():
            self.unregister(self.client_id)
            raise RuntimeError('SSS clock request subscriber unavailable')
        self.thread = threading.Thread(target=self.run, name='sss_clock_timer', daemon=True)
        rospy.on_shutdown(self.shutdown)
        self.thread.start()

    def run(self):
        ros = self.rospy
        target = ros.Time.now()+self.period
        try:
            while not self.stop.is_set() and not ros.is_shutdown():
                self.publisher.publish(self.message_type(time_client_id=self.client_id, request_time=target))
                while ros.Time.now() < target and not self.stop.is_set() and not ros.is_shutdown():
                    self.wake.wait(.05)
                    self.wake.clear()
                if self.stop.is_set() or ros.is_shutdown():
                    break
                self.callback(None)
                target = target+self.period
        except Exception as exc:
            ros.logerr('SSS timer failed: %s', exc)
            ros.signal_shutdown('SSS timer failed')

    def shutdown(self):
        if self.stop.is_set():
            return
        self.stop.set()
        self.wake.set()
        try:
            self.unregister(self.client_id)
        except Exception:
            pass
