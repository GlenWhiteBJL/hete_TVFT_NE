"""Nonblocking, bounded receive-side delay; source timestamps are immutable."""
import copy
import hashlib
import heapq
import math
import random


class DelayBuffer:
    def __init__(self, config=None, receiver=""):
        config = config or {}
        self.enabled = config.get("enabled", False)
        if not isinstance(self.enabled, bool):
            raise ValueError("communication.enabled must be boolean")
        self.fixed = self._seconds(config.get("delay_s", 0.0))
        self.jitter = self._seconds(config.get("jitter_s", 0.0))
        self.distribution = config.get("distribution", "uniform")
        if self.distribution not in {"fixed", "uniform", "normal"}:
            raise ValueError("Unknown communication distribution")
        self.std = self._seconds(config.get("std_s", 0.0))
        self.minimum = self._seconds(config.get("min_delay_s", 0.0))
        self.maximum = self._seconds(config.get("max_delay_s", .1))
        self.links = {str(k): self._seconds(v) for k, v in config.get("links", {}).items()}
        if self.distribution == "normal" and (self.maximum <= self.minimum or
                any(not self.minimum <= mean <= self.maximum for mean in [self.fixed]+list(self.links.values()))):
            raise ValueError("Normal delay means must lie within positive-width truncation bounds")
        self.limit = config.get("max_pending", 4096)
        if isinstance(self.limit, bool) or not isinstance(self.limit, int) or self.limit < 1:
            raise ValueError("max_pending must be a positive integer")
        self.receiver = receiver
        self.seed = int(config.get("seed", 7))
        self.generators = {}
        self.clear()

    @staticmethod
    def _seconds(value):
        value = float(value)
        if not math.isfinite(value) or value < 0:
            raise ValueError("Delay and jitter must be finite nonnegative seconds")
        return value

    def clear(self):
        self.pending = []
        self.sequence = 0
        self.generators.clear()
        self.delivered = 0
        self.last_injected_s = 0.0
        self.generated = {}

    def push(self, sender, payload, receipt, clock=None):
        schedule_time = receipt if clock is None else clock
        if not math.isfinite(receipt) or not math.isfinite(schedule_time):
            raise ValueError("Invalid receipt clock")
        if len(self.pending) >= self.limit:
            raise OverflowError("Communication delay queue capacity exceeded")
        key = sender + "->" + self.receiver
        if key not in self.generators:
            digest = hashlib.sha256((str(self.seed) + ":" + key).encode()).digest()
            self.generators[key] = random.Random(int.from_bytes(digest[:8], "big"))
        delay = 0.0
        if self.enabled:
            mean, rng = self.links.get(key, self.fixed), self.generators[key]
            if self.distribution == "normal":
                # Rejection sampling is truncation, not clipping mass onto endpoints.
                for _ in range(10000):
                    delay = rng.gauss(mean, self.std)
                    if self.minimum <= delay <= self.maximum:
                        break
                else:
                    raise ValueError("Normal delay rejection sampler did not converge")
            elif self.distribution == "fixed":
                delay = mean
            else:
                delay = max(0.0, mean + rng.uniform(-self.jitter, self.jitter))
        stat = self.generated.setdefault(key, {"count":0,"sum_s":0.,"sum_sq_s2":0.,
                    "min_s":delay,"max_s":delay,"histogram":[0]*20})
        stat["count"] += 1
        stat["sum_s"] += delay
        stat["sum_sq_s2"] += delay*delay
        stat["min_s"], stat["max_s"] = min(stat["min_s"],delay), max(stat["max_s"],delay)
        # Histogram is over [0,max_delay_s]; final bin also contains overflow for nonnormal profiles.
        stat["histogram"][min(19,int(delay/max(self.maximum,1e-12)*20))] += 1
        self.sequence += 1
        heapq.heappush(self.pending, (schedule_time + delay, self.sequence, sender,
                                     copy.deepcopy(payload), receipt, delay))

    def release(self, now):
        result = []
        while self.pending and self.pending[0][0] <= now:
            due, _seq, sender, payload, receipt, delay = heapq.heappop(self.pending)
            result.append((sender, payload, receipt))
            self.delivered += 1
            self.last_injected_s = delay
        return result

    def diagnostics(self):
        return {"enabled": self.enabled, "pending": len(self.pending),
                "delivered": self.delivered, "last_injected_s": self.last_injected_s,
                "distribution":self.distribution,"histogram_max_s":self.maximum,
                "generated_per_link":copy.deepcopy(self.generated)}


class SampledNetwork:
    """Offline sample-and-hold links, with constant initial prehistory."""
    def __init__(self, config):
        self.config = config
        self.links = {}

    def value(self, sender, receiver, payload, now):
        key = (sender, receiver)
        if key not in self.links:
            self.links[key] = [DelayBuffer(self.config, receiver), -math.inf, copy.deepcopy(payload)]
        link = self.links[key]
        link[0].push(sender, {"stamp": now, "value": payload}, now)
        for _sender, packet, _receipt in link[0].release(now):
            if packet["stamp"] > link[1]:
                link[1], link[2] = packet["stamp"], packet["value"]
        return copy.deepcopy(link[2])
