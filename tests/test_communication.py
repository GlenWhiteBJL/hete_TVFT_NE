import unittest
import copy
import importlib.util
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
import yaml
from ne_tvft.communication import DelayBuffer


class CommunicationTests(unittest.TestCase):
    def test_normal_is_bounded_reproducible_with_expected_moments(self):
        settings={"enabled":True,"distribution":"normal","delay_s":.05,"std_s":.015,
                  "max_delay_s":.1,"seed":15}
        q, repeat=DelayBuffer(settings,"b"), DelayBuffer(settings,"b")
        values=[]
        for i in range(3000):
            q.push("a",{},float(i)); repeat.push("a",{},float(i))
            self.assertEqual(q.pending[-1],repeat.pending[-1])
            q.release(i+1.); repeat.release(i+1.)
        stat=q.diagnostics()["generated_per_link"]["a->b"]
        mean=stat["sum_s"]/stat["count"]
        std=(stat["sum_sq_s2"]/stat["count"]-mean**2)**.5
        self.assertLess(abs(mean-.05),.001)
        self.assertLess(abs(std-.015),.001)
        self.assertGreater(stat["min_s"],0.)
        self.assertLess(stat["max_s"],.1)
        self.assertEqual(sum(stat["histogram"]),3000)

    def test_normal_invalid_bounds_rejected(self):
        with self.assertRaises(ValueError):
            DelayBuffer({"distribution":"normal","delay_s":.2,"max_delay_s":.1})

    def test_boundary_and_original_timestamp_and_receipt(self):
        queue = DelayBuffer({"enabled": True, "delay_s": .05}, "uav1")
        packet = {"stamp": 20., "w": [1., 2.]}
        queue.push("ugv1", packet, 100.)
        packet["w"][0] = 99
        self.assertEqual(queue.release(100.049), [])
        sender, delivered, receipt = queue.release(100.05)[0]
        self.assertEqual((sender, delivered["stamp"], delivered["w"], receipt),
                         ("ugv1", 20., [1., 2.], 100.))

    def test_disabled_matches_immediate_delivery(self):
        q = DelayBuffer({"enabled": False, "delay_s": 100.})
        q.push("a", {"stamp": 1.}, 10.)
        self.assertEqual(len(q.release(10.)), 1)

    def test_link_override_seed_and_clear(self):
        settings = {"enabled": True, "delay_s": .05, "jitter_s": .01,
                    "links": {"a->b": .1}, "seed": 12}
        a, b = DelayBuffer(settings, "b"), DelayBuffer(settings, "b")
        for q in (a, b):
            for i in range(10):
                q.push("a", {"stamp": i}, i)
        self.assertEqual(a.pending, b.pending)
        self.assertTrue(all(.09 <= p[0]-p[4] <= .11 for p in a.pending))
        a.clear()
        self.assertEqual(a.release(100.), [])

    def test_capacity_and_invalid_config_are_explicit(self):
        q = DelayBuffer({"max_pending": 1})
        q.push("a", {}, 1.)
        with self.assertRaises(OverflowError):
            q.push("a", {}, 1.)
        for value in (-1., float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                DelayBuffer({"delay_s": value})

    def test_agent_delays_neighbors_and_keeps_source_age_watchdog(self):
        import test_adapters as adapters
        fixture = adapters.AgentNodeTests("runTest")
        fixture.setUp()
        self.addCleanup(fixture.tearDown)
        node = fixture.node
        node.communication = DelayBuffer({"enabled": True,"delay_s":.1},node.agent_id)
        with patch.object(fixture.module.time,"monotonic",return_value=100.):
            fixture.neighbors()
        self.assertFalse(node._signals(fixture.ros.clock,100.099)[1])
        self.assertTrue(node._signals(fixture.ros.clock+.1,100.1)[1])
        # Receipt stays 100: delivering a packet does not make old data fresh.
        self.assertFalse(node._signals(fixture.ros.clock+.31,100.31)[1])
        aid = next(iter(node.expected_ids))
        stamp = node.neighbors[aid][0]["stamp"]
        packet={"version":1,"id":aid,"session_id":node.session["session_id"],
                "stamp":stamp-.01,"t0":node.session["t0"],"w":[9.,9.]}
        with patch.object(fixture.module.time,"monotonic",return_value=101.):
            node.signal_callback(aid,adapters.FakeString(json.dumps(packet)))
        node._signals(fixture.ros.clock+.2,101.2)
        self.assertEqual(node.neighbors[aid][0]["stamp"],stamp)
        self.assertEqual(node.communication_out_of_order,1)

    def test_zero_delay_offline_states_equal_unmodified_sampling(self):
        root=Path(__file__).resolve().parents[1]
        spec=importlib.util.spec_from_file_location("zero_delay_demo",root/"scripts/offline_demo.py")
        demo=importlib.util.module_from_spec(spec);spec.loader.exec_module(demo)
        config=yaml.safe_load((root/"config/experiment.yaml").read_text(encoding="utf-8"))
        baseline=demo.simulate(config,.2,.01)
        delayed=copy.deepcopy(config)
        delayed["communication"]={"enabled":True,"delay_s":0.}
        candidate=demo.simulate(delayed,.2,.01)
        np.testing.assert_array_equal(baseline["final_xl"],candidate["final_xl"])
        np.testing.assert_array_equal(baseline["final_xf"],candidate["final_xf"])


if __name__ == "__main__":
    unittest.main()
