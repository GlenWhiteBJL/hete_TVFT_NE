import unittest
from ne_tvft.adapters import neighbor_sample_is_usable


class NeighborExpiryTests(unittest.TestCase):
    def test_sss_clock_delay_preserves_disabled_age_guard(self):
        import test_adapters as adapters
        from ne_tvft.communication import DelayBuffer
        from unittest.mock import patch
        f=adapters.AgentNodeTests('runTest');f.setUp();self.addCleanup(f.tearDown)
        n=f.node;n.runtime['signal_timeout']=False;n.communication_clock='ros'
        n.communication=DelayBuffer({'enabled':True,'distribution':'fixed','delay_s':.25},n.agent_id)
        with patch.object(f.module.time,'monotonic',return_value=100.): f.neighbors()
        self.assertFalse(n._signals(100.24,100.24)[1])
        self.assertTrue(n._signals(100.5,100.5)[1])

    def test_disabled_expiry_accepts_aged_data(self):
        self.assertTrue(neighbor_sample_is_usable(20., 15., 5., False))
        self.assertFalse(neighbor_sample_is_usable(20., 15., 5., .3))

    def test_invalid_timestamps_are_still_rejected(self):
        for now,age,stamp in [(20.,0.,0.),(20.,0.,21.),(20.,-1.,19.),
                              (float('nan'),0.,19.),(20.,float('inf'),19.)]:
            self.assertFalse(neighbor_sample_is_usable(now,age,stamp,False))

    def test_node_holds_aged_neighbor_but_rejects_old_session(self):
        import test_adapters as adapters
        f=adapters.AgentNodeTests('runTest');f.setUp();self.addCleanup(f.tearDown)
        f.node.runtime['signal_timeout']=False
        from unittest.mock import patch
        with patch.object(f.module.time,'monotonic',return_value=100.): f.neighbors()
        self.assertTrue(f.node._signals(f.ros.clock+10.,110.)[1])
        aid=next(iter(f.node.neighbors))
        f.node.neighbors[aid][0]['session_id']='another_session'
        self.assertFalse(f.node._signals(f.ros.clock+10.,110.)[1])


if __name__=='__main__': unittest.main()
