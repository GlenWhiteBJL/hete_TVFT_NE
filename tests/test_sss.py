import unittest
from ne_tvft.sss import outputs_allowed
from ne_tvft.communication import DelayBuffer


class SSSBackendTests(unittest.TestCase):
    def test_simulation_does_not_need_hardware_verification(self):
        self.assertTrue(outputs_allowed('sss',False,False,{},True,True))
        self.assertFalse(outputs_allowed('hardware',False,False,{},True,True))
        self.assertFalse(outputs_allowed('hardware',False,True,{},False,False))
        flags={k:True for k in ('enabled','frames_verified','acceleration_interface_verified')}
        self.assertTrue(outputs_allowed('hardware',False,True,flags,False,False))

    def test_backend_mismatch_and_absent_clock_fail_closed(self):
        for args in [('sss',True,False,{},True,True),('sss',False,True,{},True,True),
                     ('sss',False,False,{},False,True),('sss',False,False,{},True,False),
                     ('mock',False,False,{},False,False),('unknown',False,False,{},False,False)]:
            with self.assertRaises(ValueError): outputs_allowed(*args)

    def test_ros_delay_uses_sim_time_but_preserves_wall_receipt(self):
        b=DelayBuffer({'enabled':True,'distribution':'fixed','delay_s':.1},'uav1')
        b.push('uav2',{'stamp':20},receipt=9000,clock=20)
        self.assertEqual(b.release(20.09),[])
        delivered=b.release(20.1)
        self.assertEqual(delivered,[('uav2',{'stamp':20},9000)])


if __name__=='__main__': unittest.main()
