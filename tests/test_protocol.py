import unittest
import copy
import json
from pathlib import Path
import yaml
from ne_tvft.protocol import Supervisor,decode,encode


class ProtocolTest(unittest.TestCase):
    def setUp(self):
        self.config=yaml.safe_load((Path(__file__).parents[1]/'config/experiment.yaml').read_text())
        self.s=Supervisor(self.config)

    def ready(self,now):
        for rid in self.s.ids:
            self.s.report({'version':1,'id':rid,'phase':self.s.phase,'session_id':self.s.session_id,
                           't0':self.s.t0,'stamp':now,'healthy':True,'ready':True},now)

    def advance(self,n,t):
        self.assertTrue(self.s.command(n,t)[0])
        self.ready(t+.01)
        self.s.tick(t+.01)

    def test_full_barrier_idempotence_and_stop(self):
        for i,n in enumerate([1,2,3,4]):
            self.advance(n,float(i))
        self.assertEqual(self.s.phase,'POSITIONED')
        self.assertTrue(self.s.command(5,4)[0])
        sid=self.s.session_id
        self.assertTrue(self.s.command(5,4.1)[0])
        self.assertEqual(self.s.session_id,sid)
        self.ready(4.1)
        self.s.recorder_report({'version':1,'session_id':sid,'stamp':4.1,'healthy':True,'ready':True},4.1)
        self.s.tick(4.1)
        self.assertEqual(self.s.phase,'SCHEDULED')
        self.assertGreater(self.s.t0,4.1)
        self.ready(4.2)
        self.s.tick(4.2)
        self.assertEqual(self.s.phase,'RUNNING')
        self.assertTrue(self.s.command(6,4.3)[0])
        self.ready(4.4)
        self.s.tick(4.4)
        self.assertEqual(self.s.phase,'STOPPED')

    def test_illegal_command(self):
        self.assertFalse(self.s.command(5,0)[0])
        self.assertEqual(self.s.phase,'IDLE')

    def test_stale_ready_does_not_advance(self):
        self.s.command(1,0)
        self.ready(0)
        self.s.tick(1)
        self.assertEqual(self.s.phase,'PREPARING')

    def test_missing_start_ack_aborts(self):
        self.s.phase='SCHEDULED';self.s.t0=2;self.s.session_id='x'
        self.s.tick(1.9)
        self.assertEqual(self.s.phase,'FAULT')

    def test_writer_failure_aborts(self):
        self.s.phase='RUNNING';self.s.session_id='x'
        self.ready(1)
        self.s.recorder_report({'version':1,'session_id':'x','stamp':1,'healthy':False,'ready':False},1)
        self.s.tick(1)
        self.assertEqual(self.s.phase,'FAULT')

    def test_lost_commit_ack_aborts_before_start(self):
        self.s.phase='RUNNING';self.s.session_id='s';self.s.t0=10.
        self.ready(9.9)
        self.s.reports[self.s.ids[0]][1]['phase']='SCHEDULED'
        self.s.recorder_report({'version':1,'session_id':'s','stamp':9.9,'healthy':True,'ready':True},9.9)
        self.s.tick(9.9)
        self.assertEqual(self.s.phase,'FAULT')

    def test_old_session_cannot_ack_commit(self):
        self.s.phase='RUNNING';self.s.session_id='s';self.s.t0=10.
        self.ready(9.9)
        self.s.reports[self.s.ids[0]][1]['session_id']='old'
        self.s.recorder_report({'version':1,'session_id':'s','stamp':9.9,'healthy':True,'ready':True},9.9)
        self.s.tick(9.9)
        self.assertEqual(self.s.phase,'FAULT')

    def test_clock_reset_aborts(self):
        self.s.tick(10);self.s.tick(2)
        self.assertEqual(self.s.phase,'FAULT')

    def test_nonfinite_and_protocol_rejected(self):
        for value in ['{"version":1,"x":NaN}','{"version":2}','[]']:
            self.assertRaises(ValueError,decode,value)

    def test_old_source_stamp_cannot_refresh_ready(self):
        self.assertRaises(ValueError,self.s.report,{'version':1,'id':self.s.ids[0],
            'stamp':1,'healthy':True,'ready':True,'phase':'READY'},2)

    def test_fault_not_overwritten_at_duration_deadline(self):
        self.s.phase='RUNNING';self.s.session_id='s';self.s.t0=1.
        now=200.
        self.ready(now)
        self.s.reports[self.s.ids[0]][1]['healthy']=False
        self.s.recorder_report({'version':1,'session_id':'s','stamp':now,'healthy':True,'ready':True},now)
        self.s.tick(now)
        self.assertEqual(self.s.phase,'FAULT')

    def test_unhealthy_airborne_agent_aborts_all(self):
        self.s.phase='AIRBORNE';self.ready(2)
        self.s.reports[self.s.ids[0]][1]['healthy']=False
        self.s.tick(2)
        self.assertEqual(self.s.phase,'FAULT')


if __name__=='__main__':
    unittest.main()
