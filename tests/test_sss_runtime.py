import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from smoke_sss import check_sss_graph,validate_sss_inputs,SmokeFailure


class SSSRuntimeGuards(unittest.TestCase):
    def setUp(self):
        root=Path(__file__).resolve().parents[1]
        self.config=yaml.safe_load((root/'config/sss_experiment.yaml').read_text())
        self.config.update(yaml.safe_load((root/'config/topics.yaml').read_text()))
        self.ids=list(self.config['robots'])
        self.params={'/use_sim_time':True,'/use_sss_sim_time':True}
        pub=[];sub=[('/command',['/supervisor','/recorder'])];srv=[]
        for rid,r in self.config['robots'].items():
            node='/agent_'+rid
            self.params.update({node+'/backend':'sss',node+'/mock':False,node+'/allow_hardware':False})
            pub.extend([(r['command_topic'],[node]),(r['pose_topic'],['/nodelet_manager']),
                        (r['velocity_topic'],['/nodelet_manager'])])
            sub.extend([(r['command_topic'],['/nodelet_manager']),(r['pose_topic'],[node]),(r['velocity_topic'],[node])])
            if r['kind']=='uav':
                pub.append((r['extended_state_topic'],['/nodelet_manager']))
                srv.extend([(r['mode_service'],['/nodelet_manager']),(r['arming_service'],['/nodelet_manager'])])
        self.graph=[pub,sub,srv]

    def test_expected_sss_graph_is_accepted(self):
        self.assertEqual(set(check_sss_graph(self.config,self.params,self.graph)),set(self.ids))

    def test_contested_actuator_or_foreign_sensor_is_rejected(self):
        for idx in (0,1):
            graph=copy.deepcopy(self.graph)
            topic,nodes=graph[0][idx]
            graph[0][idx]=(topic,nodes+['/foreign_node'])
            with self.assertRaises(SmokeFailure): check_sss_graph(self.config,self.params,graph)

    def test_hardware_agent_or_unrelated_command_subscriber_is_rejected(self):
        params=dict(self.params);params['/agent_uav1/backend']='hardware'
        with self.assertRaises(SmokeFailure): check_sss_graph(self.config,params,self.graph)
        graph=copy.deepcopy(self.graph);graph[1][0][1].append('/another_controller')
        with self.assertRaises(SmokeFailure): check_sss_graph(self.config,self.params,graph)

    def test_recorded_mixed_targets_cannot_pass_as_acceleration_control(self):
        packets=[{'id':rid,'session_id':'test','phase':'RUNNING','stamp':10.02,'t0':10.,
            'start_offset_s':.02,'backend':'sss','communication_clock':'ros','dt':.02,
            'command_mode':'acceleration' if rid.startswith('uav') else 'velocity',
            'setpoint':{'type_mask':2111}} for rid in self.ids]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'telemetry.jsonl'
            path.write_text(''.join(json.dumps(p)+'\n' for p in packets))
            self.assertEqual(validate_sss_inputs(directory,'test',10.,self.ids)['uav_type_mask'],2111)
            next(p for p in packets if p['id']=='uav1')['setpoint']['type_mask']=2552
            path.write_text(''.join(json.dumps(p)+'\n' for p in packets))
            with self.assertRaises(SmokeFailure): validate_sss_inputs(directory,'test',10.,self.ids)


if __name__=='__main__': unittest.main()
