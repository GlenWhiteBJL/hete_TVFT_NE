import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
import yaml
from ne_tvft.safety import GroupSafety, SafetyError, project_halfplanes

ROOT = Path(__file__).resolve().parents[1]


class GroupSafetyTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT/'config/sss_experiment.yaml').read_text())
        self.model = GroupSafety(self.config)

    def test_projection_intersection_and_infeasibility(self):
        result = project_halfplanes([-1, -2], [[1, 0], [0, 1]], [0, 0])
        np.testing.assert_allclose(result, [0, 0])
        with self.assertRaises(SafetyError):
            project_halfplanes([0, 0], [[1, 0], [-1, 0]], [1, 1])
        with self.assertRaises(SafetyError):
            project_halfplanes([0, 0], [[0, 0]], [1])

    def test_normal_orbit_is_unchanged_and_limits_are_satisfied(self):
        u, diagnostic = self.model.filter([-.005, 0], [.5, 0, 1.2], [0, .05, 0],
            [-.5, 0, 1.2], [0, -.05, 0], [0, 0, 0], [0, 0, 0])
        np.testing.assert_allclose(u, [-.005, 0])
        self.assertFalse(diagnostic['active'])
        u, _ = self.model.filter([10, 10], [.5, 0, 1.2], [0, 0, 0],
            [-.5, 0, 1.2], [0, 0, 0], [0, 0, 0], [0, 0, 0])
        self.assertLessEqual(np.linalg.norm(u), self.config['limits']['uav_acceleration']+1e-8)

    def test_reciprocal_rows_jointly_satisfy_pair_barrier(self):
        positions = np.array([[-.3, 0, 1.2], [.3, 0, 1.2]])
        velocities = np.array([[.15, 0, 0], [-.15, 0, 0]])
        outputs = [self.model.filter([0, 0], positions[j], velocities[j], positions[1-j],
            velocities[1-j], [0, 0, 0], [0, 0, 0])[0] for j in range(2)]
        p, v = positions[0, :2]-positions[1, :2], velocities[0, :2]-velocities[1, :2]
        h = p@p-self.model.distance**2
        residual = (2*v@v+2*p@(outputs[0]-outputs[1])+
                    (self.model.k1+self.model.k2)*2*p@v+self.model.k1*self.model.k2*h)
        self.assertGreaterEqual(residual, -1e-7)

    def test_head_on_approach_with_actuator_lag_keeps_separation(self):
        config = copy.deepcopy(self.config);config['limits']['uav_speed'] = .6
        model = GroupSafety(config)
        position = np.array([[-.5, 0, 1.2], [.5, 0, 1.2]])
        velocity, actual = np.zeros((2, 3)), np.zeros((2, 2))
        minimum, active = 1., 0
        for _ in range(600):
            nominal = 2*(np.array([[.2, 0], [-.2, 0]])-velocity[:, :2])
            outputs = []
            for j in range(2):
                output, diagnostic = model.filter(nominal[j], position[j], velocity[j], position[1-j],
                    velocity[1-j], [0, 0, 0], [0, 0, 0])
                outputs.append(output);active += diagnostic['active']
            actual += .01/.08*(np.array(outputs)-actual)
            position[:, :2] += .01*velocity[:, :2]+.5*.01**2*actual
            velocity[:, :2] += .01*actual
            minimum = min(minimum, np.linalg.norm(position[0, :2]-position[1, :2]))
        self.assertGreaterEqual(minimum, model.distance)
        self.assertGreater(active, 0)

    def test_low_altitude_vehicle_envelope_and_high_altitude_freedom(self):
        geo = self.model.geometry([0, 0, 1.2], [1, 0, 1.2], [0, 0, 0])
        self.assertGreater(geo['ugv_barrier_h'], 0)
        position = np.array([.8, 0, .1]);velocity = np.array([-.2, 0, 0])
        for _ in range(400):
            output, _ = self.model.filter([0, 0], position, velocity, [-1, 0, .1], [0, 0, 0],
                [0, 0, 0], [0, 0, 0], leader_stationary=True)
            position[:2] += .01*velocity[:2]+.5*.01**2*output
            velocity[:2] += .01*output
            self.assertGreaterEqual(self.model.geometry(position, [-1, 0, .1], [0, 0, 0])['ugv_barrier_h'], -1e-7)

    def test_invalid_geometry_and_nonfinite_inputs_are_rejected(self):
        bad = copy.deepcopy(self.config);bad['safety']['ugv_horizontal_radius'] = .1
        with self.assertRaises(SafetyError): GroupSafety(bad)
        bad = copy.deepcopy(self.config);bad['formation']['radii'] = [.2]*3
        with self.assertRaises(SafetyError): GroupSafety(bad)
        with self.assertRaises(SafetyError):
            self.model.filter([0, 0], [float('nan'), 0, 0], [0, 0, 0], [1, 0, 0],
                              [0, 0, 0], [0, 0, 0], [0, 0, 0])

    def test_transfer_guard_accepts_takeoff_but_rejects_swaps_and_car_descent(self):
        self.model.guard_transfer([.5,0,0], [-.5,0,0], [0,0,0], [.5,0,1.2], [-.5,0,1.2])
        with self.assertRaises(SafetyError):
            self.model.guard_transfer([.5,0,1.2], [-.5,0,1.2], [0,0,0], [-.5,0,1.2], [.5,0,1.2])
        with self.assertRaises(SafetyError):
            self.model.guard_transfer([0,0,1.2], [1,0,1.2], [0,0,0], [0,0,0], [1,0,0])


class DistanceAnalysisTests(unittest.TestCase):
    def test_all_three_pairs_keep_unmatched_data_missing(self):
        spec = importlib.util.spec_from_file_location('distance_analysis', ROOT/'scripts/analyze_experiment.py')
        module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        group = {'leader':'g', 'followers':['a','b']}
        rows = [{'id':'a','stamp':10.,'position':[.5,0,1.2]},
                {'id':'a','stamp':11.,'position':[.5,0,1.2]},
                {'id':'b','stamp':10.,'position':[-.5,0,1.2],'safety':{'active':True}},
                {'id':'g','stamp':10.,'position':[0,0,0]}]
        series = module.intra_group_series(rows, group, 10., .05)
        self.assertEqual(len(series['pairs']), 3)
        self.assertAlmostEqual(series['pairs']['a--b']['distance_3d'][0], 1.)
        self.assertAlmostEqual(series['pairs']['a--g']['distance_3d'][0], 1.3)
        self.assertTrue(np.isnan(series['pairs']['a--b']['distance_3d'][1]))
        self.assertEqual(series['active'].tolist(), [True,False])

    def test_recording_validation_requires_healthy_feasible_safety_diagnostics(self):
        sys.path.insert(0,str(ROOT/'scripts'))
        from smoke_sss import validate_sss_inputs,SmokeFailure
        packet={'id':'uav1','session_id':'s','phase':'RUNNING','stamp':10.02,'t0':10.,
                'start_offset_s':.02,'backend':'sss','communication_clock':'ros','dt':.02,
                'command_mode':'acceleration','setpoint':{'type_mask':2111}}
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'telemetry.jsonl'
            p.write_text(json.dumps(packet)+'\n')
            with self.assertRaises(SmokeFailure): validate_sss_inputs(directory,'s',10.,['uav1'],True)
            packet['safety']={'enabled':True,'healthy':True,'minimum_constraint_margin':0.}
            p.write_text(json.dumps(packet)+'\n')
            validate_sss_inputs(directory,'s',10.,['uav1'],True)
            packet['safety']['minimum_constraint_margin']=-.01
            p.write_text(json.dumps(packet)+'\n')
            with self.assertRaises(SmokeFailure): validate_sss_inputs(directory,'s',10.,['uav1'],True)


if __name__ == '__main__':
    unittest.main()
