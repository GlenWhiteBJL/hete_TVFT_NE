"""Independent mathematical/protocol checks; no ROS installation required."""
import copy
import math
from pathlib import Path
import sys
import unittest
import tempfile
import importlib.util
import json
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from ne_tvft.core import LeaderController,FollowerController,MonitoringGame,circle,validate_topology


def example_config():
    return {"groups":[{"leader":"ugv{}".format(i+1),"followers":["uav{}".format(2*i+1),"uav{}".format(2*i+2)],
                       "initial_xy":[-1.5+i*1.5,-1.0],"altitude":1.2} for i in range(3)],
            "monitoring":{"hotspots":[[-1,-.5],[1,-.5],[0,1]],"stations":[[-2,-1],[2,-1],[0,2]],
                          "task_weights":[1.,1.1,.9],"station_weights":[.1,.1,.1],
                          "overlap_weights":[[0,.2,.2],[.2,0,.2],[.2,.2,0]],"overlap_sigma":1.2},
            "topology":{"leaders":[[0,1,0],[1,0,1],[0,1,0]],"followers":[[0,1],[1,0]],"pinning":[[1,0],[1,0],[1,0]]},
            "formation":{"radii":[.35]*3,"angular_rates":[.1,.12,-.1],"phases":[0,.5,1.]},
            "gains":{"k_L":.5,"k_1":2.,"lambda_follower":1.5,
                     "kappa_alpha":.25,"sigma_alpha":1.,"delta_alpha":.5,"alpha_initial":2.,"alpha_hat_initial":1.,
                     "kappa_beta":.5,"sigma_beta":1.,"delta_beta":.5,"beta_initial":2.,"beta_hat_initial":1.}}


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.c = example_config()
        self.game = MonitoringGame(self.c)

    def test_cost_gradient_finite_difference(self):
        p = np.array([[-.3,.1],[.6,-.2],[-.1,.8]])
        expected = self.game.gradient(p)
        for i in range(3):
            for d in range(2):
                delta = np.zeros_like(p)
                delta[i,d]=1e-5
                fd = (self.game.costs(p+delta)[i]-self.game.costs(p-delta)[i])/(2e-5)
                self.assertAlmostEqual(fd,expected[i,d],places=8)

    def test_local_gradient_uses_only_own_row(self):
        rng = np.random.default_rng(4)
        z = rng.normal(size=(3,3,2))
        for i in range(3):
            np.testing.assert_allclose(self.game.local_gradient(i,z[i]),self.game.gradient(z[i])[i],atol=1e-14)

    def test_global_certificate_and_ne(self):
        certificate = self.game.certificate()
        self.assertGreater(certificate["strong_monotonicity_lower_bound"],0)
        ne = self.game.equilibrium()
        self.assertLess(np.linalg.norm(self.game.gradient(ne)),1e-8)
        rng = np.random.default_rng(7)
        for scale in (0.,.1,1.,10.):
            p = rng.normal(size=(3,2))*scale
            eig = np.linalg.eigvalsh(self.game.jacobian(p))
            self.assertGreaterEqual(eig.min(),self.game.mu-1e-12)
            self.assertLessEqual(eig.max(),self.game.lipschitz+1e-12)

    def test_jacobian_finite_difference(self):
        p = np.array([[0,0],[.1,.2],[-.3,.4]])
        x = p.ravel()
        finite = np.empty((6,6))
        for k in range(6):
            d = np.eye(6)[k]*1e-5
            finite[:,k]=(self.game.gradient((x+d).reshape(3,2)).ravel()-self.game.gradient((x-d).reshape(3,2)).ravel())/2e-5
        np.testing.assert_allclose(finite,self.game.jacobian(p),atol=1e-9)

    def test_circle_opposition_and_derivatives(self):
        t,eps = 2.3,1e-5
        h,dh,ddh = circle(self.c,1,0,t)
        h2,dh2,ddh2 = circle(self.c,1,1,t)
        np.testing.assert_allclose(h2,-h,atol=1e-14)
        np.testing.assert_allclose(dh2,-dh,atol=1e-14)
        np.testing.assert_allclose(ddh2,-ddh,atol=1e-14)
        hp,dhp,_ = circle(self.c,1,0,t+eps)
        hm,dhm,_ = circle(self.c,1,0,t-eps)
        np.testing.assert_allclose((hp-hm)/(2*eps),dh,atol=1e-10)
        np.testing.assert_allclose((dhp-dhm)/(2*eps),ddh,atol=1e-10)
        self.assertAlmostEqual(np.linalg.norm(h),.35)

    def test_disconnected_leader_graph_rejected(self):
        c = copy.deepcopy(self.c)
        c["topology"]["leaders"] = [[0,0,0],[0,0,1],[0,1,0]]
        with self.assertRaises(ValueError):
            validate_topology(c)

    def test_missing_leader_root_rejected(self):
        c = copy.deepcopy(self.c)
        c["topology"]["pinning"][1]=[0,0]
        with self.assertRaises(ValueError):
            validate_topology(c)

    def test_leader_equations_independent_loop(self):
        leader = LeaderController(self.c,1)
        leader.reset([.4,.7])
        leader.z = np.array([[.1,.2],[.4,.5],[.6,.7]])
        packets={0:{"y":[-.4,-.1],"z":np.zeros((3,2))},2:{"y":[.8,.5],"z":np.ones((3,2))}}
        output=leader.output([.2,.3],packets)
        theta=np.zeros((3,2))
        for ell in range(3):
            for k in (0,2):
                theta[ell]+=leader.z[ell]-packets[k]["z"][ell]
            if ell in packets:
                theta[ell]+=leader.z[ell]-packets[ell]["y"]
        np.testing.assert_allclose(output["theta"],theta)
        expected=-.5*self.game.gradient(leader.z)[1]-2*(np.array([.2,.3])-leader.y)
        np.testing.assert_allclose(output["u"],expected)
        with self.assertRaises(ValueError):
            leader.output([.2,.3],{0:packets[0]})

    def test_non_neighbor_packets_are_not_used(self):
        leader=LeaderController(self.c,0)
        packets={1:{"y":[.1,.2],"z":np.ones((3,2))}}
        before=leader.output([0,0],packets)
        packets[2]={"y":[1e5,1e5],"z":np.full((3,2),-1e5)}
        after=leader.output([0,0],packets)
        for name in ("u","theta","dy"):
            np.testing.assert_array_equal(before[name],after[name])

    def test_output_does_not_mutate_and_returns_copies(self):
        leader=LeaderController(self.c,0)
        packets={1:{"y":[0,0],"z":np.zeros((3,2))}}
        old=leader._state().copy()
        output=leader.output([.1,.2],packets)
        output["z"][:]=900
        np.testing.assert_array_equal(old,leader._state())

    def test_follower_physical_w_identity(self):
        controller=FollowerController(self.c,0,0)
        v=np.array([.1,-.2])
        output=controller.output(.7,[.4,.5],v,{0:np.array([.2,.3]),2:np.array([-.3,.4])})
        dw=v-output["dh"]+(output["u"]-output["ddh"])/self.c["gains"]["lambda_follower"]
        np.testing.assert_allclose(dw,-controller.beta*output["e"],atol=1e-14)
        self.assertNotIn("y",output)

    def test_unpinned_follower_does_not_need_leader_signal(self):
        controller=FollowerController(self.c,0,1)
        output=controller.output(0.,[0,0],[0,0],{1:np.array([.1,.2])})
        self.assertEqual(output["u"].shape,(2,))
        with self.assertRaises(ValueError):
            controller.output(0.,[0,0],[0,0],{})

    def test_adaptive_invariants_for_small_period(self):
        leader=LeaderController(self.c,0)
        follower=FollowerController(self.c,0,1)
        packets={1:{"y":[.1,.2],"z":np.ones((3,2))*.1}}
        for _ in range(200):
            old_ahat=leader.alpha_hat.copy()
            old_bhat=follower.beta_hat
            leader.step(.02,[0,0],packets)
            follower.step(.02,1.,[.4,.2],[0,0],{1:np.array([.1,.2])})
            self.assertTrue((leader.alpha>=leader.alpha_hat-1e-10).all())
            self.assertTrue((leader.alpha_hat>=old_ahat-1e-12).all())
            self.assertGreaterEqual(follower.beta,follower.beta_hat-1e-10)
            self.assertGreaterEqual(follower.beta_hat,old_bhat-1e-12)

    def test_invalid_period_or_state_rejected(self):
        follower=FollowerController(self.c,0,1)
        for dt in (0,-.1,float("nan")):
            with self.assertRaises(ValueError):
                follower.step(dt,0,[0,0],[0,0],{1:[0,0]})
        with self.assertRaises(ValueError):
            follower.output(0,[float("nan"),0],[0,0],{1:[0,0]})

    def test_online_initialization_does_not_solve_ne(self):
        original=MonitoringGame.equilibrium
        def prohibited(_):
            raise RuntimeError("No online NE oracle")
        MonitoringGame.equilibrium=prohibited
        try:
            controller=LeaderController(self.c,0)
            controller.output([0,0],{1:{"y":[0,0],"z":np.zeros((3,2))}})
        finally:
            MonitoringGame.equilibrium=original

    def test_bounded_demo_logs_actual_applied_commands(self):
        path=Path(__file__).resolve().parents[1]/"scripts"/"offline_demo.py"
        spec=importlib.util.spec_from_file_location("offline_demo_test",str(path))
        demo=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(demo)
        config=copy.deepcopy(self.c)
        config["limits"]={"ugv_speed":.01,"uav_acceleration":.01,"uav_speed":.001}
        # Use the known workspace test directory; managed Windows sessions can
        # expose a system TEMP directory that Python cannot subsequently reopen.
        with tempfile.TemporaryDirectory(prefix="bounded_test_",dir=str(Path(__file__).resolve().parent)) as directory:
            result=demo.simulate(config,.2,.02,Path(directory),apply_limits=True)
            rows=[json.loads(line) for line in (Path(directory)/"telemetry.jsonl").read_text().splitlines()]
        self.assertTrue(result["apply_limits"])
        self.assertGreater(sum(result["limited_output_samples_per_agent"].values()),0)
        self.assertTrue(any(row["limited"] for row in rows))
        for row in rows:
            self.assertLessEqual(np.linalg.norm(row["input_applied"][:2]),.01+1e-12)
            self.assertEqual(row["limited"],np.linalg.norm(np.array(row["input_applied"])-row["input_raw"])>1e-12)


if __name__=="__main__":
    unittest.main()
