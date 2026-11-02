import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location("trials",ROOT/"scripts/make_delay_trials.py")
trials=importlib.util.module_from_spec(spec);spec.loader.exec_module(trials)


class TrialTests(unittest.TestCase):
    def test_factorial_design_preserves_noncommunication_guards_and_task(self):
        base=yaml.safe_load((ROOT/"config/experiment.yaml").read_text())
        with tempfile.TemporaryDirectory(dir=ROOT/"tests") as tmp:
            cases=trials.generate(tmp)
            self.assertEqual(len(cases),14)
            self.assertEqual(sum(x["kind"]=="baseline" for x in cases),2)
            for initial in ("A","B"):
                actual={(x["seed"],x["signal_timeout_s"]) for x in cases if x["initial_set"]==initial and x["kind"]=="delay"}
                self.assertEqual(actual,{(seed,t) for seed in [11,29,47] for t in [.3,.6]})
            for case in cases:
                config=yaml.safe_load((Path(tmp)/case["config"]).read_text())
                self.assertEqual(config["gains"],base["gains"])
                self.assertEqual(config["limits"],base["limits"])
                self.assertEqual(config["monitoring"],base["monitoring"])
                self.assertEqual(config["hardware"],base["hardware"])
                for key,value in base["runtime"].items():
                    if key!="signal_timeout":self.assertEqual(config["runtime"][key],value)
                self.assertEqual(config["communication"]["std_s"],.05)
                self.assertEqual(config["communication"]["max_delay_s"],.5)

if __name__=="__main__":unittest.main()
