"""Physical access, load-bearing parking and observable-equipment contracts."""
import json
import math
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

TASK=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(TASK/"tests"))
from lane import aisle_dimensions,plan
from world import World
SEEDS=json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]
HOME=[0.,.9,-1.8]*4


class ServiceAisleTests(unittest.TestCase):
    def test_all_original_layouts_have_two_distinct_physical_transfers(self):
        dims=[]
        for index,seed in enumerate(SEEDS):
            stations,*_=plan(seed,index)
            s=aisle_dimensions(seed,stations)
            a,b,deck=stations[:3]
            self.assertLess((a[1]-s["gate_y"])*(b[1]-s["gate_y"]),0)
            self.assertLess(abs(s["gate_x"]-a[0]),.03)
            self.assertEqual(deck,s["deck"])
            self.assertTrue(.75<=s["width"]<=.81 and .38<=s["clearance"]<=.39)
            self.assertTrue(.046<=s["rise"]<=.056)
            dims.append((s["clearance"],s["rise"]))
        self.assertEqual(len(set(dims)),24)

    def test_actual_camera_hood_contacts_overhead_at_standing_but_not_low_height(self):
        w=World(SEEDS[0],False,0)
        self.addCleanup(w.close)
        s=w.aisle
        yaw=s["hand"]*math.pi/2
        w.data.qpos[3:7]=[math.cos(yaw/2),0,0,math.sin(yaw/2)]
        w.data.qpos[:2]=[s["gate_x"],s["gate_y"]-s["hand"]*.096]
        def touches(z):
            w.data.qpos[2]=z
            mujoco.mj_forward(w.model,w.data)
            return any(set(pair)=={w.hood,w.duct} for pair in w.data.contact.geom)
        self.assertTrue(touches(.265))
        self.assertFalse(touches(.18))
        for name in ("bay_left","bay_back","bay_rail_2","duct_jamb_right","cable_deck","camera_hood"):
            self.assertTrue(w.model.geom(name).contype or w.model.geom(name).conaffinity)

    def test_deck_inspection_needs_actual_loaded_top_contacts_not_location_alone(self):
        w=World(SEEDS[0],False,0)
        self.addCleanup(w.close)
        x,y,yaw=w.stations[2]
        w.current,w.approaches,w.delivered=3,2,2
        w._place_station(2)
        w.data.qpos[:3]=[x,y,w.aisle["rise"]+.45]
        w.data.qpos[3:7]=[math.cos(yaw/2),0,0,math.sin(yaw/2)]
        w.data.qvel[:]=0
        mujoco.mj_forward(w.model,w.data)
        for _ in range(200):
            w._tick()
        self.assertEqual((w.approaches,w.delivered,w.dwell),(3,2,0))
        w.data.qpos[2]=w.aisle["rise"]+.27
        mujoco.mj_forward(w.model,w.data)
        self.assertTrue(w.act(HOME,500)["goal_reached"])
        self.assertGreaterEqual(w.passage["deck_max_feet"],2)
        self.assertGreater(w.passage["deck_dock_trunk_z_m"],w.aisle["rise"]+.15)


if __name__=="__main__":
    unittest.main(verbosity=2)
