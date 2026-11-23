"""Geometry contracts, plus an optional forensic measurement on actual development holds."""
import argparse
import json
import math
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

TASK = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(TASK/"tests"),str(TASK/"solution")]
from lane import PIT_DEPTH
from reference import outboard_side
from service import dispatch
from world import World

SEEDS = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]


def contact_rows(w):
    d = w.data
    for i, (a,b) in enumerate(d.contact.geom):
        if a in w.feet and b in w.runners:
            foot, surface, sign = a,b,-1
        elif b in w.feet and a in w.runners:
            foot, surface, sign = b,a,1
        else:
            continue
        c = d.contact[i]
        force = np.zeros(6)
        mujoco.mj_contactForce(w.model,d,i,force)
        z = d.geom_xpos[surface,2] + w.model.geom_size[surface,2]
        yield {"foot":w.model.geom(foot).name,"surface":w.model.geom(surface).name,
               "normal_up":float(sign*c.frame[2]),"normal_n":float(force[0]),
               "midpoint_below_face_m":float(z-c.pos[2])}


def forensic(path):
    for i, seed in enumerate(SEEDS[:2]):
        w = World(seed,False,i)
        try:
            for event in map(json.loads,(path/str(i)/"ledger/events.jsonl").read_text().splitlines()):
                args = event["params"]
                dispatch(w,event["method"],args)
                if w.current==1 and args.get("ticks")==320:
                    print(json.dumps({"index":i,"pose":w.steps,"xyz":w.data.xpos[w.trunk].tolist(),
                                      "loads":list(contact_rows(w))}),flush=True)
                    break
        finally:
            w.close()


class SplitDeckTests(unittest.TestCase):
    def test_unloaded_alignment_pays_approach_but_cannot_park(self):
        w = World(SEEDS[0],False,0)
        self.addCleanup(w.close)
        w.advance()
        x,y,angle = w.stations[0]
        w.data.qpos[:3] = [x,y,.46]
        w.data.qpos[3:7] = [math.cos(angle/2),0.,0.,math.sin(angle/2)]
        w.data.qvel[:] = 0.
        mujoco.mj_forward(w.model,w.data)
        for _ in range(200):
            w._tick()
        self.assertEqual((w.approaches,w.delivered,w.dwell),(1,0,0))
        self.assertEqual(w.metrics()["milestone_fraction"],1/14)

    def test_bed_is_open_and_both_visible_tracks_are_real_surfaces(self):
        mask = np.array([1,0,0,0,0,0],dtype=np.uint8)
        for i, side in enumerate((1,-1)):
            w = World(SEEDS[i],True,i)
            try:
                w.advance()
                self.assertEqual(outboard_side(w.observe()["images"][0]),side)
                for y,name,depth in ((-.121*side,"near_runner",-.0008),(.240*side,"wide_runner",-.0008),
                                     (0.,"pit_bed",PIT_DEPTH),(.32,"slab_left",0.)):
                    hit = np.zeros(1,dtype=np.int32)
                    dist = mujoco.mj_ray(w.model,w.data,np.array([.40,y,.12]),np.array([0.,0.,-1.]),
                                         mask,1,-1,hit)
                    self.assertEqual(w.model.geom(hit[0]).name,name)
                    self.assertAlmostEqual(dist,.12+depth,places=5)
            finally:
                w.close()


if __name__=="__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--forensic",type=Path)
    args = p.parse_args()
    if args.forensic:
        forensic(args.forensic)
    else:
        unittest.main(argv=[sys.argv[0]],verbosity=2)
