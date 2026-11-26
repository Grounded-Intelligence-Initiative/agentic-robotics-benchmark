"""Author diagnostic only: compare RGB interpretation to simulation labels."""
from collections import Counter
from pathlib import Path
import sys

sys.path[:0] = [str(Path(__file__).resolve().parents[1]),
                str(Path(__file__).resolve().parents[2] / "solution")]
from cameras import Cameras
from controller import Team, read_view
from world import Home


def layout(w, p):
    if w.blocked(p):
        return "wall"
    if p in w.furniture:
        return "furniture"
    if p == w.box:
        return "box"
    if p in w.dust:
        return "dust"
    if p in w.rugs:
        return "runner"
    if p == w.dock:
        return "dock"
    return next((k for k,v in w.mats.items() if v==p),w.area[p])


if __name__ == "__main__":
    w = Home(int(sys.argv[1]))
    cam, team, errors = Cameras(w), Team(), Counter()
    try:
        for t in range(w.budget):
            obs = cam.observation()
            for sensor, pos, r in (("ground",w.ground,1), ("air",w.air,3)):
                if obs[sensor] and t%3==0:
                    for q,mark,base in read_view(obs[sensor],pos,r):
                        true=layout(w,q)
                        if mark!=true: errors[sensor,true,mark]+=1
            action = team.choose(t,obs); w.advance(action)
            if w.result()["success"]: break
        print("common vision errors:",errors.most_common(15))
        print("terminal",w.result(), "requested",w.request,"known",team.pads,
              "estimate box",team.box,"real relative box",(w.box[0]-w.dock[0],w.box[1]-w.dock[1]),
              "known dirt",team.dirt,"visited",team.cleaned,"last",action)
    finally:
        cam.close()
