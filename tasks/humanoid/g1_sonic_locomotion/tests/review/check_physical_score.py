"""Deterministic scoring fixtures; run inside the task's CPU image with /opt/venv/bin/python."""
import json
import math

import numpy as np
from sonic_loco.score import PhysicalScore


def sample(i, xy=(0.,0.), yaw=0., velocity=(0.,0.), omega=0., fallen=False, loads=(80.,80.)):
    return dict(time_s=i*.002,base_xy=xy,heading_rad=yaw,velocity_xy=velocity,yaw_rate_rad_s=omega,
                fallen=fallen,upright=not fallen,left_foot_load_n=loads[0],right_foot_load_n=loads[1])


def trial(route, distance, turn, fn, horizon=100):
    s=PhysicalScore(route,distance,turn,sample(0),np.zeros(29),horizon)
    for t in range(1,horizon+1):
        codes=[16]*64
        codes[0]=t%32
        s.update_tick(codes,np.full(29,2.*t*.02),[fn(i) for i in range(10*t-9,10*t+1)])
        if s.done:
            break
    return s.summary()


def check():
    stand=trial("straight_stop",5.,0.,sample,horizon=10)
    assert stand["reason"]=="time_limit" and stand["final_xy_error_m"]==5. and stand["route_efficiency"]==0
    stop=trial("straight_stop",5.,0.,lambda i: sample(i,xy=(5.,0.)))
    assert stop["reason"]=="success" and math.isclose(stop["simulated_s"],.75) and stop["applied_rows"]==38
    assert math.isclose(stop["issued_control_s"],.76) and stop["code_changes"]==37
    assert stop["scalar_codes"]==64*38 and stop["eligible_code_pairs"]==64*37
    assert stop["joint_accel_samples"]==29*(38-4) and math.isclose(stop["joint_accel_rms_rad_s2"],2.)
    assert stop["route_efficiency"]==1
    speed=trial("straight_stop",5.,0.,lambda i:sample(i,xy=(5.,0.),velocity=(.3 if i==300 else 0.,0.)))
    assert speed["reason"]=="success" and math.isclose(speed["simulated_s"],(300+375)*.002)
    yaw_pass=trial("right_turn_stop",0.,-math.pi/2,lambda i:sample(i,yaw=-math.pi/2))
    assert yaw_pass["reason"]=="success" and yaw_pass["route_efficiency"]==1
    wrong=trial("right_turn_stop",0.,-math.pi/2,lambda i:sample(i,yaw=3*math.pi/2))
    drift=trial("right_turn_stop",0.,-math.pi/2,lambda i:sample(i,xy=(.451,0.) if i==15 else (0.,0.),yaw=-math.pi/2))
    corridor=trial("straight_stop",5.,0.,lambda i:sample(i,xy=(2.5,.451) if i==15 else (5.,0.)))
    for r in (wrong,drift,corridor):
        assert r["reason"]=="path_violation"
    fall=trial("straight_stop",5.,0.,lambda i:sample(i,xy=(5.,0.),fallen=(i==375)))
    assert fall["reason"]=="fall" and math.isclose(fall["simulated_s"],.75)
    feet=trial("straight_stop",5.,0.,lambda i: sample(i,loads=(0. if 21<=i<=40 or 71<=i<=75 else 80.,80.)),horizon=10)
    assert (feet["left_unloads"],feet["left_reloads"],feet["right_unloads"],feet["right_reloads"])==(1,1,0,0)
    return dict(standing_reason=stand["reason"],correct_turn_reason=yaw_pass["reason"],
                complementary_left_turn_reason=wrong["reason"],drift_reason=drift["reason"],
                corridor_reason=corridor["reason"],fall_precedence=fall["reason"],
                terminal_event_s=stop["simulated_s"],charged_row_s=stop["issued_control_s"],
                native_row_count=stop["applied_rows"],scalar_code_count=stop["scalar_codes"],
                adjacent_coordinate_changes=stop["code_changes"],eligible_coordinate_comparisons=stop["eligible_code_pairs"],
                joint_acceleration_samples=stop["joint_accel_samples"],joint_acceleration_rms_rad_s2=stop["joint_accel_rms_rad_s2"],
                interrupted_braking_event_s=speed["simulated_s"],sustained_left_unloads=feet["left_unloads"],
                sustained_left_reloads=feet["left_reloads"])


if __name__=="__main__":
    print(json.dumps(check(),indent=2,allow_nan=False))
