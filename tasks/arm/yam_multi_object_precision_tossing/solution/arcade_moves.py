"""Seed-17 sorting, hoop and foam-disc motion primitives."""
import math
import numpy as np

def configuration(arena, item, source, clip, params, target):
    rows = [r.copy() for r in source]
    base = arena.data.body(item.arm+'_base').xpos
    bearing = math.atan2(target[1]-base[1],target[0]-base[0])
    angle, timing, wind, bend, roll, speed = params
    if item.mode == 'ring':
        part = [r.copy() for r in clip['right_slot_small_play_piece'][7:]]
        for r in part:
            r[6] = np.pi/2
            if r[-1] == 0: r[-1]=.004
        rows += part
    if item.mode in ('slot','ring'):
        rows = [r for r in rows if r[0] < 3.6] + rows[-4:]
        hold = rows[-3][0]
        original_open = rows[-2][0]
        rows.pop(-2)
        first,last=rows[-2:]
        first[1]=rows[-3][1]=last[1]=bearing+angle
        rows[-3][2] += wind
        first[2] += wind
        rows[-3][3] += bend
        first[3] += bend
        rows[-3][4] += roll
        first[4] += roll
        last[0]=hold+(last[0]-hold)*speed
        open_t=hold+(original_open-hold)*speed+timing
        open_t=np.clip(open_t,hold+.006,last[0]-.005)
        u=(open_t-hold)/(last[0]-hold)
        row=[float(open_t),*(np.array(first[1:7])*(1-u)+np.array(last[1:7])*u),last[-1]]
        rows.insert(-1,row)
    else:
        start,middle,last=rows[-3:]
        for row in (start,middle,last):
            row[1]+=angle
            row[2]+=wind
            row[3]+=bend
            row[4]+=roll
        start_time=start[0]
        open_t=start_time+(middle[0]-start_time)*speed+timing
        last[0]=start_time+(last[0]-start_time)*speed
        u=(open_t-start_time)/(last[0]-start_time)
        middle[:]=[float(open_t),*(np.array(start[1:7])*(1-u)+np.array(last[1:7])*u),middle[-1]]
    limits=arena.arms[item.arm]['range']
    for row in rows:
        row[1:7]=np.clip(row[1:7],limits[:,0],limits[:,1]).tolist()
    return {'item':item.id,'arm':item.arm,'waypoints':rows}
