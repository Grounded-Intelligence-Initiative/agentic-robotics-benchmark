"""Practice grasp alignment using the rim or the foam disc edge."""
import math
import numpy as np
import mujoco as mj

def make_grip(arena,obj,template,params):
    radial_offset,tangent_offset,height,pitch,roll=params
    side=obj['arm']; arm=arena.arms[side]
    site=arena.model.site(side+'_grasp_site').id
    base=np.asarray(arena.data.body(side+'_base').xpos)
    xyz=np.array(obj['position'])
    radial=xyz[:2]-base[:2]; radial/=np.linalg.norm(radial)
    tangent=np.array([-radial[1],radial[0]])
    xy=xyz[:2]+radial_offset*radial+tangent_offset*tangent
    d=mj.MjData(arena.model)
    d.qpos[:]=arena.data.qpos
    jac=np.zeros((3,arena.model.nv))
    limits=arm['range']
    dofs=arm['dof']
    qpos=arm['qpos'][:6]
    def ik(point,guess):
        q=np.array(guess,dtype=float)
        q[0]=math.atan2(point[1]-base[1],point[0]-base[0]);q[4]=0.;q[5]=roll
        for _ in range(50):
            q[3]=np.clip(q[1]-q[2]-pitch,*limits[3])
            d.qpos[qpos]=q
            mj.mj_forward(arena.model,d)
            error=point-d.site_xpos[site]
            if np.linalg.norm(error)<.0004: break
            mj.mj_jacSite(arena.model,d,jac,None,site)
            derivative=jac[:,dofs[:3]].copy()
            derivative[:,1]+=jac[:,dofs[3]]
            derivative[:,2]-=jac[:,dofs[3]]
            dq=derivative.T @ np.linalg.solve(derivative@derivative.T+1.e-4*np.eye(3),error)
            q[:3]=np.clip(q[:3]+np.clip(dq,-.12,.12),limits[:3,0],limits[:3,1])
        q[3]=np.clip(q[1]-q[2]-pitch,*limits[3])
        return q.tolist()
    rows=[r.copy() for r in template]
    if obj['mode']=='ring':
        outline={.68:(-.085,.12),1.05:(-.080,.022),1.56:(0,.003),1.72:(0,.003),1.98:(0,.005),2.60:(-.008,.15)}
    else:
        outline={1.45:(-.02,.10),1.86:(0,.005),2.02:(0,.002),2.24:(0,.004),2.68:(-.018,.14)}
    for row in rows:
        if row[0] not in outline:continue
        shift,z=outline[row[0]]
        row[1:7]=ik(np.r_[xy+shift*radial,xyz[2]+z+height],row[1:7])
    return rows
