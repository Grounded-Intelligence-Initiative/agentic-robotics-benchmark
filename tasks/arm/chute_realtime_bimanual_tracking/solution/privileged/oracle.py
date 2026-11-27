import numpy as np
import math
from motion import PoseIK
from narrow import Oracle as Narrow,rot
from dual import WIDE
from wide import Oracle as Dual
from wide import Oracle as Cracker
from packing import Packing
class Oracle:
    def __init__(self,**kw):
        self.narrow=Narrow(**kw);self.dual=Dual(**kw);self.cracker=Cracker(**kw)
        self.packing=Packing();self.mode=None;self.home_steps=0;self.return_event=None;self.returning=False
    def act(self,obs,tr):
        self.packing.observe(tr)
        p=np.asarray(tr['proprio'])
        if self.dual.home is None:
            self.dual.helper.calibrate(tr);self.cracker.helper.calibrate(tr)
            self.narrow._calibrate(tr,p);self.narrow.target_q=self.narrow._q(p);self.narrow.planner=PoseIK(self.narrow.arm,self.narrow.tool);self.narrow.planner_state="init"
            self.narrow.home_left=p[3:10].copy();self.narrow.base_home=np.asarray(tr['base_pos']);self.narrow.base_goal=self.narrow.base_home[:2].copy();self.narrow.base_yaw0=0.
        low=np.asarray(tr['low_end']);up=np.asarray(tr['up_chute']);left=np.asarray(tr['left']);normal=np.cross(up,left);normal/=np.linalg.norm(normal)
        lane_pos=low+up*self.narrow.s_catch+left*tr['lane']
        home_base=self.narrow.base_home[:2]+np.array([.15,np.clip(lane_pos[1]-self.narrow.base_home[1]+.385,-.25,.25)])
        if self.home_steps<24:
            if self.home_steps==0:self.home_planner=PoseIK(self.narrow.arm,self.narrow.tool)
            target=low+up*self.narrow.s_catch+left*tr['lane']+normal*.22
            R=np.stack([np.cross(left,normal),left,normal],axis=1)
            self.home_q=self.home_planner.solve(self.narrow._q(p),target-np.r_[home_base,tr['base_pos'][2]],R)
            self.home_steps+=1
        active=tr['phase'] in ('staged','live')
        item=tr['item'] if active else tr.get('upcoming',[{}])[0].get('item','') if tr.get('upcoming') else ''
        # 2026-11-26, variant C: the clamps and the drill are taken by one hand at a narrow feature (a handle) with
        # the narrow-part routine and carried by that hand alone. With the left hand joining afterwards (assist.py) the right
        # hand held 10 of 12 clamps but the two-handed transfer ran out of time (job 62725255); with both hands at once on the
        # moving part (wide.py) 2 of 16 wide parts were placed (job 62724807).
        single=item in ('051_large_clamp','052_extra_large_clamp','035_power_drill')
        mode='cracker' if item=='003_cracker_box' else 'narrow' if single else 'dual' if item in WIDE else 'narrow'
        if mode!=self.mode:
            self.mode=mode
            ctl=self.cracker if mode=='cracker' else self.dual if mode=='dual' else self.narrow
            ctl.event=-1;ctl.state='rest';ctl.target_q=ctl._q(p)
            if hasattr(ctl,'planner'):ctl.planner.plan=ctl.planner.pref.copy();ctl.planner_state='init'
        ctl=self.cracker if mode=='cracker' else self.dual if mode=='dual' else self.narrow
        if mode=='narrow' and tr['phase']=='gap' and self.return_event!=tr['event']:
            self.return_event=tr['event'];self.returning=True;self.return_stage='up';self.return_steps=0
        if mode=='narrow' and self.returning and self.home_steps>=24:
            self.return_steps+=1;tp,Rm=self.narrow._measured(tr);qn=self.narrow._q(p)
            if self.return_stage=='up':
                if tp[2]>low[2]+.21:self.return_stage='joint'
                else:
                    old=self.narrow.state;self.narrow.state='return_up'
                    q,_=self.narrow._servo(tr,qn,np.array([tp[0],tp[1],low[2]+.26]),Rm)
                    self.narrow.state=old
            if self.return_stage=='joint':q=qn+np.clip(self.home_q-qn,-.10,.10)
            row=np.zeros(23);row[3:7]=q[:4];row[15:22]=q[4:];row[7:14]=p[3:10]+np.clip(self.narrow.home_left-p[3:10],-.10,.10);row[14]=row[22]=1.
            pb=np.asarray(tr['base_pos']);Rb=rot(tr['base_quat']);e=Rb[:2,:2].T@(home_base-pb[:2]);yaw=math.atan2(Rb[1,0],Rb[0,0])
            row[:3]=[np.clip(4*e[0],-.85,.85),np.clip(4*e[1],-.85,.85),np.clip(-3*yaw,-1.,1.)]
            self.state='return_'+self.return_stage;self.t_state=self.return_steps;self.event=tr['event'];self.diag={};self.log=self.narrow.log
            if self.return_stage=='joint' and np.max(np.abs(qn-self.home_q))<.14 and np.linalg.norm(home_base-pb[:2])<.04 and abs(yaw)<.06:
                if active:
                    self.returning=False;self.narrow.state='wait';self.narrow.event=-1;self.narrow.planner.plan=self.home_q.copy();self.narrow.planner_state='wait'
            return row
        if mode!='narrow':self.returning=False
        if mode=='narrow' and hasattr(ctl,'flat_R') and active:
            tr=dict(tr);tr['packing_rotation']=ctl.flat_R.tolist()
        derived=self.packing.choose(tr) if active and ctl.state in ('carry','lower','drop','transit','release') else tr
        if mode=='narrow' and active and tr['item'] in ('051_large_clamp','052_extra_large_clamp','035_power_drill'):
            pairs,w=self.dual.pair(tr);(lp,la),(rp,ra)=pairs
            derived=dict(derived);derived.update(grasp_local=rp.tolist(),grasp_axis=ra.tolist(),grasp_width=w,no_join=True)
        row=ctl.act(obs,derived)
        for k in ('state','t_state','event','diag','log'):setattr(self,k,getattr(ctl,k))
        return row
