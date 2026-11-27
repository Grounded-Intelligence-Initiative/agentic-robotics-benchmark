"""Coordinated grasp and transport of wide parts with both hands."""
import math
import numpy as np
import kin
from scipy.optimize import least_squares
from narrow import rot, PRO

WIDE = {'003_cracker_box', '035_power_drill', '051_large_clamp', '052_extra_large_clamp'}

class Dual:
    def __init__(self):
        self.arms = {s: kin.Arm(s) for s in ('left','right')}
        self.off = {}; self.reach = {}; self.tool = {}
        self.state='wait'; self.t_state=0; self.event=-1; self.log=[]; self.diag={}
        self.grips=[1.,1.]; self.home=None
        self.qpref=np.r_[.3,-.6,-.3,0., -1.5,.2,-.5,-.8,0.,.5,-.2, -1.5,-.2,.5,-.8,0.,.5,.2]

    def calibrate(self,tr):
        self.home=np.array(tr['base_pos']); self.yaw0=math.atan2(rot(tr['base_quat'])[1,0],rot(tr['base_quat'])[0,0])
        Rb=rot(tr['base_quat']); p=np.array(tr['proprio'])
        for s in self.arms:
            q=np.r_[p[53:57],p[PRO['arm_'+s]]]
            pm,Rm,_=self.arms[s].fk(q)
            self.off[s]=(Rb@Rm).T@rot(tr['eef_'+s+'_quat'])
            R=rot(tr['eef_'+s+'_quat'])@self.off[s].T
            self.reach[s]=float(np.clip(tr['fingers_'+s][2]-tr['finger_low_'+s],.02,.12)) if R[2,2]>.97 else .042
            self.tool[s]=np.array([0.,0.,-.03689-self.reach[s]])

    def measured(self,tr,s):
        R=rot(tr['eef_'+s+'_quat'])@self.off[s].T
        return np.array(tr['fingers_'+s])-R[:,2]*self.reach[s],R

    def go(self,s):
        if self.state!=s: self.state=s;self.t_state=0

    def servo(self,tr,targets,rots,gain=.65):
        p=np.asarray(tr['proprio']); q=np.r_[p[53:57],p[3:10],p[28:35]]
        Rb=rot(tr['base_quat']); A=[];es=[];self.errors=[]
        for i,s in enumerate(('left','right')):
            idx=np.r_[np.arange(4),np.arange(4+7*i,11+7*i)]
            qm=q[idx]; J,_,_=self.arms[s].jacobian(qm,self.tool[s]); tp,Rm=self.measured(tr,s)
            Rt=rots[i]; flip=Rt@np.diag([-1.,-1.,1.])
            if np.trace(flip@Rm.T)>np.trace(Rt@Rm.T):Rt=flip
            ep=Rb.T@(targets[i]-tp); Re=(Rb.T@Rt)@(Rb.T@Rm).T
            er=.5*np.array([Re[2,1]-Re[1,2],Re[0,2]-Re[2,0],Re[1,0]-Re[0,1]])
            a=np.zeros((6,18)); a[:,idx]=np.r_[J[:3],.25*J[3:]]
            A.append(a);es.extend(np.r_[np.clip(ep,-.15,.15),.25*er]);self.errors.append(float(np.linalg.norm(ep)))
        A=np.concatenate(A);e=np.array(es);free=np.ones(18,bool);free[:4]=self.state in ('carry','lower','settle','drop'); A=A[:,free]
        pinv=A.T@np.linalg.inv(A@A.T+.04**2*np.eye(12));dq=pinv@e
        dq+=(np.eye(int(free.sum()))-np.linalg.pinv(A,rcond=1e-3)@A)@(.05*(self.qpref[free]-q[free]))
        lo=np.r_[self.arms['left'].lo,self.arms['right'].lo[4:]];hi=np.r_[self.arms['left'].hi,self.arms['right'].hi[4:]]
        if hasattr(self,'last_command'):
            limits=np.r_[np.full(4,.18),np.full(14,.07)]
            self.servo_bias=np.clip(.9*getattr(self,'servo_bias',np.zeros(18))+.1*(self.last_command-q),-limits,limits)
        else:self.servo_bias=np.zeros(18)
        q[free]=np.clip(q[free]+np.clip(gain*dq,-.07,.07)+self.servo_bias[free],lo[free],hi[free])
        if not free[0]:q[:4]=self.qpref[:4]+self.servo_bias[:4]
        return q

    def solve_ready(self,tr,targets,rots,base_goal):
        # A bounded number of analytic IK iterations per control period. The staged
        # interval supplies enough periods without making any reply late.
        yaw=tr['yaw']-np.pi/2;Rb=np.array([[np.cos(yaw),-np.sin(yaw),0.],[np.sin(yaw),np.cos(yaw),0.],[0.,0.,1.]]); basep=np.r_[base_goal,tr['base_pos'][2]]
        self.plan_low_ps=[Rb.T@(p-basep) for p in targets]
        self.plan_ps=[p+np.array([0.,0.,.20]) for p in self.plan_low_ps]
        self.plan_stage='high';self.high_q=None;self.approach_high=True
        self.plan_Rs=[Rb.T@r for r in rots]
        self.plan_q=self.qpref.copy(); self.plan_iter=0
        self.plan_best=(float('inf'),self.plan_q.copy())
        self.ready_q=self.plan_q.copy();self.ready_steps=0

    def plan_step(self):
        lo=np.r_[self.arms['left'].lo,self.arms['right'].lo[4:]]+.01
        hi=np.r_[self.arms['left'].hi,self.arms['right'].hi[4:]]-.01
        lo[:4]=np.array([.3,-.6,-.3,0.])-np.array([.03,.03,.03,.12]);hi[:4]=np.array([.3,-.6,-.3,0.])+np.array([.03,.03,.03,.12])
        # Full rotation residual, analytic derivatives, and bounded work. Separate
        # symmetric wrist branches are explored across successive staged replies.
        branch=min(3,self.plan_iter//8); flip=3-branch
        if self.plan_iter%8==0:self.plan_q=np.clip(self.qpref,lo,hi)
        Rs=[r@(np.diag([-1.,-1.,1.]) if (flip>>i)&1 else np.eye(3)) for i,r in enumerate(self.plan_Rs)]
        idxs=[np.r_[np.arange(4),np.arange(4+7*i,11+7*i)] for i in range(2)]
        cached=[None,None,None]
        def evaluate(q):
            if cached[0] is not None and np.array_equal(q,cached[0]):return cached[1],cached[2]
            err=[];rows=[]
            for i,side in enumerate(('left','right')):
                ix=idxs[i];J,p,R=self.arms[side].jacobian(q[ix],self.tool[side])
                err.extend(p-self.plan_ps[i]);err.extend((.25*(R-Rs[i])).ravel())
                a=np.zeros((12,18));a[:3,ix]=J[:3]
                for k in range(11):a[3:,ix[k]]=(.25*np.cross(J[3:,k],R.T).T).ravel()
                rows.append(a)
            err.extend(.004*(q-self.qpref));err.append(.1*q[3])
            A=np.concatenate(rows+[.004*np.eye(18),np.eye(18)[3:4]*.1])
            cached[:]=[q.copy(),np.asarray(err),A];return cached[1],A
        res=least_squares(lambda q:evaluate(q)[0],self.plan_q,jac=lambda q:evaluate(q)[1],bounds=(lo,hi),max_nfev=5,ftol=1e-6)
        self.plan_q=res.x;cost=np.linalg.norm(res.fun[:24])+.0003*np.linalg.norm(res.x-self.qpref)
        if cost<self.plan_best[0]:self.plan_best=(cost,res.x.copy())
        self.ready_q=self.plan_best[1].copy();self.plan_iter+=1
        if self.plan_iter>=32 or (self.plan_iter>=8 and self.plan_best[0]<.008):
            if self.plan_stage=='high':
                self.high_q=self.ready_q.copy();self.qpref=self.high_q.copy()
                self.plan_ps=self.plan_low_ps;self.plan_stage='low';self.plan_iter=0
                self.plan_q=self.high_q.copy();self.plan_best=(float('inf'),self.high_q.copy())
            else:self.plan_stage='done'

    def grasps(self,tr,n):
        """Local edge/handle pinches, assigned to the nearest side of the robot."""
        name=tr['item']; sz=np.array(tr['size']); Rp=rot(tr['quat']);p=np.array(tr['pos'])
        if name=='003_cracker_box':
            d=.020
            points=[np.array([sz[0]/2-d,sz[1]/2-d,0.]),np.array([-sz[0]/2+d,-sz[1]/2+d,0.])]
            axes=[np.array([1.,-1.,0.])/2**.5]*2
        elif name=='035_power_drill':
            points=[np.array([.035,.035,0.]),np.array([-.055,-.065,0.])]
            axes=[np.array([1.,0.,0.]),np.array([0.,1.,0.])]
        else:
            points=[np.array([-.18*sz[0],0.,0.]),np.array([.40*sz[0],.40*sz[1],0.])]
            axes=[np.array([0.,1.,0.]),np.array([-.6,.8,0.])]
        out=[]
        for point,axis in zip(points,axes):
            gp=p+Rp@point; ga=Rp@axis;ga-=n*(ga@n);ga/=np.linalg.norm(ga)
            out.append((gp,np.stack([np.cross(ga,n),ga,n],axis=1)))
        center=.5*(out[0][0]+out[1][0])
        tilted=[]
        for gp,R in out:
            tangent=R[:,0];sign=1. if tangent@(gp-center)>0 else -1.
            nz=n.copy()
            ga=R[:,1];tilted.append((gp,np.stack([np.cross(ga,nz),ga,nz],axis=1)))
        out=tilted
        out.sort(key=lambda x: -float(x[0]@np.array(tr['fwd'])))
        return out

    def preposition(self,tr):
        p=np.asarray(tr['proprio']);qn=np.r_[p[53:57],p[3:10],p[28:35]]
        safe=np.r_[.3,-.6,-.3,0., 0.,1.3,0.,0.,0.,0.,0., 0.,-1.3,0.,0.,0.,0.,0.]
        q=qn+np.clip(safe-qn,-.07,.07)
        low=np.asarray(tr['low_end']);left=np.asarray(tr['left']);goal=(low+np.asarray(tr['fwd'])*.65+left*.58)[:2]
        Rb=rot(tr['base_quat']);pb=np.asarray(tr['base_pos']);e=Rb[:2,:2].T@(goal-pb[:2])
        yaw=(math.atan2(Rb[1,0],Rb[0,0])-(tr['yaw']-np.pi/2)+np.pi)%(2*np.pi)-np.pi
        self.last_command=q.copy()
        row=np.zeros(23);row[3:7]=q[:4];row[7:14]=q[4:11];row[15:22]=q[11:];row[14]=row[22]=1.
        row[:3]=[np.clip(4*e[0],-.8,.8),np.clip(4*e[1],-.8,.8),np.clip(-3*yaw,-1.2,1.2)]
        return row

    def act(self,obs,tr):
        if self.home is None:self.calibrate(tr)
        self.t_state+=1
        low=np.array(tr['low_end']);up=np.array(tr['up_chute']);left=np.array(tr['left']);n=np.cross(up,left);n/=np.linalg.norm(n)
        valid=tr['phase'] in ('staged','live')
        if valid and tr['event']!=self.event:
            self.event=tr['event'];self.go('wait');self.grips=[1.,1.];self.closed=0;self.stable=0;self.prev_pos=None;self.i=0.;self.settled=0;self.ready_q=None;self.motion_q=None;self.high_steps=0;self.preparing=True;self.path_checked=False;self.joint_path_safe=False
        if not valid and self.state not in ('wait','rest'):
            self.go('rest');self.grips=[1.,1.]
        Rb=rot(tr['base_quat']);pb=np.array(tr['base_pos']);tote=np.array(tr['tote'])
        measured=[self.measured(tr,s)[0] for s in ('left','right')]
        # Place the feed lane between the shoulders for two-handed operation.
        if valid and self.state in ('wait','follow','close'):
            lane=float((np.array(tr['pos'])-low)@left)
            self.base_goal=(low+np.asarray(tr['fwd'])*.65+left*.58)[:2]
        elif self.state in ('carry','lower','settle','drop'):
            self.base_goal=tote[:2]-np.array([.57,0.])
        elif not hasattr(self,'base_goal'):self.base_goal=self.home[:2].copy()
        targets=[];rots=[]
        if self.state in ('wait','follow','close') and valid:
            pos=np.array(tr['pos']);vel=np.array(tr['vel']);s=float((pos-low)@up);v=max(0.,-vel@up)
            grasps=self.grasps(tr,n)
            a=max(0.,tr.get('accel_slide',0.));distance=max(0.,s-.65)
            eta=2*distance/(v+np.sqrt(v*v+2*a*distance)+1e-9)
            if tr['phase']=='staged':eta=10.
            if self.state=='wait' and tr['phase']=='live' and eta<.35 and not self.preparing and not self.approach_high and self.plan_stage=='done':self.go('follow')
            center=(measured[0]+measured[1])/2
            if self.state in ('follow','close') and s<=.65:self.i=float(np.clip(self.i+.08*((pos-center)@up),-.07,.07))
            sref=float(np.clip(s-v*.10+self.i,.10,.65)) if self.state != "close" else s + float(vel@up)*.08
            height=2*float(np.abs(rot(tr['quat']).T@n)@(np.array(tr['size'])/2))
            tip_h=max(.008,height-.055) if self.state != "wait" else height+.04
            for gp,R in grasps:
                t=gp+up*(sref-s)
                offset=.025 if tr["item"]=="003_cracker_box" else .012
                if self.state == "wait": t+=n*(height+.04-(t-low)@n)
                else:
                    t-=n*offset
                    t+=n*(height/2+.04+offset)*float(np.clip((eta-.10)/.22,0.,1.))
                targets.append(t);rots.append(R)
            if self.ready_q is None:self.solve_ready(tr,targets,rots,self.base_goal)
            if self.state=='wait' and self.plan_stage!='done':self.plan_step()
            errs=[np.linalg.norm((gp-cur)-n*((gp-cur)@n)) for (gp,_),cur in zip(grasps,measured)]
            depth=max(float((cur-gp)@n)+(.025 if tr['item']=='003_cracker_box' else .012) for cur,(gp,_) in zip(measured,grasps))
            depths=[float((cur-gp)@n)+(.025 if tr['item']=='003_cracker_box' else .012) for cur,(gp,_) in zip(measured,grasps)]
            for i in range(2):
                tolerance=.009 if 'clamp' in tr['item'] else .024
                if self.grips[i]>0 and self.state in ('follow','close') and errs[i]<.035 and depths[i]<tolerance:
                    self.grips[i]=-1.
                    if self.state=='follow':
                        self.closed=tr['step'];self.go('close')
                        self.log.append({'event':self.event,'item':tr['item'],'close_s':s,'errors':errs})
            if any(g<0 for g in self.grips):
                if not hasattr(self,'closing_rots') or self.closing_event!=self.event:
                    self.closing_rots=[r.copy() for r in rots];self.closing_event=self.event
                rots=[self.closing_rots[i] if self.grips[i]<0 else r for i,r in enumerate(rots)]
            if self.state=='close':
                gaps=[tr['finger_gap_'+side] for side in ('left','right')]
                both_blocked=min(gaps)>.035
                self.stable=self.stable+1 if both_blocked else 0
                if tr.get('attached_right') or tr.get('attached_left') or (self.stable>=15 and tr['step']-self.closed>=16):
                    self.grips=[-1.,-1.]
                    self.lift_targets=[cur+np.array([0.,0.,.22]) for cur in measured];self.hold_rots=[self.measured(tr,side)[1] for side in ('left','right')]
                    self.go('lift')
                elif tr['step']-self.closed>30:
                    self.lift_targets=[cur+np.array([0.,0.,.22]) for cur in measured];self.hold_rots=[self.measured(tr,side)[1] for side in ('left','right')];self.go('lift')
            self.diag[self.event]={'item':tr['item'],'s':round(s,3),'errors':np.round(errs,3).tolist(),'depth':round(depth,3),'gaps':[round(tr['finger_gap_'+side],3) for side in ('left','right')],'mode':'dual'}
        elif self.state=='lift':
            targets=self.lift_targets;rots=self.hold_rots
            if self.t_state>24:
                self.go('drive_back');self.back_y=pb[1]
                p=np.asarray(tr['proprio']);self.drive_q=np.r_[p[53:57],p[3:10],p[28:35]]
        elif self.state in ('drive_back','drive_side'):
            self.base_goal=np.array([tote[0]-.65,self.back_y if self.state=='drive_back' else tote[1]])
            targets=measured;rots=[self.measured(tr,side)[1] for side in ('left','right')]
            if np.linalg.norm(pb[:2]-self.base_goal)<.035:
                if self.state=='drive_back':self.go('drive_side')
                else:
                    self.hold_rots=[self.measured(tr,side)[1] for side in ('left','right')];self.go('carry')
        elif self.state in ('carry','lower'):
            pos=np.array(tr['pos']) if valid else (measured[0]+measured[1])/2
            goal=tote+np.array([0.,0.,max(.38,low[2]+.33-tote[2]) if self.state=='carry' else .24])
            delta=goal-pos;delta=np.clip(delta,-.12,.12)
            targets=[cur+delta for cur in measured];rots=self.hold_rots
            near=np.linalg.norm((pos-tote)[:2])<.04 and abs(pos[2]-goal[2])<.04
            self.settled=self.settled+1 if near and np.linalg.norm(tr.get('vel',[0,0,0]))<.20 else 0
            if self.state=='carry' and self.settled>=3:self.go('lower');self.settled=0
            elif (self.state=='lower' and (self.settled>=3 or (self.t_state>85 and np.linalg.norm((pos-tote)[:2])<.05))) or self.t_state>280:
                self.go('settle');self.drop_targets=[cur.copy() for cur in measured]
                p=np.asarray(tr['proprio']);self.release_q=np.r_[p[53:57],p[3:10],p[28:35]]
        elif self.state=='settle':
            targets=self.drop_targets;rots=self.hold_rots
            if self.t_state>12:self.go('drop');self.grips=[1.,1.]
        elif self.state=='drop':
            targets=[p+np.array([0.,0.,.12 if self.t_state>6 else 0.]) for p in self.drop_targets];rots=self.hold_rots
            if self.t_state>20:self.go('rest')
        else:
            self.grips=[1.,1.]
            for sign in (1.,-1.):
                targets.append(low+up*.65+left*(sign*.14)+n*.25)
                rots.append(np.stack([np.cross(left,n),left,n],axis=1))
        if not targets:
            targets=measured;rots=[self.measured(tr,s)[1] for s in ('left','right')]
        if self.state=='settle' or (self.state=='drop' and self.t_state<10):
            q=self.release_q
        elif self.state in ('drive_back','drive_side'):
            q=self.drive_q
        elif self.state=='wait' and self.preparing:
            p=np.asarray(tr['proprio']);qn=np.r_[p[53:57],p[3:10],p[28:35]]
            safe=np.r_[.3,-.6,-.3,0., 0.,1.3,0.,0.,0.,0.,0., 0.,-1.3,0.,0.,0.,0.,0.]
            q=qn+np.clip(safe-qn,-.07,.07)
            yaw=math.atan2(Rb[1,0],Rb[0,0]);yawerr=(yaw-(tr['yaw']-np.pi/2)+np.pi)%(2*np.pi)-np.pi
            if np.max(np.abs(qn-safe))<.16 and np.linalg.norm(pb[:2]-self.base_goal)<.04 and abs(yawerr)<.12:self.preparing=False
        elif self.state=='wait' and self.ready_q is not None:
            p=np.asarray(tr['proprio']);qn=np.r_[p[53:57],p[3:10],p[28:35]]
            if self.approach_high:
                goal_q=self.high_q if self.high_q is not None else self.ready_q
                self.high_steps+=1
                if self.high_q is not None and (np.max(np.abs(qn-self.high_q))<.18 or (self.high_steps>36 and min(cur[2] for cur in measured)>low[2]+.30)):self.approach_high=False
            else:goal_q=self.ready_q
            if self.motion_q is None:self.motion_q=qn.copy()
            self.motion_q+=np.clip(goal_q-self.motion_q,-.07,.07)
            self.motion_q=np.clip(self.motion_q,qn-.3,qn+.3);q=self.motion_q.copy();self.ready_steps+=1
        else:
            if self.state=='wait' and self.ready_q is not None:self.qpref=self.ready_q.copy()
            q=self.servo(tr,targets,rots,gain=.35 if self.state in ('lift','carry','lower') else .5)
        self.last_command=q.copy()
        row=np.zeros(23);row[3:7]=q[:4];row[7:14]=q[4:11];row[15:22]=q[11:];row[14],row[22]=self.grips
        if self.state not in ('follow','close','lift','settle','drop'):
            e=Rb[:2,:2].T@(self.base_goal-pb[:2]);goal_yaw=tr['yaw']-np.pi/2 if self.state in ('wait','follow','close','lift') else self.yaw0; yaw=(math.atan2(Rb[1,0],Rb[0,0])-goal_yaw+np.pi)%(2*np.pi)-np.pi
            row[:3]=[np.clip(4*e[0],-.8,.8),np.clip(4*e[1],-.8,.8),np.clip(-3*yaw,-1.2,1.2)]
        return row
