"""Right-hand lead with a separately controlled supporting left hand on wide parts."""
import math
import numpy as np
from scipy.optimize import least_squares
from narrow import Oracle as Narrow,rot
from dual import Dual,WIDE
class Oracle(Narrow):
    def __init__(self,**kw):
        super().__init__(**kw);self.helper=Dual();self.left_event=-1;self.left_grip=1.;self.left_lifted=False
    @property
    def home(self):return self.helper.home
    @property
    def yaw0(self):return self.helper.yaw0
    def calibrate(self,tr):self.helper.calibrate(tr)
    def pair(self,tr):
        name=tr['item'];sz=np.asarray(tr['size']);Rp=rot(tr['quat'])
        if name=='003_cracker_box':
            pts=[np.array([sz[0]/2-.020,sz[1]/2-.020,0.]),np.array([-sz[0]/2+.020,-sz[1]/2+.020,0.])];axes=[np.array([1.,-1.,0.])/2**.5]*2;w=.060
        elif name=='035_power_drill':
            pts=[np.array([.035,.035,0.]),np.array([-.050,-.065,0.])];axes=[np.array([1.,0.,0.]),np.array([0.,1.,0.])];w=.055
        else:
            pts=[np.array([.40*sz[0],.40*sz[1],0.]),np.array([.40*sz[0],-.40*sz[1],0.])];axes=[np.array([-.6,.8,0.]),np.array([.6,.8,0.])];w=.030
        pairs=list(zip(pts,axes));pairs.sort(key=lambda pa:-(Rp@pa[0])[1])
        if name=='035_power_drill':pairs=[(pts[1],axes[1]),(pts[0],axes[0])]
        return pairs,w
    def left_servo(self,tr,target,Rt):
        p=np.asarray(tr['proprio']);trunk=p[53:57];qn=p[3:10];arm=self.helper.arms['left'];Rb=rot(tr['base_quat']);pb=np.asarray(tr['base_pos'])
        pref=np.array([-2.,.1,-2.,-1.1,.4,.1,-1.2]);lo=arm.lo[4:]+.01;hi=arm.hi[4:]-.01
        seed=np.clip(getattr(self,'left_plan',pref),lo,hi);target_p=Rb.T@(target-pb);target_R=Rb.T@Rt
        _,Rseed,_=arm.fk(np.r_[trunk,seed],self.helper.tool['left'])
        flip=target_R@np.diag([-1.,-1.,1.])
        if np.trace(flip@Rseed.T)>np.trace(target_R@Rseed.T):target_R=flip
        cache=[None,None,None]
        def ev(q):
            if cache[0] is not None and np.array_equal(q,cache[0]):return cache[1],cache[2]
            J,pk,R=arm.jacobian(np.r_[trunk,q],self.helper.tool['left']);J=J[:,4:]
            e=np.r_[pk-target_p,(.25*(R-target_R)).ravel(),.003*(q-pref)]
            A=np.zeros((19,7));A[:3]=J[:3]
            for k in range(7):A[3:12,k]=(.25*np.cross(J[3:,k],R.T).T).ravel()
            A[12:]=.003*np.eye(7);cache[:]=[q.copy(),e,A];return e,A
        res=least_squares(lambda q:ev(q)[0],seed,jac=lambda q:ev(q)[1],bounds=(lo,hi),max_nfev=4)
        self.left_plan=res.x.copy()
        if not hasattr(self,'left_command'):self.left_command=qn.copy()
        self.left_command+=np.clip(self.left_plan-self.left_command,-.08,.08)
        self.left_command=np.clip(self.left_command,qn-.25,qn+.25)
        return self.left_command.copy()
    def both_servo(self,tr,targets,rots):
        p=np.asarray(tr['proprio']);qn=np.r_[p[53:57],p[3:10],p[28:35]];Rb=rot(tr['base_quat']);pb=np.asarray(tr['base_pos'])
        lo=np.r_[self.helper.arms['left'].lo,self.helper.arms['right'].lo[4:]]+.008;hi=np.r_[self.helper.arms['left'].hi,self.helper.arms['right'].hi[4:]]-.008
        ps=[Rb.T@(pp-pb) for pp in targets];Rs=[];idxs=[np.r_[np.arange(4),np.arange(4+7*i,11+7*i)] for i in range(2)]
        for i,side in enumerate(('left','right')):
            rt=rots[i];refq=getattr(self,'both_plan',qn)[idxs[i]];rm=Rb@self.helper.arms[side].fk(refq,self.helper.tool[side])[1];flip=rt@np.diag([-1.,-1.,1.])
            if np.trace(flip@rm.T)>np.trace(rt@rm.T):rt=flip
            Rs.append(Rb.T@rt)
        cache=[None,None,None]
        def ev(q):
            if cache[0] is not None and np.array_equal(q,cache[0]):return cache[1],cache[2]
            err=[];rows=[]
            for i,side in enumerate(('left','right')):
                ix=idxs[i];J,pk,R=self.helper.arms[side].jacobian(q[ix],self.helper.tool[side]);err.extend(pk-ps[i]);err.extend((.20*(R-Rs[i])).ravel())
                A=np.zeros((12,18));A[:3,ix]=J[:3]
                for k in range(11):A[3:,ix[k]]=(.20*np.cross(J[3:,k],R.T).T).ravel()
                rows.append(A)
            err.extend(.006*(q-qn));rows.append(.006*np.eye(18));cache[:]=[q.copy(),np.asarray(err),np.concatenate(rows)];return cache[1],cache[2]
        seed=np.clip(getattr(self,'both_plan',qn),lo,hi)
        res=least_squares(lambda q:ev(q)[0],seed,jac=lambda q:ev(q)[1],bounds=(lo,hi),max_nfev=4)
        self.both_plan=res.x.copy()
        return qn+np.clip(res.x-qn,-.07,.07)
    def act(self,obs,tr):
        if self.helper.home is None:self.helper.calibrate(tr)
        active=tr['phase'] in ('staged','live');wide=active and tr['item'] in WIDE
        nxt=tr.get('upcoming',[]);prepare=tr['phase']=='gap' and nxt and nxt[0]['item'] in WIDE
        derived=dict(tr)
        if wide:
            pairs,w=self.pair(tr);(lp,la),(rp,ra)=pairs
            derived.update(grasp_local=rp.tolist(),grasp_axis=ra.tolist(),grasp_width=w)
        if self.state=='carry' and getattr(self,'travel_event',None)!=self.event:
            self.travel_event=self.event
            self._go('transit');self.travel_stage='back';self.travel_settle=0
            p0=np.asarray(tr['proprio']);self.travel_q=np.r_[p0[53:57],p0[3:10],p0[28:35]]
            self.travel_Rb=rot(tr['base_quat']);self.travel_base=np.asarray(tr['base_pos']);self.travel_yaw=math.atan2(self.travel_Rb[1,0],self.travel_Rb[0,0])
            self.travel_ps=[];self.travel_Rs=[]
            for side in ('left','right'):
                pp,rr=self.helper.measured(tr,side);self.travel_ps.append(self.travel_Rb.T@(pp-self.travel_base));self.travel_Rs.append(self.travel_Rb.T@rr)
        row=super().act(obs,derived)
        if self.state in ('transit','setdown','release'):
            pb=np.asarray(tr['base_pos']);Rb=rot(tr['base_quat']);tote=np.asarray(tr['tote']);part=np.asarray(tr['pos'])
            row=np.zeros(23);row[14]=-1.;row[22]=-1.
            targets=[pb+Rb@pp for pp in self.travel_ps];rots=[Rb@rr for rr in self.travel_Rs]
            if self.state=='transit':
                goal=np.array([tote[0]-.68,self.travel_base[1]]) if self.travel_stage=='back' else np.array([tote[0]-.68,tote[1]])
                if self.travel_stage=='back' and abs(pb[0]-goal[0])<.025:self.travel_stage='side'
                error=Rb[:2,:2].T@(goal-pb[:2]);yaw=math.atan2(Rb[1,0],Rb[0,0])-self.travel_yaw
                row[:3]=[np.clip(3*error[0],-.5,.5),np.clip(3*error[1],-.5,.5),np.clip(-3*yaw,-.4,.4)]
                q=self.travel_q
                self.travel_settle=self.travel_settle+1 if self.travel_stage=='side' and np.linalg.norm(goal-pb[:2])<.022 else 0
                if self.travel_settle>=4 or self.t_state>220:
                    self._go('setdown');self.down_ps=[self.helper.measured(tr,s)[0] for s in ('left','right')];self.down_Rs=[self.helper.measured(tr,s)[1] for s in ('left','right')];self.down_dist=max(0.,part[2]-tote[2]-.23)
            else:
                dz=min(self.down_dist,self.t_state*.006) if self.state=='setdown' else self.down_dist-.10*min(1.,max(0,self.t_state-10)/20)
                targets=[pp-np.array([0.,0.,dz]) for pp in self.down_ps];rots=self.down_Rs
                self.helper.state='carry';q=self.both_servo(tr,targets,rots)
                if self.state=='setdown' and (part[2]<tote[2]+.25 or self.t_state>100):self._go('release');self.grip=1.
                if self.state=='release':
                    row[14]=row[22]=1.
                    if self.t_state>40:self._go('rest')
            row[3:7]=q[:4];row[7:14]=q[4:11];row[15:22]=q[11:]
            self.last_assist_state=self.state
            return row
        p=np.asarray(tr['proprio']);low=np.asarray(tr['low_end']);up=np.asarray(tr['up_chute']);left=np.asarray(tr['left']);n=np.cross(up,left);n/=np.linalg.norm(n)
        tp,Rm=self.helper.measured(tr,'left');right_tp,_=self._measured(tr)
        if wide or prepare:
            if self.left_event!=tr['event']:
                self.left_event=tr['event'];self.left_grip=1.;self.left_lifted=False
            if prepare or (not self.left_lifted and self.state in ('wait','follow')):
                target=np.array([low[0]+.12,max(low[1]+.45,tr['base_pos'][1]+.20),low[2]+.30]);Rt=np.stack([np.cross(left,n),left,n],axis=1)
                if tp[2]>low[2]+.20:self.left_lifted=True
            else:target=tp.copy();Rt=Rm
            if wide and self.left_lifted:
                Rp=rot(tr['quat']);gp=np.asarray(tr['pos'])+Rp@lp;right_gp=np.asarray(tr['pos'])+Rp@rp
                ga=Rp@la;ga-=n*(ga@n);ga/=np.linalg.norm(ga);Rt=np.stack([np.cross(ga,n),ga,n],axis=1)
                if self.state in ('wait','follow'):
                    target=gp+(np.asarray(self.want)-right_gp)
                    if self.grip>0:
                        height=float(np.abs(Rp.T@n)@np.asarray(tr['size']))
                        target+=n*max(0.,height+.055-(target-low)@n)
                    off=tp-gp;horizontal=np.linalg.norm(off-n*(off@n))
                    if self.state=='follow' and horizontal<.035 and off@n<.015:self.left_grip=-1.
                elif self.state=='join':
                    target=gp-n*.010+(np.asarray(self.want)-right_tp)
                    off=tp-gp
                    if np.linalg.norm(off-n*(off@n))<.018 and off@n<.005:self.left_grip=-1.
                elif self.state in ('lift','carry','lower','drop'):
                    if not hasattr(self,'transport_event') or self.transport_event!=self.event or getattr(self,'last_assist_state',None)=='join':
                        self.transport_event=self.event;self.left_hold_R=Rm.copy()
                    Rt=self.left_hold_R;target=gp-n*.010+(np.asarray(self.want)-right_tp)
                    if self.state=='lift':self.left_grip=1.
                    elif self.grip>0:self.left_grip=1.
                else:self.left_grip=1.
            if self.state in ('join','carry','lower','drop'):
                self.helper.state='carry'
                self.helper.qpref=np.r_[.3,-.6,-.3,0., -1.8,.3,0.,-1.2,0.,.2,0., -1.8,-.3,0.,-1.2,0.,.2,0.]
                q=self.both_servo(tr,[target,np.asarray(self.want)],[Rt,self.want_R])
                row[3:7]=q[:4];row[7:14]=q[4:11];row[15:22]=q[11:]
            else:row[7:14]=self.left_servo(tr,target,Rt)
            row[14]=self.left_grip
        else:
            row[7:14]=p[3:10]+np.clip(self.home_left-p[3:10],-.07,.07);row[14]=1.;self.left_grip=1.;self.left_lifted=False
        self.last_assist_state=self.state
        return row
