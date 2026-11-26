"""Synchronized edge/handle interception and short, fixed-heading base transfer."""
import math
import numpy as np
from assist import Oracle as Assisted
from narrow import rot
from scipy.optimize import lsq_linear
from motion import PairIK

class Oracle(Assisted):
    def __init__(self,**kw):
        super().__init__(**kw);self.grips=[1.,1.];self.plan_event=None;self.base_velocity=np.zeros(3)
    def points(self,name,sz):
        if name=='003_cracker_box':
            a=np.array([1.,-1.,0.]);a/=np.linalg.norm(a)
            return [(np.array([sz[0]/2-.022,sz[1]/2-.022,0.]),a),(np.array([-sz[0]/2+.022,-sz[1]/2+.022,0.]),a)]
        if name=='035_power_drill':
            return [(np.array([.005,.075,0.]),np.array([0.,1.,0.])),(np.array([.015,-.065,0.]),np.array([0.,1.,0.]))]
        scale=sz[0]/.1649
        return [(np.array([.066,.050,0.])*scale,np.array([-.60,.80,0.])),(np.array([.066,-.050,0.])*scale,np.array([.60,.80,0.]))]
    def local_pair(self,tr,targets,rots):
        p=np.asarray(tr['proprio']);q=np.r_[p[53:57],p[3:10],p[28:35]];Rb=rot(tr['base_quat']);As=[];es=[]
        for i,side in enumerate(('left','right')):
            ix=np.r_[np.arange(4),np.arange(4+7*i,11+7*i)];J,_,_=self.helper.arms[side].jacobian(q[ix],self.helper.tool[side]);tp,Rm=self.helper.measured(tr,side)
            Rt=rots[i];flip=Rt@np.diag([-1.,-1.,1.])
            if np.trace(flip@Rm.T)>np.trace(Rt@Rm.T):Rt=flip
            Re=(Rb.T@Rt)@(Rb.T@Rm).T;er=.5*np.array([Re[2,1]-Re[1,2],Re[0,2]-Re[2,0],Re[1,0]-Re[0,1]])
            A=np.zeros((6,18));A[:,ix]=np.r_[J[:3],.25*J[3:]];As.append(A);es.extend(np.r_[Rb.T@(targets[i]-tp),.15*er])
        A=np.r_[np.concatenate(As),.02*np.eye(18)];b=np.r_[es,np.zeros(18)]
        lo=np.r_[self.helper.arms['left'].lo,self.helper.arms['right'].lo[4:]]+.05;hi=np.r_[self.helper.arms['left'].hi,self.helper.arms['right'].hi[4:]]-.05
        dlo=np.minimum(np.maximum(lo-q,-.075),.074);dhi=np.maximum(np.minimum(hi-q,.075),-.074)
        return q+lsq_linear(A,b,bounds=(dlo,dhi),method='bvls',max_iter=18,tol=1e-6).x
    def act(self,obs,tr):
        if self.helper.home is None:self.helper.calibrate(tr)
        self.t_state+=1
        p=np.asarray(tr['proprio']);qn=np.r_[p[53:57],p[3:10],p[28:35]]
        low=np.asarray(tr['low_end']);up=np.asarray(tr['up_chute']);left=np.asarray(tr['left']);n=np.cross(up,left);n/=np.linalg.norm(n)
        pb=np.asarray(tr['base_pos']);Rb=rot(tr['base_quat']);tote=np.asarray(tr['tote'])
        active=tr['phase'] in ('staged','live');nxt=tr.get('upcoming',[])
        ev=tr['event'] if active else nxt[0]['event'] if nxt else tr['event']+1
        name=tr['item'] if active else nxt[0]['item'] if nxt else ''
        sz=np.asarray(tr['size'] if active else nxt[0]['size'] if nxt else [.15,.15,.04])
        if ev!=self.plan_event:
            self.plan_event=ev;self.event=ev;self._go('wait');self.grips=[1.,1.];self.i=0.;self.closed_step=None;self.both_plan=np.r_[.3,-.6,-.3,0.,-1.5,.3,0.,-1.2,0.,.2,0.,-1.5,-.3,0.,-1.2,0.,.2,0.]
            self.prep=0;self.command=qn.copy();self.attached_once=False;self.pair_plan=PairIK(self.helper);self.joint_bias=np.zeros(18)
        measured=[self.helper.measured(tr,s)[0] for s in ('left','right')];mrs=[self.helper.measured(tr,s)[1] for s in ('left','right')]
        yawgoal=-np.pi/2
        lane=tr['lane'];pickbase=np.array([low[0]+.38,.28])
        basegoal=pb[:2].copy();q=qn.copy();targets=measured;rots=mrs
        if self.state in ('wait','follow','close'):
            self.prep+=1;basegoal=pickbase
            if active:pos=np.asarray(tr['pos']);Rp=rot(tr['quat']);vel=np.asarray(tr['vel'])
            else:
                yaw=tr['yaw']+(0. if sz[0]>=sz[1] else np.pi/2);Rp=rot([0,0,math.sin(yaw/2),math.cos(yaw/2)])
                pos=low+up*.95+left*lane+n*(sz[2]/2);vel=np.zeros(3)
            s=float((pos-low)@up);v=max(0.,-vel@up);lane=float((pos-low)@left)
            pairs=self.points(name,sz);pairs.sort(key=lambda pa:-float((Rp@pa[0])@up))
            gps=[pos+Rp@pt for pt,a in pairs];axes=[]
            for pt,a in pairs:
                ga=Rp@a;ga-=n*(ga@n);ga/=np.linalg.norm(ga);axes.append(ga)
            rots=[np.stack([np.cross(a,n),a,n],axis=1) for a in axes]
            height=float(np.abs(Rp.T@n)@sz);a=max(.01,tr['accel_slide']);d=max(0.,s-.48)
            eta=2*d/(v+math.sqrt(v*v+2*a*d)+1e-9) if active and tr['phase']=='live' else 10.
            if self.state=='wait' and eta<.35:self._go('follow')
            center=np.mean(measured,axis=0);paircenter=np.mean(gps,axis=0)
            if self.state in ('follow','close') and s<.53:self.i=float(np.clip(self.i+.10*((paircenter-center)@up),-.065,.065))
            sr=float(np.clip(s-v*.09+self.i,.08,.48))
            # Synchronize horizontally before lowering through the top surface.
            high=height+.055;h=max(.006,height-.032)
            if self.state=='wait':h=high
            elif self.state=='follow':h+=max(0.,high-h)*float(np.clip((eta-.03)/.20,0.,1.))
            center=low+up*sr+left*lane+n*h
            targets=[center+Rp@pt-n*float((Rp@pt)@n) for pt,a in pairs]
            if self.state=='close':rots=self.close_rots
            errors=[np.linalg.norm((gp-cur)-n*((gp-cur)@n)) for gp,cur in zip(gps,measured)]
            depths=[float((cur-low)@n)-(height-.032) for cur in measured]
            if self.state in ('follow','close'):
                for i in range(2):
                    if errors[i]<.030 and depths[i]<.022:self.grips[i]=-1.
                if self.state=='follow' and min(self.grips)<0:
                    self._go('close');self.close_rots=rots;self.closed_step=tr['step'];self.log.append({'event':ev,'item':name,'close_s':s,'errors':errors})
                if tr.get('attached_right') or tr.get('attached_left'):
                    self.attached_once=True;self._go('lift');self.grips=[-1.,-1.];self.hold_rots=mrs
                    self.lift_targets=[cur+np.array([0.,0.,.12]) for cur in measured]
            self.diag[ev]={'item':name,'s':s,'errors':errors,'depths':depths,'eta':eta}
            # Solve in the eventual base frame while driving into position.
            fake=dict(tr)
            if self.state=='wait' and (np.linalg.norm(pb[:2]-pickbase)>.025 or abs(math.atan2(Rb[1,0],Rb[0,0])-yawgoal)>.03):
                fake['base_pos']=[*pickbase,pb[2]];fake['base_quat']=[0.,0.,math.sin(yawgoal/2),math.cos(yawgoal/2)]
            bf=np.asarray(fake['base_pos']);rf=rot(fake['base_quat'])
            plan,cost=self.pair_plan.solve(qn,[rf.T@(tar-bf) for tar in targets],[rf.T@r for r in rots],self.state=='wait' and (not active or tr.get('release_in',0)>20))
            track=plan-qn
            if self.state=='wait' and np.max(np.abs(track))<.20:
                self.joint_bias=np.clip(self.joint_bias+.10*track,-np.r_[np.full(4,.13),np.full(14,.07)],np.r_[np.full(4,.13),np.full(14,.07)])
            q=qn+np.clip(plan+self.joint_bias-qn,-.08,.08)
            self.diag.setdefault(ev,{})['ik_cost']=cost
        elif self.state=='lift':
            targets=[]
            for cur,goal in zip(measured,self.lift_targets):
                delta=goal-cur;targets.append(cur+delta*min(1.,.018/(np.linalg.norm(delta)+1e-9)))
            q=self.local_pair(tr,targets,self.hold_rots)
            if (self.t_state>=12 and max(np.linalg.norm(cur-goal) for cur,goal in zip(measured,self.lift_targets))<.035) or self.t_state>24:
                self._go('transit');self.transfer_velocity=np.zeros(3);self.route='back';self.pick_y=pb[1];self.lower_stable=0
        elif self.state=='transit':
            # Keep the chassis outside the chute until it has cleared the low end.
            finalbase=tote[:2]+np.array([0.,.60])
            if self.route=='back':
                basegoal=np.array([min(finalbase[0],low[0]-.20),self.pick_y])
                if pb[0]<low[0]-.18:self.route='side'
            else:basegoal=finalbase
            pos=np.asarray(tr['pos'])
            clear=pos[0]<low[0]+.08 or (pos-low)@left<-.32
            goal=tote+np.array([0.,0.,.20 if clear else low[2]+.15-tote[2]])
            delta=goal-pos;velocity=delta*4.;velocity*=min(1.,.60/(np.linalg.norm(velocity)+1e-9))
            dv=velocity-self.transfer_velocity;self.transfer_velocity+=dv*min(1.,.06/(np.linalg.norm(dv)+1e-9))
            targets=[cur+self.transfer_velocity/30. for cur in measured]
            # Account for the commanded base motion in the arm IK.
            e=Rb[:2,:2].T@(basegoal-pb[:2]);bv=np.clip(4*e,-.65,.65)
            predicted=self.base_velocity.copy();predicted[:2]+=np.clip(bv-predicted[:2],-.05,.05)
            fake=dict(tr);fake['base_pos']=(pb+Rb@np.r_[predicted[:2]/30.,0.]).tolist()
            targets=[tar-Rb@np.r_[predicted[:2]/30.,0.] for tar in targets];q=self.local_pair(tr,targets,self.hold_rots)
            stable=np.linalg.norm(pos-goal)<.035 and np.linalg.norm(tr['vel'])<.16 and clear
            self.lower_stable=self.lower_stable+1 if stable else 0
            if self.lower_stable>=2 or (tr['limit_in']<16 and np.linalg.norm((pos-tote)[:2])<.10):
                self._go('release');self.release_q=qn.copy();self.release_targets=measured;self.grips=[1.,1.]
        elif self.state=='release':
            q=self.release_q if self.t_state<10 else self.both_servo(tr,[cur+np.array([0.,0.,.12]) for cur in self.release_targets],self.hold_rots)
        self.command+=np.clip(q-self.command,-.08,.08);self.command=np.clip(self.command,qn-.25,qn+.25);q=self.command.copy()
        row=np.zeros(23);row[3:7]=q[:4];row[7:14]=q[4:11];row[15:22]=q[11:];row[14],row[22]=self.grips
        if name=='035_power_drill' and self.state in ('wait','follow','close'):
            if self.grips[0]>0:row[14]=.55
            if self.grips[1]>0:row[22]=.55
        if self.state in ('wait','transit'):
            e=Rb[:2,:2].T@(basegoal-pb[:2]);yaw=(math.atan2(Rb[1,0],Rb[0,0])-yawgoal+np.pi)%(2*np.pi)-np.pi
            vel=np.r_[np.clip(4*e,-.65,.65),np.clip(-3*yaw,-1.0,1.0)]
            if self.state=='transit':
                speed=np.linalg.norm(vel[:2]);vel[:2]*=min(1.,.75/(speed+1e-9))
            self.base_velocity+=np.clip(vel-self.base_velocity,-.05,.05);row[:3]=self.base_velocity
        else:self.base_velocity=np.zeros(3)
        return row
