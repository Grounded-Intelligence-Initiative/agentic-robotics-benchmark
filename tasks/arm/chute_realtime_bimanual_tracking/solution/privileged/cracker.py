"""Two-corner interception of a box wider than either gripper."""
import math
import numpy as np
from assist import Oracle as Assist
from narrow import rot
class Oracle(Assist):
    def __init__(self,**kw):
        super().__init__(**kw);self.prep_event=-1;self.prep_steps=0;self.state='wait';self.grips=[1.,1.]
    def act(self,obs,tr):
        if self.helper.home is None:self.helper.calibrate(tr)
        self.t_state+=1
        active=tr['phase'] in ('staged','live') and tr['item']=='003_cracker_box'
        upcoming=tr.get('upcoming',[]);prepare=tr['phase']=='gap' and upcoming and upcoming[0]['item']=='003_cracker_box'
        p=np.asarray(tr['proprio']);qn=np.r_[p[53:57],p[3:10],p[28:35]];pb=np.asarray(tr['base_pos']);Rb=rot(tr['base_quat'])
        low=np.asarray(tr['low_end']);up=np.asarray(tr['up_chute']);left=np.asarray(tr['left']);n=np.cross(up,left);n/=np.linalg.norm(n);tote=np.asarray(tr['tote'])
        row=np.zeros(23);row[14]=row[22]=1.;q=qn.copy();self.diag={}
        if active and self.event!=tr['event']:
            self.event=tr['event'];self._go('wait');self.grips=[1.,1.];self.closed=0;self.i=0.;self.hmin=float('inf')
        if not active and not prepare:self._go('rest')
        if active or prepare:
            next_event=tr['event'] if active else upcoming[0]['event']
            if self.prep_event!=next_event:self.prep_event=next_event;self.prep_steps=0;self.both_plan=qn.copy()
            self.prep_steps+=1
        goal=(low+np.asarray(tr['fwd'])*.65+left*.60)[:2];yawgoal=tr['yaw']-np.pi/2
        measured=[self.helper.measured(tr,s)[0] for s in ('left','right')]
        if self.state in ('wait','follow','close') or prepare:
            if active:
                pos=np.asarray(tr['pos']);Rp=rot(tr['quat']);sz=np.asarray(tr['size']);vel=np.asarray(tr['vel']);lane=(pos-low)@left;s=(pos-low)@up;v=max(0.,-vel@up)
            else:
                pos=low+up*.95+left*tr['lane']+n*.036;Rp=np.stack([up,left,n],axis=1);sz=np.asarray(upcoming[0]['size']);lane=tr['lane'];s=.95;v=0.
            corner=np.array([sz[0]/2-.035,sz[1]/2-.035,0.]);axis=Rp@np.array([1.,-1.,0.]);axis-=n*(axis@n);axis/=np.linalg.norm(axis)
            R=np.stack([np.cross(axis,n),axis,n],axis=1)
            corners=[Rp@corner,-Rp@corner];corners.sort(key=lambda c:-(c@up))
            a=max(0.,tr.get('accel_slide',0.));distance=max(0.,s-.65);eta=2*distance/(v+np.sqrt(v*v+2*a*distance)+1e-9)
            if not active or tr['phase']=='staged':eta=10.
            if self.state=='wait' and active and tr['phase']=='live' and eta<.4:self._go('follow')
            height=float(np.abs(Rp.T@n)@sz)
            if self.state in ('follow','close') and s<=.65:self.i=np.clip(self.i+.08*((pos-(measured[0]+measured[1])/2)@up),-.07,.07)
            sr=np.clip(s-v*.04+getattr(self,'i',0.),.10,.65)
            h=height/2-.022+(height/2+.065)*np.clip((eta-.03)/.12,0.,1.) if self.state in ('follow','close') else height+.10
            if self.state in ('follow','close'):self.hmin=min(self.hmin,h);h=self.hmin
            center=low+up*sr+left*lane+n*h
            targets=[center+c for c in corners];rots=[R,R]
            if self.state=='close':rots=self.closing_R
            if self.state=='wait' or prepare:
                fake=dict(tr);fake['base_pos']=[*goal,pb[2]];fake['base_quat']=[0.,0.,math.sin(yawgoal/2),math.cos(yawgoal/2)]
                planned=self.both_servo(fake,targets,rots)
                if self.prep_steps<25:
                    safe=np.r_[.3,-.6,-.3,0.,0.,1.3,0.,0.,0.,0.,0.,0.,-1.3,0.,0.,0.,0.,0.];q=qn+np.clip(safe-qn,-.07,.07)
                else:q=planned
            else:q=self.both_servo(tr,targets,rots)
            if active and self.state in ('follow','close'):
                gps=[pos+c for c in corners];errors=[np.linalg.norm((gp-cur)-n*((gp-cur)@n)) for gp,cur in zip(gps,measured)];depths=[(cur-gp)@n for gp,cur in zip(gps,measured)]
                self.diag[self.event]={'errors':np.round(errors,3).tolist(),'depths':np.round(depths,3).tolist(),'s':round(float(s),3),'mode':'box_pair'}
                if self.state=='follow' and max(errors)<.032 and (max(depths)<.040 or all(any(tr['finger_contact_'+side]) for side in ('left','right'))):
                    self.grips=[-1.,-1.];self.closing_R=[self.helper.measured(tr,side)[1] for side in ('left','right')];self._go('close');self.closed=tr['step']
                    self.log.append({'event':self.event,'item':tr['item'],'close_s':float(s)})
                if self.state=='close' and (tr.get('attached_right') or tr.get('attached_left') or (tr['step']-self.closed>15 and all(any(tr['finger_contact_'+side]) for side in ('left','right')))):
                    self.lift_ps=[m+np.array([0.,0.,.22]) for m in measured];self.lift_Rs=[self.helper.measured(tr,side)[1] for side in ('left','right')];self._go('lift')
        elif self.state=='lift':
            q=self.both_servo(tr,self.lift_ps,self.lift_Rs)
            if self.t_state>30:
                self._go('back');self.drive_q=qn.copy();self.back_y=pb[1]
        elif self.state in ('back','side'):
            q=self.drive_q;goal=np.array([tote[0]-.68,self.back_y if self.state=='back' else tote[1]]);yawgoal=self.helper.yaw0
            if np.linalg.norm(pb[:2]-goal)<.025 and abs(math.atan2(Rb[1,0],Rb[0,0])-yawgoal)<.04:
                if self.state=='back':self._go('side')
                else:self._go('lower');self.lower_stable=0;self.lower_Rs=[self.helper.measured(tr,side)[1] for side in ('left','right')]
        elif self.state=='lower':
            pos=np.asarray(tr['pos']);goalpart=tote+np.array([0.,0.,.24]);delta=np.clip(goalpart-pos,-.10,.10);targets=[m+delta for m in measured]
            q=self.both_servo(tr,targets,self.lower_Rs)
            stable=np.linalg.norm(pos-goalpart)<.025 and np.linalg.norm(tr['vel'])<.08
            self.lower_stable=self.lower_stable+1 if stable else 0
            if self.lower_stable>=6 or self.t_state>180:
                self._go('release');self.release_ps=[m.copy() for m in measured];self.grips=[1.,1.]
        elif self.state=='release':
            q=self.both_servo(tr,[m+np.array([0.,0.,.14 if self.t_state>10 else 0.]) for m in self.release_ps],self.lower_Rs)
            if self.t_state>40:self._go('rest')
        if self.state in ('wait','back','side') or prepare:
            e=Rb[:2,:2].T@(goal-pb[:2]);yaw=(math.atan2(Rb[1,0],Rb[0,0])-yawgoal+np.pi)%(2*np.pi)-np.pi
            row[:3]=[np.clip(4*e[0],-.8,.8),np.clip(4*e[1],-.8,.8),np.clip(-3*yaw,-1.2,1.2)]
        if prepare:self.grips=[1.,1.] if self.prep_steps>65 and np.linalg.norm(pb[:2]-goal)<.04 and max(np.linalg.norm(cur-tar) for cur,tar in zip(measured,targets))<.05 else [-1.,-1.]
        row[3:7]=q[:4];row[7:14]=q[4:11];row[15:22]=q[11:];row[14],row[22]=self.grips
        return row
