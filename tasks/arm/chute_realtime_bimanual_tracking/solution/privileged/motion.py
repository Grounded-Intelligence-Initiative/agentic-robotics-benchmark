"""Bounded, deterministic IK work per reply, with a fixed preferred branch."""
import numpy as np
from scipy.optimize import least_squares

class PoseIK:
    def __init__(self,arm,tool):
        self.arm=arm;self.tool=tool;self.plan=None
        self.pref=np.array([.45,-.65,-.25,0.,-.25,-.65,.25,-1.25,0.,.1,0.])
    def solve(self,qn,p,R,reset=False):
        lo=self.arm.lo+.10;hi=self.arm.hi-.10
        # Keep the mobile torso near its normal working range.
        lo[:3]=np.maximum(lo[:3],[-.2,-1.4,-.9]);hi[:3]=np.minimum(hi[:3],[1.1,.7,.9])
        seed=np.clip(self.pref if reset or self.plan is None else self.plan,lo,hi)
        cache=[None,None,None]
        def ev(q):
            if cache[0] is not None and np.array_equal(q,cache[0]):return cache[1],cache[2]
            J,pk,Rk=self.arm.jacobian(q,self.tool)
            err=np.r_[pk-p,(.25*(Rk-R)).ravel(),.006*(q-self.pref)]
            A=np.zeros((23,11));A[:3]=J[:3]
            for k in range(11):A[3:12,k]=(.25*np.cross(J[3:,k],Rk.T).T).ravel()
            A[12:]=.006*np.eye(11)
            cache[:]=[q.copy(),err,A];return err,A
        res=least_squares(lambda q:ev(q)[0],seed,jac=lambda q:ev(q)[1],bounds=(lo,hi),max_nfev=5)
        self.plan=res.x.copy()
        return res.x

class PairIK:
    """Small fixed IK slices; search symmetric wrist branches during the feed delay."""
    def __init__(self,helper):
        self.helper=helper;self.ixs=[np.r_[np.arange(4),np.arange(4+7*i,11+7*i)] for i in range(2)]
        self.pref=np.r_[.35,-.6,-.3,0.,-1.,.5,0.,-1.1,0.,.2,0.,-1.,-.5,0.,-1.1,0.,.2,0.]
        self.lo=np.r_[helper.arms['left'].lo,helper.arms['right'].lo[4:]]+.07
        self.hi=np.r_[helper.arms['left'].hi,helper.arms['right'].hi[4:]]-.07
        self.best=None;self.flip=0;self.iter=0;self.candidate=None
    def evaluate(self,q,ps,Rs,jac=True):
        es=[];As=[]
        for i,side in enumerate(('left','right')):
            ix=self.ixs[i];arm=self.helper.arms[side]
            if jac:J,p,R=arm.jacobian(q[ix],self.helper.tool[side])
            else:p,R,_=arm.fk(q[ix],self.helper.tool[side])
            es.extend(p-ps[i]);es.extend((.24*(R-Rs[i])).ravel())
            if jac:
                A=np.zeros((12,18));A[:3,ix]=J[:3]
                for k in range(11):A[3:,ix[k]]=(.24*np.cross(J[3:,k],R.T).T).ravel()
                As.append(A)
        es.extend(.005*(q-self.pref))
        if jac:As.append(.005*np.eye(18))
        return np.asarray(es),np.concatenate(As) if jac else None
    def refine(self,q,ps,Rs,steps):
        cache=[None,None,None]
        def ev(q):
            if cache[0] is None or not np.array_equal(q,cache[0]):
                e,A=self.evaluate(q,ps,Rs);cache[:]=[q.copy(),e,A]
            return cache[1],cache[2]
        r=least_squares(lambda q:ev(q)[0],np.clip(q,self.lo,self.hi),jac=lambda q:ev(q)[1],bounds=(self.lo,self.hi),max_nfev=steps)
        return r.x,float(np.linalg.norm(r.fun))
    def solve(self,qn,ps,Rs,search):
        def flipped(bit):return [R@(np.diag([-1.,-1.,1.]) if bit>>i&1 else np.eye(3)) for i,R in enumerate(Rs)]
        if self.best is None:self.best=np.clip(self.pref,self.lo,self.hi)
        self.best,cost=self.refine(self.best,ps,flipped(self.flip),3 if search else 5)
        if search and self.iter<96:
            branch=self.iter//8;bit=branch%4
            if self.iter%8==0:
                seed=self.pref.copy()
                if branch//4==1:seed[:4]=[.8,-.3,.2,0.];seed[[4,11]]=.3;seed[[6,13]]=[1.,-1.]
                elif branch//4==2:seed[:4]=[1.,-.8,-.4,0.];seed[[4,11]]=-3.;seed[[6,13]]=[-1.,1.]
                self.candidate=np.clip(seed,self.lo,self.hi)
            self.candidate,cc=self.refine(self.candidate,ps,flipped(bit),5)
            if cc<cost:self.best=self.candidate.copy();self.flip=bit;cost=cc
            self.iter+=1
        return self.best.copy(),cost
