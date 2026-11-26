"""Small deterministic height map built only from observed final part poses."""
import numpy as np
from narrow import rot
class Packing:
    def __init__(self):
        self.parts=[];self.last=None;self.recorded=-1;self.event=-1;self.offset=np.zeros(2);self.height=.23
        self.x=np.linspace(-.20,.20,41);self.y=np.linspace(-.15,.15,31);self.xx,self.yy=np.meshgrid(self.x,self.y,indexing='ij')
    def observe(self,tr):
        if tr['phase']=='live':self.last=tr
        elif tr['phase'] in ('gap','done') and self.last is not None and self.last['event']!=self.recorded:
            t=self.last;self.recorded=t['event'];rel=np.asarray(t['pos'])-t['tote']
            if abs(rel[0])<.21 and abs(rel[1])<.16 and -.01<rel[2]<.30:
                ext=np.abs(rot(t['quat']))@np.asarray(t['size'])/2
                self.parts.append((rel.copy(),ext.copy()))
    def choose(self,tr):
        if self.event!=tr['event']:
            self.event=tr['event'];ext=np.abs(np.asarray(tr.get('packing_rotation',rot(tr['quat']))))@np.asarray(tr['size'])/2
            grid=np.zeros(self.xx.shape)
            for p,e in self.parts:
                mask=(abs(self.xx-p[0])<e[0]+.012)&(abs(self.yy-p[1])<e[1]+.012)
                grid[mask]=np.maximum(grid[mask],p[2]+e[2])
            mx=max(0.,min(.13,.21-ext[0]-.02));my=max(0.,min(.075,.16-ext[1]-.020));best=None
            for x in np.linspace(-mx,mx,9):
                for y in np.linspace(-my,my,7):
                    mask=(abs(self.xx-x)<=ext[0]+.01)&(abs(self.yy-y)<=ext[1]+.01)
                    h=float(np.max(grid[mask])) if np.any(mask) else 0.
                    lane=float((np.asarray(tr['tote'])+np.array([x,y,0.])-np.asarray(tr['low_end']))@np.asarray(tr['left']))
                    cost=h+.012*(x+.21)+.012*(y+.16)+20*max(0.,lane+.315)
                    if best is None or cost<best[0]:best=(cost,np.array([x,y]),h)
            self.offset=best[1];self.height=float(np.clip(best[2]+ext[2]+.015,.085,.23))
        out=dict(tr);out['tote_origin']=tr['tote'];out['tote']=(np.asarray(tr['tote'])+np.r_[self.offset,0.]).tolist();out['drop_height']=self.height
        return out
