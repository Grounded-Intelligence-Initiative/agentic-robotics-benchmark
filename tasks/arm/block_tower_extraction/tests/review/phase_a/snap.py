import mujoco, numpy as np, os
_r={}
def snap(c, path, az=-60, el=-15, dist=0.6, look=None, w=640, h=480):
    m,d=c.m,c.d
    if "r" not in _r:
        m.vis.global_.offwidth=max(m.vis.global_.offwidth,w); m.vis.global_.offheight=max(m.vis.global_.offheight,h)
        _r["r"]=mujoco.Renderer(m,h,w)
    r=_r["r"]
    cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:]=look if look is not None else [0.5,0,0.33]; cam.azimuth=az; cam.elevation=el; cam.distance=dist
    r.update_scene(d,cam); img=r.render()
    import PIL.Image; PIL.Image.fromarray(img).save(path)
