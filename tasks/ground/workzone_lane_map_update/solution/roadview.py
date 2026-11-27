"""World-space appearance and roadside occluders for the calibrated survey."""

from hashlib import blake2b
from functools import lru_cache
from bisect import bisect_left, bisect_right
from io import BytesIO
from math import ceil, cos, floor, pi, sin
from random import Random

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

from geometry import cross_section
from render import NEAR, SIZE, scene


# Private surface IDs used by rasterization and visibility checks. Only JPEGs leave setup.
COLORS = ("sky", "#555961", "#f0ce69", "#f9eec6", "#dad9cc", "#cdd1c5", "#929892",
          "#eaa85d", "#30b8df", "#222730", "#758267", "#989082", "#393632", "#ff7026",
          "#fff4e0", "#68717a", "#54f58a", "#f13755", "body", "roof", "glass", "rubber",
          "metal", "warning", "trunk", "leaves", "lamp", "access_face", "#a59b7e", "#aaa58b", "#827e70")
ID = {color:i for i,color in enumerate(COLORS)}
ROAD, EDGE, ARROW, STRIPE, TOP, WALL, BAND = range(1,8)
PROP = np.arange(ID["body"], ID["warning"]+1)
SIGN_W, SIGN_H, SIGN_TOP = 2.30,1.32,3.65


@lru_cache(maxsize=2)
def sign_ink(public):
    bitmap = Image.new("L",(228,132),0)
    draw = ImageDraw.Draw(bitmap)
    font = ImageFont.load_default(size=48)
    for y,line in zip((33,96),("PUBLIC","ACCESS") if public else ("WORK","ONLY")):
        draw.text((114,y),line,anchor="mm",font=font,fill=255,stroke_width=1)
    return np.asarray(bitmap,dtype="f")/255


class SurfaceBuffer:
    def __init__(self, camera):
        self.camera = camera
        u = (np.arange(SIZE[0],dtype="f")+.5-camera.cx)/camera.focal
        v = (np.arange(SIZE[1],dtype="f")+.5-camera.cy)/camera.focal
        self.rays = [f+r*u[None,:]+d*v[:,None] for f,r,d in zip(camera.forward,camera.right,camera.down)]
        self.depth = np.full(SIZE[::-1], 1000., dtype="f")
        self.ids = np.zeros(SIZE[::-1], dtype=np.uint8)
        self.light = np.ones(SIZE[::-1], dtype="f")

    def world_polygon(self, points, color):
        surface = getattr(self.camera,"surface",None)
        if surface is None:
            self._rasterize(points,color)
            return
        ys=[p[1] for p in points]
        cuts=surface.knots[bisect_right(surface.knots,min(ys)):bisect_left(surface.knots,max(ys))]
        remaining=points
        for station in cuts:
            low,high=[],[]
            for a,b in zip(remaining,remaining[1:]+remaining[:1]):
                if a[1]<=station:
                    low.append(a)
                if a[1]>=station:
                    high.append(a)
                if (a[1]<station<b[1]) or (b[1]<station<a[1]):
                    f=(station-a[1])/(b[1]-a[1])
                    intersection=(a[0]+f*(b[0]-a[0]),station,a[2]+f*(b[2]-a[2]))
                    low.append(intersection)
                    high.append(intersection)
            if len(low)>=3:
                self._rasterize([(x,y,z+surface.height(x,y)) for x,y,z in low],color)
            remaining=high
        if len(remaining)>=3:
            self._rasterize([(x,y,z+surface.height(x,y)) for x,y,z in remaining],color)

    def _rasterize(self, points, color):
        cam = self.camera
        vertices = [cam.camera_point(p) for p in points]
        if max(p[2] for p in vertices) < NEAR:
            return
        clipped = []
        for a,b in zip(vertices,vertices[1:]+vertices[:1]):
            if a[2] >= NEAR:
                clipped.append(a)
            if (a[2] >= NEAR) != (b[2] >= NEAR):
                t = (NEAR-a[2])/(b[2]-a[2])
                clipped.append(tuple(x+t*(y-x) for x,y in zip(a,b)))
        pixels = [cam.pixel(p) for p in clipped]
        xs,ys = zip(*pixels)
        x0,x1 = max(0,floor(min(xs))),min(SIZE[0],ceil(max(xs))+1)
        y0,y1 = max(0,floor(min(ys))),min(SIZE[1],ceil(max(ys))+1)
        if x0 >= x1 or y0 >= y1:
            return
        mask = Image.new("1",(x1-x0,y1-y0))
        ImageDraw.Draw(mask).polygon([(x-x0,y-y0) for x,y in pixels],fill=1)
        region = np.s_[y0:y1,x0:x1]
        if all(p[2] == points[0][2] for p in points):
            numerator = points[0][2]-cam.position[2]
            denominator = self.rays[2][region]
        else:
            a,b,c = np.asarray([points[0], points[1], points[-1]])
            n = np.cross(b-a,c-a)
            if np.dot(n,n)<1e-14:
                offsets=np.asarray(points)-a
                normals=np.cross(offsets[:-1],offsets[1:])
                n=normals[np.argmax((normals*normals).sum(axis=1))]
                if np.dot(n,n)<1e-14:
                    return
            numerator = np.dot(n,a-cam.position)
            denominator = sum(n[i]*self.rays[i][region] for i in range(3))
        with np.errstate(divide="ignore",invalid="ignore"):
            depth = numerator/denominator
        visible = np.asarray(mask) & (depth >= NEAR) & (depth <= self.depth[region]+.003)
        self.depth[region][visible] = depth[visible]
        self.ids[region][visible] = ID[color]
        if color in ("body","roof","glass","trunk","leaves","rubber","metal","warning","lamp"):
            normal = np.cross(np.subtract(points[1],points[0]),np.subtract(points[-1],points[0]))
            normal = normal/max(1e-8,np.linalg.norm(normal))
            self.light[region][visible] = .81+.19*abs(float(normal @ [.45,-.48,.75]))


def box(faces, x, y, z, width, length, height, sides, top):
    base = [(x-width/2,y-length/2,z),(x+width/2,y-length/2,z),
            (x+width/2,y+length/2,z),(x-width/2,y+length/2,z)]
    roof = [(a,b,z+height) for a,b,_ in base]
    faces.append((roof,top))
    for i,j in ((0,1),(1,2),(2,3),(3,0)):
        faces.append(([base[i],base[j],roof[j],roof[i]],sides))


def shadow_poly(draw, base, height, sun):
    # The footprint and every extruded side project to one connected shadow.
    def uv(p):
        return ((p[0]+32)*12,(p[1]+48)*12)
    shifted = [(x+height*sun[0],y+height*sun[1]) for x,y in base]
    draw.polygon([uv(p) for p in base],fill=240)
    draw.polygon([uv(p) for p in shifted],fill=240)
    for i in range(len(base)):
        j = (i+1)%len(base)
        draw.polygon([uv(p) for p in (base[i],base[j],shifted[j],shifted[i])],fill=240)


class RoadAppearance:
    def __init__(self, layout, fixtures, attempt=0):
        key = f"road-survey-v1/{layout['length'].hex()}/{attempt}"
        self.seed = int.from_bytes(blake2b(key.encode(),digest_size=8).digest(),"big")
        rng = Random(self.seed)
        self.tile = np.random.default_rng(self.seed).uniform(-1,1,(256,256)).astype("f")
        self.phase = rng.uniform(-5,5)
        self.road = rng.uniform(77,113)
        self.concrete = rng.uniform(128,171)
        self.paint = rng.uniform(178,212)
        self.warm = rng.uniform(-7,10)
        self.wear = rng.uniform(.37,.57) if attempt < 3 else .29
        self.edge_alpha_floor = .38 if attempt >= 3 else 0
        self.sun = (rng.choice((-1,1))*rng.uniform(1.1,2.3),rng.uniform(-1.3,1.3))
        self.shade = rng.uniform(.38,.55)
        self.faces = []
        self.signs = layout.get("access_signs",[])
        for sign in self.signs:
            x,y = sign["x"],sign["y"]
            box(self.faces,x,y+.10,0,.13,.12,SIGN_TOP,"metal","metal")
            box(self.faces,x,y+.035,SIGN_TOP-SIGN_H,SIGN_W,.055,SIGN_H,"metal","metal")
            self.faces.append(([(x-SIGN_W/2,y,SIGN_TOP-SIGN_H),(x+SIGN_W/2,y,SIGN_TOP-SIGN_H),
                                (x+SIGN_W/2,y,SIGN_TOP),(x-SIGN_W/2,y,SIGN_TOP)],"access_face"))
        length = layout["length"]
        shadow = Image.new("L",(768,ceil((length+110)*12)))
        pen = ImageDraw.Draw(shadow)
        for fixture in fixtures:
            if fixture["kind"]!="milled_surface":
                shadow_poly(pen,fixture["poly"],.23 if fixture["kind"]=="channel_island" else .68,self.sun)
        for y in np.arange(7,length+20,15.5):
            station = y+rng.uniform(-4,4)
            sign = rng.choice((-1,1))
            edge = layout["access_edges"]["right" if sign>0 else "left"]
            x = cross_section(edge,station)+sign*rng.uniform(2.15,2.70)
            if attempt >= 3:
                x += 1.5*sign
            if rng.random() < .34:
                self.van(x,station)
                w,l,h = 1.85,4.7,2.1
            else:
                box(self.faces,x,station,0,.13,.13,1.9,"metal","metal")
                box(self.faces,x,station,.73,1.32,.13,.98,"warning","metal")
                for k in (-.34,.34):
                    box(self.faces,x+k,station-.075,.85,.18,.025,.67,"rubber","rubber")
                w,l,h = 1.34,.16,1.85
            base = [(x-w/2,station-l/2),(x+w/2,station-l/2),(x+w/2,station+l/2),(x-w/2,station+l/2)]
            shadow_poly(pen,base,h,self.sun)
        for sign in (-1,1):
            for y in np.arange(-12,length+75,10):
                x = sign*rng.uniform(10.7,22)
                y += rng.uniform(-3,3)
                h = rng.uniform(4.2,7.7)
                box(self.faces,x,y,0,.33,.37,h,"trunk","trunk")
                for offset in (-1,0,1):
                    tx,ty=x+offset*rng.uniform(.8,1.5),y+rng.uniform(-.8,.8)
                    z=h-abs(offset)*.55
                    rings = []
                    for radius,dz in ((.65,-1.8),(1.85,-.85),(2.05,.15),(1.42,1.22),(.57,1.86)):
                        rings.append([(tx+cos(k*pi/4)*radius*rng.uniform(.87,1.13),
                                       ty+sin(k*pi/4)*radius*rng.uniform(.87,1.13),z+dz+rng.uniform(-.22,.22))
                                      for k in range(8)])
                    self.faces.append((rings[-1],"leaves"))
                    for bottom,top in zip(rings,rings[1:]):
                        for i in range(8):
                            j=(i+1)%8
                            self.faces.append(([bottom[i],bottom[j],top[j],top[i]],"leaves"))
                    shadow_poly(pen,[(a,b) for a,b,_ in rings[2]],z,self.sun)
        self.shadow = np.asarray(shadow.filter(ImageFilter.GaussianBlur(2.2)),dtype="f")/255

    def van(self,x,y):
        faces = self.faces
        box(faces,x,y,.38,1.85,4.7,.88,"body","roof")
        box(faces,x,y+.52,1.23,1.78,3.58,.84,"body","roof")
        windshield = [(x-.84,y-1.94,1.27),(x+.84,y-1.94,1.27),
                      (x+.82,y-1.23,2.00),(x-.82,y-1.23,2.00)]
        faces.append((windshield,"glass"))
        for s in (-1,1):
            faces.append(([(x+s*.88,y-1.97,1.25),(x+s*.88,y-1.25,2.05),
                           (x+s*.88,y-1.25,1.25)],"body"))
        box(faces,x,y-2.36,.39,1.83,.07,.13,"rubber","rubber")
        box(faces,x,y-2.38,.71,.83,.03,.23,"rubber","metal")
        box(faces,x,y-2.40,.52,.41,.03,.10,"lamp","lamp")
        for s in (-1,1):
            for d in (-1.39,1.39):
                for radius,material,axle in ((.34,"rubber",.95),(.165,"metal",.97)):
                    wheel = [(x+s*axle,y+d+radius*cos(k*pi/8),.35+radius*sin(k*pi/8)) for k in range(16)]
                    faces.append((wheel,material))
            box(faces,x+s*.884,y-.54,1.38,.038,1.15,.51,"glass","glass")
            box(faces,x+s*.932,y+.12,.77,.035,4.18,.10,"warning","warning")
            box(faces,x+s*.68,y-2.36,.67,.34,.04,.19,"lamp","lamp")

    def noise(self,x,y,scale):
        u,v = x/scale,y/scale
        i,j = np.floor(u).astype("i"),np.floor(v).astype("i")
        a,b = u-i,v-j
        table = self.tile
        return ((1-b)*((1-a)*table[j%256,i%256]+a*table[j%256,(i+1)%256])
                +b*((1-a)*table[(j+1)%256,i%256]+a*table[(j+1)%256,(i+1)%256])).astype("f")

    def photograph(self, canvas):
        ids,cam = canvas.ids,canvas.camera
        depth = np.minimum(canvas.depth,320)
        x,y,z = [p+r*depth for p,r in zip(cam.position,canvas.rays)]
        if hasattr(cam,"surface"):
            z=z-cam.surface.height(x,y)
        coarse = self.noise(x,y,4.5)
        medium = self.noise(x+.72*z,y+.43*z,.72)
        grain = self.noise(x+.3*z,y+z,.085)
        fine = self.noise(x,y,.23)
        texture = 14*coarse+10*medium+11*grain
        patch = self.noise(x+5,y-8,3.1)
        pavement = self.road+texture+14*(patch>.23)-10*(abs(patch-.23)<.024)
        pavement -= 8*np.exp(-((np.sin(1.42*x+.014*y+self.phase))/.35)**2)
        sx = np.clip(((x+32)*12).astype("i"),0,self.shadow.shape[1]-1)
        sy = np.clip(((y+48)*12).astype("i"),0,self.shadow.shape[0]-1)
        illumination = 1-self.shade*self.shadow[sy,sx]
        scar = ids==ID["#a59b7e"]
        road_like = (ids==ROAD)|(ids==EDGE)|(ids==ARROW)|(ids==STRIPE)|scar
        # Aged pigment reveals the SAME underlying asphalt texture and shadow across cameras.
        alpha = np.clip(.60+.65*medium+.70*grain+.25*fine-self.wear*.35,0,1)
        if self.edge_alpha_floor:
            alpha = np.where(ids==EDGE,np.maximum(alpha,self.edge_alpha_floor),alpha)
        pigment = np.where(ids==EDGE,self.paint-12,self.paint+8)+11*fine
        luminance = np.where(ids==ROAD,pavement,(1-alpha)*pavement+alpha*pigment)
        # Grinding and patching remove different amounts of the old stripe along its length.
        remnant=np.clip(.42+.60*coarse+.56*medium+.45*fine,0,.86)
        scar_luminance=(1-remnant)*(pavement-10)+remnant*(self.paint-16)
        luminance=np.where(scar,scar_luminance,luminance)
        base = np.zeros((*ids.shape,3),dtype="f")
        tint=(ids==EDGE)*alpha+scar*remnant
        for ch, bias in enumerate((3,1,-3)):
            base[:,:,ch] = (luminance + bias + tint*(10,0,-42)[ch])*illumination
        mats = {"#cdd1c5":(self.concrete+15,)*3, "#929892":(self.concrete-14,)*3,
                "#eaa85d":(self.concrete+12,self.concrete+4,self.concrete-14),
                "#30b8df":(53,156,171),"#222730":(49,52,53),"#758267":(105,115,84),
                "#989082":(127,120,108),"#393632":(50,49,45),"#ff7026":(203,109,52),
                "#fff4e0":(207,198,174),"#68717a":(111,116,117),"#54f58a":(76,217,138),
                "#f13755":(211,65,68),"body":(160+self.warm,154,140),
                "roof":(180,177,165),"glass":(59,77,83),"rubber":(50,50,48),
                "metal":(107,111,109),"warning":(199,148,65),"trunk":(90,78,62),
                "leaves":(78,105,68),"lamp":(204,198,163)}
        lut = np.zeros((len(COLORS),3),dtype="f")
        for k,col in mats.items():
            lut[ID[k]] = col
        lut[ID["#aaa58b"]] = self.concrete+5
        lut[ID["#827e70"]] = self.concrete-19
        concrete = (ids==TOP)|(ids==WALL)|(ids==BAND)|(ids==ID["#aaa58b"])|(ids==ID["#827e70"])
        mottling = texture+10*fine-14*np.exp(-((z-.12)/.19)**2)
        amplitude = np.where((ids>=ID["body"]) & (ids<=ID["metal"]),.18,1.)
        mottling = np.where(ids==ID["leaves"],16*fine+24*grain+14*medium,mottling*amplitude)
        material = (lut[ids]+mottling[:,:,None])*(.82+.18*illumination[:,:,None])*canvas.light[:,:,None]
        # Concrete also passes through broader sun/shade zones; pale sunlit concrete and
        # shaded or worn paint can have the same apparent brightness.
        material *= np.where(concrete,.83+.23*coarse,1)[:,:,None]
        base = np.where(road_like[:,:,None],base,material)
        for sign in self.signs:
            select = (ids==ID["access_face"]) & (abs(y-sign["y"])<.04)
            if not np.any(select):
                continue
            ink = sign_ink(sign["public"])
            ix = np.clip(((x[select]-sign["x"])/SIGN_W+.5)*ink.shape[1],0,ink.shape[1]-1).astype("i")
            iz = np.clip((SIGN_TOP-z[select])/SIGN_H*ink.shape[0],0,ink.shape[0]-1).astype("i")
            shade = ink[iz,ix,None]
            base[select] = ((1-shade)*np.array([223,193,132])+shade*np.array([39,38,34]))*(.82+.18*illumination[select,None])
        sky = ids==0
        v = np.arange(SIZE[1],dtype="f")[:,None]/SIZE[1]
        sky_rgb = np.stack(np.broadcast_arrays(151+65*v,177+47*v,194+34*v),axis=2)
        base = np.where(sky[:,:,None],sky_rgb,base)
        exposure = .96+.07*sin(.09*cam.position[1]+self.phase)
        sensor_seed = self.seed ^ (int((cam.position[1]+100)*1000) & 0xffffffff)
        noise = np.random.default_rng(sensor_seed).normal(0,1.45,ids.shape).astype("f")
        rgb = (base+np.array([self.warm*.45,0,-self.warm*.4],dtype="f"))*exposure+noise[:,:,None]
        image = Image.fromarray(np.clip(rgb,0,255).astype("uint8")).filter(ImageFilter.GaussianBlur(.40))
        with BytesIO() as stream:
            image.save(stream,format="JPEG",quality=84,subsampling=2)
            encoded = stream.getvalue()
        return encoded, alpha

    def render(self, camera, layout, paths, cones, signals, painted, barriers, fixtures):
        surface = SurfaceBuffer(camera)
        scene(camera,layout,paths,cones,signals,painted,barriers,fixtures,canvas=surface)
        for points,color in self.faces:
            # Distant vegetation is atmospheric background; cull objects beyond the far clip.
            if points[0][1]<camera.position[1]-6 or points[0][1]>camera.position[1]+110:
                continue
            surface.world_polygon(points,color)
        encoded, alpha = self.photograph(surface)
        return encoded, surface.ids, alpha


def projected(cam, x, y, z=0):
    if hasattr(cam,"surface"):
        z+=cam.surface.height(x,y)
    point = cam.camera_point((x,y,z))
    if not 2 < point[2] < 58:
        return None
    u,v = map(round,cam.pixel(point))
    return (u,v) if 3 <= u < SIZE[0]-3 and 3 <= v < SIZE[1]-3 else None


def read_arrow(camera, arrow, ids, alpha, luma):
    x,y,slope = arrow
    corners = [projected(camera,x+dx+slope*dy,y+dy) for dx in (-.5,.5) for dy in (-1.35,1.1)]
    if None in corners:
        return None
    us,vs = zip(*corners)
    u0,v0 = max(0,min(us)-3),max(0,min(vs)-3)
    region = np.s_[v0:max(vs)+4,u0:max(us)+4]
    material, brightness = ids[region],luma[region]
    road = brightness[material==ROAD]
    if road.size < 8:
        return None
    visible = (material==ARROW) & (alpha[region]>.24) & (brightness > np.median(road)+19)
    yy,xx = np.nonzero(visible)
    if len(xx) < 5 or np.ptp(yy) < 1:
        return None
    # Reconstruct each arrow's visible centroid from the final-image contrast test.
    return camera.ground_point(u0+float(np.median(xx)),v0+float(np.median(yy)))


def assess_frame(camera, ids, alpha, decoded, layout):
    luma = decoded.astype("f").mean(axis=2)
    left,right = (layout["access_edges"][k] for k in ("left","right"))
    def center(y):
        if "current_divider" in layout:
            return cross_section(layout["current_divider"],y)
        return (cross_section(left,y)+cross_section(right,y))/2
    def surface_visible(y, solid):
        pos = projected(camera,center(y),y,.68 if solid else 0)
        if pos is None:
            return False
        u,v = pos
        patch = ids[v-1:v+2,u-1:u+2]
        selected = (patch==TOP)|(patch==WALL)|(patch==BAND) if solid else patch==ROAD
        if np.count_nonzero(selected) < 3:
            return False
        if solid:
            neighborhood = luma[max(0,v-5):v+6,max(0,u-5):u+6]
            return np.ptp(neighborhood) >= 15
        return True
    findings = {}
    for target in layout["visibility_targets"]:
        y,h = target["station"],target["half_gap"]
        middle = all(surface_visible(y+d,not target["open"]) for d in (-1.8,0,1.8))
        ends = [surface_visible(y+d,True) for d in (-h-1.1,h+1.1)] if target["open"] else [True]
        readable = False
        # Survey frames share a map datum. The pair of arrows can establish
        # direction in a different frame from the one showing both median ends.
        if target.get("check_direction",target["name"]=="temporary"):
            a,b = [read_arrow(camera,arrow,ids,alpha,luma) for arrow in target["arrows"]]
            if a and b:
                observed = (b[0]-center(b[1]))-(a[0]-center(a[1]))
                first,second = target["arrows"]
                expected = (second[0]-center(second[1]))-(first[0]-center(first[1]))
                readable = abs(observed)>.75 and observed*expected>0
        findings[target["name"]] = (middle and all(ends),readable)
    return findings


def audit_colors(decoded, ids, clip, counts):
    """Measure held-out RGB lookup performance; internal labels are never written to the survey."""
    rgb, labels = decoded[::2,::2].astype("i"), ids[::2,::2]
    bins = (rgb[:,:,0]//8)*1024+(rgb[:,:,1]//8)*32+rgb[:,:,2]//8
    key = "oblique" if clip.endswith("oblique") or clip=="elevated" else "forward"
    masks = (labels==ROAD, (labels==TOP)|(labels==WALL)|(labels==BAND), (labels==EDGE)|(labels==ARROW))
    colors = (("#555961",),("#cdd1c5","#929892","#eaa85d"),("#f0ce69","#f9eec6"))
    for i,(mask,old) in enumerate(zip(masks,colors)):
        counts[key][i] += np.bincount(bins[mask],minlength=32768)
        pixels = rgb[mask]
        counts["exact"][i] += sum(np.count_nonzero(np.all(pixels==tuple(bytes.fromhex(c[1:])),axis=1)) for c in old)
        counts["total"][i] += len(pixels)


def visible_edge_points(camera, layout, ids, alpha, decoded):
    """Require repeat sightings of the same paint sample on each bend."""
    for bank,edge in layout["access_edges"].items():
        for fraction in (.25,.4,.6,.75,*layout.get("pinch_stations",{}).get(bank,[])):
            station = fraction*layout["length"]
            occupied = layout.get("occupied_bank")
            if occupied and bank==occupied[0] and occupied[1]-2.5 <= station <= occupied[2]+2.5:
                # Here the work enclosure covers the old edge. Check the approach
                # or departure stripe beside its end. The enclosure has its own check.
                start,end = occupied[1:]
                station = start-4.5 if station < (start+end)/2 else end+4.5
            for delta in (-2.5,0,2.5):
                y = station+delta
                uv = projected(camera,cross_section(edge,y),y)
                if uv is None:
                    continue
                u,v = uv
                box = np.s_[v-3:v+4,u-3:u+4]
                paint = (ids[box]==EDGE) & (alpha[box]>.24)
                road = ids[box]==ROAD
                if paint.sum() >= 3 and road.sum() >= 3:
                    patch = decoded[box].mean(axis=2)
                    if np.max(patch[paint])-np.median(patch[road]) >= 12:
                        yield bank,fraction,delta


def legible_sign(camera, sign, ids, decoded):
    """Read the rectified JPEG against both printed legends, with equal colour priors."""
    sx,sy = sign["x"],sign["y"]
    corners=[projected(camera,sx+dx,sy,SIGN_TOP+dz) for dx in (-SIGN_W/2,SIGN_W/2) for dz in (-SIGN_H,0)]
    if None in corners or min(abs(corners[i][1]-corners[i+1][1]) for i in (0,2))<18:
        return False
    ink = sign_ink(sign["public"])
    rows,cols = np.mgrid[1:ink.shape[0]:3,1:ink.shape[1]:3]
    xs=sx+SIGN_W*((cols+.5)/ink.shape[1]-.5)
    ys=np.full(cols.shape,sy)
    zs=SIGN_TOP-SIGN_H*(rows+.5)/ink.shape[0]
    if hasattr(camera,"surface"):
        zs+=camera.surface.height(xs,ys)
    points = np.stack((xs,ys,zs),axis=-1)-camera.position
    depth=points@camera.forward
    u=np.rint(camera.cx+camera.focal*(points@camera.right)/depth).astype(int)
    v=np.rint(camera.cy+camera.focal*(points@camera.down)/depth).astype(int)
    mask=ids[v,u]==ID["access_face"]
    if mask.mean()<.86:
        return False
    light=decoded[v[mask],u[mask]].mean(axis=1)
    expected=ink[rows[mask],cols[mask]]
    alternate=sign_ink(not sign["public"])[rows[mask],cols[mask]]
    if np.std(light)<14:
        return False
    matches=[np.corrcoef(-light,legend)[0,1] for legend in (expected,alternate)]
    return matches[0]>.45 and matches[0]-matches[1]>.14


def visible_channel(camera, target, ids, decoded):
    for point,material in (("curb_upstream",ID["#aaa58b"]),("curb_downstream",ID["#aaa58b"]),("closure",TOP)):
        uv = projected(camera,*target[point])
        if uv is None:
            return False
        u,v = uv
        patch=ids[v-2:v+3,u-2:u+3]
        if np.count_nonzero(patch==material)<4:
            return False
        if np.ptp(decoded[max(0,v-5):v+6,max(0,u-5):u+6].mean(axis=2))<13:
            return False
    # A painted survey panel can occupy the centre sample. Check exposed asphalt
    # on both approaches to the actual opening in this same photograph.
    x,y,_=target["pavement"]
    for side in (-1,1):
        for delta in (.45,1.25,1.85):
            uv=projected(camera,x,y+side*delta)
            if uv:
                u,v=uv
                patch=ids[v-2:v+3,u-2:u+3]
                if np.count_nonzero((patch==ROAD)|(patch==ID["#a59b7e"]))>=4:
                    break
        else:
            return False
    return True


def write_photographs(directory,cameras,layout,paths,cones,signals,painted,barriers,fixtures,audit):
    """Accept the cosmetic realization only when the actual JPEGs retain multi-view evidence."""
    for attempt in range(5):
        style = RoadAppearance(layout,fixtures,attempt)
        visible = {t["name"]:[] for t in layout["visibility_targets"]}
        edge_samples = {(bank,f,d):[] for bank in ("left","right")
                        for f in (.25,.4,.6,.75,*layout.get("pinch_stations",{}).get(bank,[]))
                        for d in (-2.5,0,2.5)} if "survey_alignment" in layout else {}
        by_gate = {t["name"]:[] for t in layout["visibility_targets"]
                   if t.get("check_direction",t["name"]=="temporary")}
        bay_views = []
        channels = {bank:[] for bank in layout.get("channel_targets",{})}
        signs = {s["gate"]:[] for s in layout.get("access_signs",[])}
        package = []
        if audit is not None:
            colors = {k:np.zeros((3,32768),dtype="int64") for k in ("forward","oblique")}
            colors.update(exact=np.zeros(3,dtype="int64"),total=np.zeros(3,dtype="int64"))
            blocked = []
        for clip,index,camera in cameras:
            name = f"{clip}_{index:02}.jpg"
            encoded,ids,alpha = style.render(camera,layout,paths,cones,signals,painted,barriers,fixtures)
            with Image.open(BytesIO(encoded)) as jpeg:
                decoded = np.asarray(jpeg.convert("RGB"))
            findings = assess_frame(camera,ids,alpha,decoded,layout)
            for bank,target in layout.get("channel_targets",{}).items():
                if visible_channel(camera,target,ids,decoded):
                    channels[bank].append(name)
            for sign in layout.get("access_signs",[]):
                if legible_sign(camera,sign,ids,decoded):
                    signs[sign["gate"]].append(name)
            if "bay_target" in layout:
                x,y = layout["bay_target"]
                uv = projected(camera,x,y,.68)
                if uv:
                    u,v = uv
                    box = np.s_[v-2:v+3,u-2:u+3]
                    if np.count_nonzero(ids[box]==TOP)>=4 and np.ptp(decoded[max(v-6,0):v+7,max(u-6,0):u+7].mean(axis=2))>=14:
                        bay_views.append(name)
            if edge_samples:
                for key in visible_edge_points(camera,layout,ids,alpha,decoded):
                    edge_samples[key].append(name)
            if audit is not None:
                audit_colors(decoded,ids,clip,colors)
                for target in layout["visibility_targets"]:
                    features = [(target["center"],target["station"],0),
                                *((x,y,0) for x,y,_ in target["arrows"])]
                    for x,y,z in features:
                        uv = projected(camera,x,y,z)
                        if uv and ids[uv[1],uv[0]] in PROP:
                            blocked.append((name,target["name"]))
                            break
            for key,(gap,direction) in findings.items():
                if gap:
                    visible[key].append(name)
                if direction:
                    by_gate[key].append(name)
            package.append((name,encoded))
        edges = {(bank,f):max((edge_samples[bank,f,d] for d in (-2.5,0,2.5)),key=len)
                 for bank,f,d in edge_samples if d==0}
        if (min(map(len,visible.values())) < 2 or any(len(v)<2 for v in by_gate.values())
                or any(len(views)<2 for views in edges.values()) or ("bay_target" in layout and len(bay_views)<2)
                or any(len(v)<2 for v in signs.values()) or any(len(v)<2 for v in channels.values())):
            continue
        for name,encoded in package:
            (directory/name).write_bytes(encoded)
        if audit is not None:
            audit.update(attempt=attempt,visible=visible,directions=sorted({n for names in by_gate.values() for n in names}),
                         by_gate=by_gate,bay_views=bay_views,sign_views=signs,blocked=blocked,colors=colors)
            audit["edge_views"] = {f"{bank}_{f:g}":len(views) for (bank,f),views in edges.items()}
            audit["channel_views"] = channels
        return
    weak = {label:{str(k):len(v) for k,v in counts.items() if len(v)<2}
            for label,counts in (("openings",visible),("arrows",by_gate),("edges",edges),("signs",signs),
                                 ("channels",channels))}
    raise RuntimeError(f"Insufficient coverage at road length {layout['length']:.4f}: {weak}, bay_views={len(bay_views)}")
