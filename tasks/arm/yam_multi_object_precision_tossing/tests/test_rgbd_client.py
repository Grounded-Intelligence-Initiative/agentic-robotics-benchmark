"""Sensor-only public kit tests. No physical rollout or scoring is performed."""
import ast
import base64
import importlib.util
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zlib

import numpy as np
from PIL import Image

TASK = Path(__file__).resolve().parents[1]
KIT = TASK/'environment/public_kit'
sys.path.insert(0,str(KIT))


def load(name):
    path = KIT/(name+'.py')
    assert path.exists(), 'Missing sensor-only public helper: '+name
    spec = importlib.util.spec_from_file_location(name,path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def camera_sample():
    rgb=np.array([[[10,20,30],[40,50,60]],[[70,80,90],[100,110,120]]],dtype=np.uint8)
    depth=np.array([[1.,2.],[3.,np.nan]],dtype='<f4')
    buffer=io.BytesIO();Image.fromarray(rgb).save(buffer,format='PNG')
    camera=dict(width=2,height=2,K=[2,0,.5,0,2,.5,0,0,1],T_world_camera=[1,0,0,1,0,1,0,2,0,0,1,3,0,0,0,1],rgb_png_base64=base64.b64encode(buffer.getvalue()).decode(),depth_f32_zlib_base64=base64.b64encode(zlib.compress(depth.tobytes())).decode(),depth_units='m',depth_convention='camera_z',depth_dtype='<f4')
    return camera,rgb,depth


class SensorKitTests(unittest.TestCase):
    def test_decode_roundtrip_and_camera_projection(self):
        client=load('rgbd_client');camera,rgb,depth=camera_sample()
        actual_rgb,actual_depth=client.decode_camera(camera)
        np.testing.assert_array_equal(actual_rgb,rgb)
        np.testing.assert_array_equal(actual_depth,depth)
        point=client.unproject(camera,np.array([.5,.5]),2.)
        np.testing.assert_allclose(point,[1,2,5],atol=1e-12)
        pixels,z=client.project(camera,point)
        np.testing.assert_allclose(pixels,[.5,.5],atol=1e-12);self.assertEqual(float(z),2.)
        rotated=dict(camera,T_world_camera=[0,-1,0,1,1,0,0,2,0,0,1,3,0,0,0,1])
        points=client.unproject(rotated,[[1.,.5],[.5,1.]],[2.,4.])
        np.testing.assert_allclose(points,[[1,2.5,5],[0,2,7]],atol=1e-12)
        pixels,z=client.project(rotated,points)
        np.testing.assert_allclose(pixels,[[1,.5],[.5,1]],atol=1e-12)
        np.testing.assert_allclose(z,[2,4],atol=1e-12)
        with self.assertRaises(ValueError):client.unproject(camera,[0,0],0.)
        broken=dict(camera,depth_f32_zlib_base64=base64.b64encode(zlib.compress(b'123')).decode())
        with self.assertRaises(ValueError):client.decode_camera(broken)

    def test_decoder_cli_outputs_images_and_lossless_depth(self):
        client=load('rgbd_client');camera,_,depth=camera_sample()
        with tempfile.TemporaryDirectory() as directory:
            sample=Path(directory)/'sample.json';sample.write_text(json.dumps(dict(time=0,arms={},cameras={'view':camera})))
            output=Path(directory)/'decoded'
            subprocess.run([sys.executable,client.__file__,str(sample),'--output',str(output)],check=True,capture_output=True)
            self.assertTrue((output/'view_rgb.png').exists());self.assertTrue((output/'view_depth_preview.png').exists())
            np.testing.assert_array_equal(np.load(output/'view_depth.npy'),depth)

    def test_fk_matches_own_robot_tree_without_physics(self):
        client=load('rgbd_client')
        import mujoco as mj
        tree=ET.fromstring((TASK/'environment/yam/yam.xml').read_text().split('-->')[-1])
        for section in ('asset','contact','equality'):
            node=tree.find(section)
            if node is not None:tree.remove(node)
        for body in tree.findall('.//body'):
            for geom in list(body.findall('geom')):body.remove(geom)
        model=mj.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'));data=mj.MjData(model)
        robot=json.loads((KIT/'robot_kinematics.json').read_text())
        for q in (np.zeros(6),np.array([.3,1.4,1.2,-.5,.2,-.4]),np.array([-.8,2.,.8,.4,-.3,.5])):
            data.qpos[:6]=q;mj.mj_forward(model,data)
            for side in ('left','right'):
                actual=client.forward_kinematics(q,side,robot)
                base=client.pose_matrix(robot['bases'][side])
                expected=np.eye(4);expected[:3,:3]=data.site('grasp_site').xmat.reshape(3,3);expected[:3,3]=data.site('grasp_site').xpos
                np.testing.assert_allclose(actual,base@expected,atol=2e-12,rtol=0)
        with self.assertRaises(ValueError):client.forward_kinematics([0]*5,'left',robot)
        allowed={'schema','units','joint_order','links','bases','tcp','jaw_limits'}
        self.assertEqual(set(robot),allowed)
        text=(KIT/'robot_kinematics.json').read_text().lower()
        for forbidden in ('mass','inertia','mesh','friction','receiver','payload','seed','collision'):
            self.assertNotIn(forbidden,text)

    def test_smoke_uses_only_recorded_observation(self):
        smoke=load('sensor_smoke');camera,_,_=camera_sample()
        arm=dict(joints=[0,1.5,1.45,0,0,0],joint_velocity=[0]*6,finger_positions=[.025]*2,finger_velocity=[0]*2,reference=[0,1.5,1.45,0,0,0],jaw_reference=.025)
        sample=dict(time=0,arms={'left':arm,'right':arm},cameras={'view':camera})
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);path=folder/'sensor.json';path.write_text(json.dumps(sample))
            controller=folder/'run.sh';controller.write_text('#!/bin/sh\nexec '+sys.executable+' '+str(folder/'policy.py')+'\n')
            (folder/'policy.py').write_text('import json,sys\nprint("READY",flush=True)\nfor line in sys.stdin:\n m=json.loads(line)\n print(json.dumps({"action":{"finish":True}} if m["type"]=="act" else {"ok":True}),flush=True)\n if m["type"]=="close":break\n')
            result=smoke.run_smoke(controller,path)
            self.assertEqual(result['protocol'],'pass');self.assertIsNone(result['physical_success'])
            (folder/'policy.py').write_text('import sys\nprint("READY",flush=True)\nfor line in sys.stdin:print("bad-json",flush=True)\n')
            with self.assertRaises(RuntimeError):smoke.run_smoke(controller,path)
        self.assertFalse((TASK/'environment/setup/payload/practice_grader').exists())
        for source in KIT.glob('*.py'):
            text=source.read_text()
            for forbidden in ('import mujoco','from workcell','from arena','MjModel','mj_step'):
                self.assertNotIn(forbidden,text)

    def test_smoke_rejects_invalid_waypoints_without_world_state(self):
        smoke=load('sensor_smoke')
        action=dict(arm='left',item='left_item0',waypoints=[[0,0,1.5,1.45,0,0,0,.025],[6,0,1.5,1.45,0,0,0,.025]])
        smoke.validate_action(action)
        action['waypoints'][1][1]=99
        with self.assertRaises(RuntimeError):smoke.validate_action(action)
        action['waypoints'][1][1]=0;action['waypoints'][1][0]=7
        with self.assertRaises(RuntimeError):smoke.validate_action(action)

    def test_left_azimuth_and_right_order_are_seed_invariant(self):
        tree=ast.parse((TASK/'environment/station.py').read_text());constants={}
        for node in tree.body:
            if isinstance(node,ast.Assign):
                for target in node.targets:
                    if isinstance(target,ast.Name) and target.id in ('PICKS','SHAPES'):
                        constants[target.id]=ast.literal_eval(node.value)
        radius=max(max(shape[1]) for shape in constants['SHAPES'].values())*1.17
        shift=(radius+.0002)*math.sin(math.radians(22))
        bounds=[];ybounds=[]
        for angle,r in constants['PICKS']:
            points=[(max(.24,rr*math.cos(a)),rr*math.sin(a)) for rr in (r-.022,r+.022) for a in (angle-.055,angle+.055)]
            extra=math.asin(shift/(r-.022));angles=[math.atan2(y,x) for x,y in points]
            bounds.append((min(angles)-extra,max(angles)+extra));ybounds.append((min(y for x,y in points)-shift,max(y for x,y in points)+shift))
        self.assertTrue(all(a[1]<b[0] for a,b in zip(bounds,bounds[1:])))
        self.assertGreater(ybounds[0][1],ybounds[1][0], 'Document azimuth, not non-invariant y-order')
        arena=ast.parse((TASK/'environment/arena.py').read_text())
        layout=next(ast.literal_eval(node.value) for node in arena.body if isinstance(node,ast.Assign) and any(isinstance(target,ast.Name) and target.id=='PRECISION_LAYOUT' for target in node.targets))
        ordered=sorted(layout,key=lambda row:row[3][1])
        self.assertEqual([(row[0],row[1]) for row in ordered],[('slot','large'),('slot','medium'),('slot','small'),('ring','large'),('ring','medium'),('ring','small'),('wood','medium')])
        right_shift=.009+(.018+.0002)*math.sin(math.radians(22))
        self.assertTrue(all(a[3][1]+right_shift<b[3][1]-right_shift for a,b in zip(ordered,ordered[1:])))


if __name__=='__main__':unittest.main()
