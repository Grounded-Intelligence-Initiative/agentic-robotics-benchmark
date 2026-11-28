"""Author checks: metric math, indexing, process failures and canonical copies."""
import importlib.util
import hashlib
import zipfile
import json
import os
import sys
import tempfile
import time
import subprocess
import shutil
import tarfile
import io
import unittest
from pathlib import Path
import torch
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path(__file__).parent))
from uav_eval import metrics,validate_boxes,SubmissionError,infer
from robotics_grader import ratio_rewards

TASK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TASK/"solution"))

def dataset():
    return {"info":{}, "categories":[{"id":1,"name":"UAV"}],
            "images":[{"id":1,"width":32,"height":32,"file_name":"a.png","sequence_id":"a","frame_index":1},
                      {"id":2,"width":32,"height":32,"file_name":"b.png","sequence_id":"b","frame_index":1}],
            "annotations":[{"id":1,"image_id":1,"category_id":1,"bbox":[4,4,8,8],"area":64,"iscrowd":0}]}

def detection(image=1, confidence=.9):
    return dict(image_id=image,category_id=1,bbox=[4,4,8,8],score=confidence)

class MetricTests(unittest.TestCase):
    def test_reward_keeps_gt_scale(self):
        anchor=json.loads((TASK/"tests/anchor.json").read_text())

        for predictions, expected in (([],0.0),([detection()],1.0),
                ([detection(2,1),detection(1,.9)],0.5)):
            measured=metrics(dataset(),predictions)["uav_ap50"]
            verdict=ratio_rewards(measured,anchor["value"],full_at=anchor["full_at"])
            self.assertAlmostEqual(verdict["rewards"]["reward"],expected)
        self.assertEqual(anchor["value"],1.0)
        self.assertEqual(anchor["full_at"],1.0)

    def test_perfect(self):
        result=metrics(dataset(),[detection()])
        self.assertAlmostEqual(result["uav_ap50"],1)
        self.assertAlmostEqual(result["uav_ap50_95"],1)

    def test_negative_frame_false_positive_counts(self):
        result=metrics(dataset(),[detection(2,1),detection(1,.9)])
        self.assertAlmostEqual(result["uav_ap50"],.5)

    def test_empty(self):
        self.assertEqual(metrics(dataset(),[])["uav_ap50"],0)

    def test_missed_target_counts(self):
        self.assertEqual(metrics(dataset(),[detection(2)])["uav_ap50"],0)

    def test_ap50_differs_from_ap5095(self):
        pred=detection()
        pred["bbox"]=[6,4,8,8]
        m=metrics(dataset(),[pred])
        self.assertAlmostEqual(m["uav_ap50"],1)
        self.assertLess(m["uav_ap50_95"],.5)

    def test_max_detections_really_300(self):
        preds=[dict(image_id=1,category_id=1,bbox=[20,20,1,1],score=1-i/1000) for i in range(110)]
        preds.append(detection(1,.5))
        self.assertGreater(metrics(dataset(),preds)["uav_ap50"],0)

    def test_bad_boxes(self):
        for boxes in ([[0,0,float("nan"),1,.5]],[[0,0,33,1,.5]],[[1,1,0,2,.5]],
                      [[0,0,1,1,1.1]],[[0,0,1,1]],[[0,0,1,1,.5]]*301,None):
            with self.subTest(boxes=str(boxes)[:60]), self.assertRaises(SubmissionError):
                validate_boxes(boxes,32,32)
        self.assertEqual(validate_boxes([],32,32).shape,(0,5))

class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.sub=self.root/"submission"; self.sub.mkdir()
        for name in ("model.pt","model.py","train.sh","config.json","training_log.jsonl"):
            (self.sub/name).write_text("")
        torch.save({"weight":torch.ones(1)}, self.sub/"model.pt")
        (self.sub/"config.json").write_text(json.dumps({"architecture":"test fixture"}))
        (self.sub/"training_log.jsonl").write_text(json.dumps({"step":1,"loss":1.0})+"\n")
        self.data=self.root/"data"; self.data.mkdir()
        d=dataset()
        (self.data/"annotations.json").write_text(json.dumps(d))
        for im in d["images"]:
            Image.new("RGB",(32,32)).save(self.data/im["file_name"])

    def tearDown(self):
        self.temp.cleanup()

    def run_model(self,source,timeout=15):
        (self.sub/"model.py").write_text(source)
        return infer(self.sub,self.data,TASK/"tests/worker.py",device="cpu",
                     startup_timeout=15,frame_timeout=timeout,total_timeout=max(30,timeout*3))

    def test_learned_detector_uses_normal_worker_without_answer_fixture(self):
        from model import build_model

        torch.manual_seed(7)
        torch.save(build_model().state_dict(), self.sub/"model.pt")
        Image.new("RGB", (32, 32), (37, 83, 149)).save(self.data/"b.png")
        shutil.copy2(TASK/"solution/model.py", self.sub/"model.py")
        data, preds = infer(self.sub, self.data, TASK/"tests/worker.py", device="cpu",
                            startup_timeout=30, frame_timeout=30, total_timeout=120)
        self.assertEqual(len(data["images"]), 2)
        self.assertTrue(all(p["image_id"] in (1, 2) for p in preds))
        self.assertFalse(any(p.name.startswith("gt_") for p in self.sub.iterdir()))

    def test_gt_oracle_uses_normal_worker(self):
        from prepare_gt_fixture import pixel_key
        from build_gt_submission import package
        from gt_model import Detector

        Image.new("RGB",(32,32),(255,0,0)).save(self.data/"b.png")
        predictions={}
        for name,boxes in (("a.png",[[4,4,12,12,1.0]]),("b.png",[])):
            with Image.open(self.data/name) as im:
                predictions[pixel_key(np.array(im.convert("RGB")))]=boxes
        fixture=self.root/"fixture.json"
        fixture.write_text(json.dumps({"schema":1,"role":"author_only_ground_truth_oracle",
            "frames":2,"annotations_sha256":"synthetic-unit-test",
            "predictions":predictions}))
        baseline=self.root/"baseline"
        shutil.copytree(self.sub,baseline)
        original_log=(baseline/"training_log.jsonl").read_bytes()
        original_hash=hashlib.sha256((baseline/"model.pt").read_bytes()).hexdigest()
        (baseline/"config.json").write_text(json.dumps({"checkpoint_sha256":original_hash}))
        package(baseline,self.sub,fixture)
        self.assertEqual((self.sub/"training_log.jsonl").read_bytes(),original_log)
        self.assertEqual(hashlib.sha256((baseline/"model.pt").read_bytes()).hexdigest(),original_hash)
        data,preds=infer(self.sub,self.data,TASK/"tests/worker.py",device="cpu")
        self.assertAlmostEqual(metrics(data,preds)["uav_ap50"],1.0)
        self.assertEqual(len(preds),1)
        model=Detector(str(self.sub/"model.pt"),"cpu")
        with self.assertRaisesRegex(ValueError,"outside"):
            model.predict(np.full((32,32,3),2,dtype=np.uint8))

    def test_fresh_state_each_sequence(self):
        d,p=self.run_model("class Detector:\n def __init__(self,*a): self.n=0\n def reset(self): self.n=0\n def predict(self,frame):\n  self.n+=1\n  return [[self.n,0,8,8,.9]]\n")
        self.assertEqual([r["bbox"][0] for r in p],[1,1])

    def test_broken_checkpoint_is_submission_failure(self):
        (self.sub/"model.pt").write_bytes(b"truncated checkpoint")
        with self.assertRaises(SubmissionError):
            self.run_model("class Detector:\n def __init__(self,*a): raise RuntimeError('truncated checkpoint')\n")

    def test_nan_is_submission_failure(self):
        with self.assertRaises(SubmissionError):
            self.run_model("class Detector:\n def __init__(self,*a): pass\n def reset(self): pass\n def predict(self,f): return [[0,0,float('nan'),1,.5]]\n")

    def test_prediction_timeout(self):
        start=time.monotonic()
        with self.assertRaises(SubmissionError):
            self.run_model("import time\nfrom pathlib import Path\nclass Detector:\n def __init__(self,*a): pass\n def reset(self): pass\n def predict(self,f):\n  Path('predict-entered').touch()\n  time.sleep(20)\n",timeout=.5)
        self.assertTrue((self.sub/"predict-entered").exists())
        self.assertLess(time.monotonic()-start,10)

    def test_no_read_causes_bounded_write(self):
        Image.new("RGB",(1024,1024)).save(self.data/"a.png")
        d=dataset();d["images"][0].update(width=1024,height=1024)
        (self.data/"annotations.json").write_text(json.dumps(d))
        worker=self.root/"blocked.py"
        worker.write_text("import time,json\nprint(json.dumps(dict(ready=True)),flush=True)\ntime.sleep(20)\n")
        start=time.monotonic()
        with self.assertRaises(SubmissionError):
            infer(self.sub,self.data,worker,device="cpu",startup_timeout=1,frame_timeout=.1)
        self.assertLess(time.monotonic()-start,3)

    def test_oversized_reply(self):
        worker=self.root/"flood.py"
        worker.write_text("print('x'*200000,flush=True)")
        with self.assertRaisesRegex(SubmissionError,"oversized"):
            infer(self.sub,self.data,worker,device="cpu",startup_timeout=1)

    def test_invalid_training_artifacts(self):
        from worker import validate_artifacts
        validate_artifacts(self.sub)
        cases = [
            ("config.json", "{}"),
            ("config.json", "[]"),
            ("training_log.jsonl", ""),
            ("training_log.jsonl", '{"step":1,"loss":NaN}\n'),
            ("training_log.jsonl", '{"step":1,"loss":1}\n{"step":1,"loss":0.5}\n'),
            ("training_log.jsonl", '{"step":true,"loss":1}\n'),
        ]
        for name, text in cases:
            path = self.sub/name; original = path.read_text()
            with self.subTest(name=name, text=text), self.assertRaises(ValueError):
                path.write_text(text); validate_artifacts(self.sub)
            path.write_text(original)
        torch.save({}, self.sub/"model.pt")
        with self.assertRaises(ValueError):
            validate_artifacts(self.sub)
        torch.save({"weight":"not a tensor"}, self.sub/"model.pt")
        with self.assertRaises(ValueError):
            validate_artifacts(self.sub)

    def test_missing_artifact(self):
        (self.sub/"model.pt").unlink()
        with self.assertRaises(SubmissionError):
            self.run_model("")

class DataTests(unittest.TestCase):
    def test_original_yxyx_and_one_based_time_layer(self):
        spec=importlib.util.spec_from_file_location("prepare_data",TASK/"solution/prepare_data.py")
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); imgs=root/"AllFrames/train"
            imgs.mkdir(parents=True)
            Image.new("RGB",(100,100)).save(imgs/"Clip_1_00001.png")
            archive=root/"annotations.zip"
            with zipfile.ZipFile(archive,"w") as z:
                z.writestr("Video_Annotation/Clip_1_gt.txt","time_layer: 1 detections: (10, 60, 20, 90),\ntime_layer: 2 detections: \n")
            digest=hashlib.sha256(archive.read_bytes()).hexdigest()
            module.convert(root,"train",root/"export",archive,expected_sha256=digest)
            d=json.loads((root/"export/annotations.json").read_text())
            self.assertEqual(d["annotations"][0]["bbox"],[60,10,30,10])
            Image.new("RGB",(100,100)).save(imgs/"Clip_1_00003.png")
            with self.assertRaises(KeyError):
                module.convert(root,"train",root/"export2",archive,expected_sha256=digest)
            self.assertEqual(module.coco_boxes([(60,10,90,20)],(100,100),(50,50)),[[30,5,15,5]])

    def test_canonical_copies(self):
        for name in ("worker.py","uav_eval.py"):
            self.assertEqual((TASK/"tests"/name).read_bytes(),
                (TASK/"environment/setup/payload/practice_grader"/name).read_bytes())
        repo=TASK.parents[2]
        for path in (repo/"shared/robotics_grader").glob("*.py"):
            self.assertEqual(path.read_bytes(),(TASK/"tests/robotics_grader"/path.name).read_bytes())
        for name in ("prepare_release.py", "raw_annotations.py"):
            self.assertEqual((TASK/"solution"/name).read_bytes(), (TASK/"tests"/name).read_bytes())

    def test_public_archive_rejects_private_paths_and_links(self):
        for name, kind in (("test/annotations.json", "file"), ("../escape", "file"),
                           ("/tmp/escape", "file"), ("train/images/link", "link")):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temp:
                root=Path(temp)
                with tarfile.open(root/"shard.tar", "w") as archive:
                    member=tarfile.TarInfo(name)
                    if kind=="link":
                        member.type=tarfile.SYMTYPE; member.linkname="/etc/passwd"
                        archive.addfile(member)
                    else:
                        member.size=2; archive.addfile(member, io.BytesIO(b"{}"))
                result=subprocess.run([sys.executable, str(TASK/"environment/unpack_public.py"),
                    str(root/"shard.tar"), str(root/"output")],capture_output=True)
                self.assertNotEqual(result.returncode,0)
                self.assertFalse((root/"escape").exists())

if __name__=="__main__":
    unittest.main(verbosity=2)
