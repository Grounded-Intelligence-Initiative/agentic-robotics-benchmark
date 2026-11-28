"""Shared visible metric/transport semantics; no private data or normalization."""
import contextlib
import io
import json
import math
import os
import select
import signal
import struct
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
import numpy as np
from PIL import Image

class SubmissionError(Exception):
    pass

def validate_boxes(value, width, height):
    try:
        boxes = np.asarray(value,dtype=np.float64)
    except (TypeError,ValueError,OverflowError) as exc:
        raise SubmissionError("boxes must be numeric") from exc
    if boxes.shape == (0,):
        boxes = boxes.reshape(0,5)
    if boxes.ndim != 2 or boxes.shape[1] != 5 or len(boxes)>300:
        raise SubmissionError("expected at most 300 Nx5 boxes")
    if not np.isfinite(boxes).all():
        raise SubmissionError("non-finite detection")
    if len(boxes):
        x1,y1,x2,y2,s = boxes.T
        if not (np.all(x1>=0) and np.all(y1>=0) and np.all(x2<=width) and np.all(y2<=height)
                and np.all(x2>x1) and np.all(y2>y1) and np.all((s>=0)&(s<=1))):
            raise SubmissionError("invalid box bounds or confidence")
    return boxes

class Channel:
    """Bounded parent-side I/O, including blocked writes and missing newlines."""
    def __init__(self, process):
        self.process=process
        self.pending=bytearray()
        os.set_blocking(process.stdin.fileno(),False)
        os.set_blocking(process.stdout.fileno(),False)

    def send(self, data, deadline):
        data=memoryview(data)
        while data:
            timeout=deadline-time.monotonic()
            if timeout<=0:
                raise SubmissionError("input deadline exceeded")
            _,ready,_=select.select([], [self.process.stdin], [], timeout)
            if not ready:
                raise SubmissionError("input deadline exceeded")
            try:
                count=os.write(self.process.stdin.fileno(), data[:65536])
            except (BrokenPipeError,OSError) as exc:
                raise SubmissionError("model exited while receiving frame") from exc
            data=data[count:]

    def receive(self, deadline, max_bytes=100000):
        while b"\n" not in self.pending:
            timeout=deadline-time.monotonic()
            if timeout<=0:
                raise SubmissionError("prediction deadline exceeded")
            ready,_,_=select.select([self.process.stdout], [], [], timeout)
            if not ready:
                raise SubmissionError("prediction deadline exceeded")
            block=os.read(self.process.stdout.fileno(),min(65536,max_bytes+1-len(self.pending)))
            if not block:
                raise SubmissionError(f"model closed protocol (exit={self.process.poll()})")
            self.pending.extend(block)
            if len(self.pending)>max_bytes:
                raise SubmissionError("oversized model reply")
        line,_,rest=self.pending.partition(b"\n")
        self.pending=bytearray(rest)
        try:
            result=json.loads(line)
            if not isinstance(result,dict):
                raise ValueError("reply must be an object")
            return result
        except (ValueError,UnicodeError) as exc:
            raise SubmissionError("malformed model reply") from exc

def stop(process):
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=10)
    process.stdin.close(); process.stdout.close()

def infer(submission, data_root, worker, *, spawn=subprocess.Popen, device="cuda",
          startup_timeout=120, frame_timeout=10, total_timeout=6000):
    submission=Path(submission).resolve()
    required=("model.py","model.pt","train.sh","config.json","training_log.jsonl")
    if any(not (submission/name).is_file() for name in required):
        raise SubmissionError("missing submission artifacts")
    data_root=Path(data_root)
    dataset=json.loads((data_root/"annotations.json").read_text())
    groups=defaultdict(list)
    for record in dataset["images"]:
        groups[record["sequence_id"]].append(record)
    if not groups:
        raise ValueError("grader dataset is empty")
    deadline=time.monotonic()+total_timeout
    predictions=[]
    for sequence in sorted(groups):
        # Paths/sequence IDs/ground truth never cross into the child.
        argv=[sys.executable,"-I","-B",str(worker),str(submission),device]
        process=spawn(argv,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                      stderr=subprocess.DEVNULL,start_new_session=True,cwd=str(submission))
        channel=Channel(process)
        try:
            if channel.receive(min(deadline,time.monotonic()+startup_timeout)) != {"ready":True}:
                raise SubmissionError("missing model ready response")
            for record in sorted(groups[sequence],key=lambda r:r["frame_index"]):
                path=(data_root/record["file_name"]).resolve()
                with Image.open(path) as image:
                    frame=np.array(image.convert("RGB"))
                h,w=frame.shape[:2]
                if (w,h)!=(record["width"],record["height"]):
                    raise ValueError("grader image dimensions do not match manifest")
                header=json.dumps({"height":h,"width":w}).encode()
                frame_deadline=min(deadline,time.monotonic()+frame_timeout)
                channel.send(struct.pack(">I",len(header))+header+frame.tobytes(),frame_deadline)
                reply=channel.receive(frame_deadline)
                boxes=validate_boxes(reply.get("boxes"),w,h)
                for x1,y1,x2,y2,score in boxes:
                    # COCO's per-box confidence field, not a normalized task reward.
                    predictions.append(dict(image_id=record["id"], category_id=1,
                        bbox=[float(x1),float(y1),float(x2-x1),float(y2-y1)], score=float(score)))
        finally:
            stop(process)
    return dataset,predictions

def metrics(dataset, predictions):
    from pycocotools.coco import COCO
    from pycocotools.cocoeval import COCOeval
    if not dataset.get("annotations"):
        raise ValueError("evaluation requires positive ground truth")
    if not predictions:
        return {"uav_ap50":0.0,"uav_ap50_95":0.0,"uav_ap_small":0.0,"uav_ar300":0.0}
    # Standard COCO matching/101-point interpolation; explicitly use maxDets=300.
    with contextlib.redirect_stdout(io.StringIO()):
        gt=COCO(); gt.dataset=dataset; gt.createIndex()
        dt=gt.loadRes(predictions)
        evaluator=COCOeval(gt,dt,"bbox")
        evaluator.params.imgIds=sorted(im["id"] for im in dataset["images"])
        evaluator.params.catIds=[1]
        evaluator.params.maxDets=[1,10,300]
        evaluator.evaluate(); evaluator.accumulate()
    p=evaluator.eval["precision"]
    r=evaluator.eval["recall"]
    def mean(values):
        values=values[values>=0]
        return float(values.mean()) if len(values) else 0.0
    return {"uav_ap50":mean(p[0,:,0,0,-1]),
            "uav_ap50_95":mean(p[:,:,0,0,-1]),
            "uav_ap_small":mean(p[:,:,0,1,-1]),"uav_ar300":mean(r[:,0,0,-1])}
