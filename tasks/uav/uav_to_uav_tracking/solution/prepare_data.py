"""Rebuild original labels; existing formatted YOLO labels have known conversion bugs."""
import argparse
import json
import re
from pathlib import Path
from PIL import Image
from raw_annotations import load_annotations, coco_boxes, RAW_ANNOTATION_SHA256

def convert(source, split, output, annotations, stride=1, limit=None,
            expected_sha256=RAW_ANNOTATION_SHA256):
    if stride < 1:
        raise ValueError("stride must be positive")
    source, output = Path(source), Path(output)
    truth = load_annotations(annotations, expected_sha256)
    output.mkdir(parents=True, exist_ok=True)
    (output/"images").mkdir(exist_ok=True)
    expected = {"train":range(1,37), "val":range(37,41), "test":range(41,51)}[split]
    data = {"info":{"source":"Original Purdue time_layer yxyx annotations, regenerated",
                    "annotation_sha256":expected_sha256,
                    "source_split":split, "frame_stride":stride},
            "images":[], "annotations":[], "categories":[{"id":1,"name":"UAV"}]}
    frames = []
    for path in (source/"AllFrames"/split).glob("*.png"):
        m = re.fullmatch(r"Clip_(\d+)_(\d+)\.png", path.name)
        if not m:
            raise ValueError(f"unrecognized frame: {path}")
        clip, index = map(int, m.groups())
        if clip not in expected or index < 1:
            raise ValueError(f"split/index violation: {path}")
        if (index-1) % stride == 0:
            frames.append((clip,index,path))
    for clip,index,path in sorted(frames)[:limit]:
        # Original PNG indices and raw time_layer keys are both 1-based.
        boxes = truth[clip][index]
        with Image.open(path) as im:
            w,h = im.size
        image_id = len(data["images"])+1
        # The export step must materialize links before archiving for distribution.
        dest = output/"images"/path.name
        if not dest.exists():
            dest.symlink_to(path.resolve())
        data["images"].append({"id":image_id,"file_name":f"images/{path.name}",
                               "width":w,"height":h,"sequence_id":f"clip_{clip:02d}","frame_index":index})
        for box in coco_boxes(boxes, (w,h), (w,h)):
            data["annotations"].append({"id":len(data["annotations"])+1,"image_id":image_id,
                "category_id":1,"bbox":box,"area":box[2]*box[3],"iscrowd":0})
    if not data["images"]:
        raise ValueError("no frames selected")
    (output/"annotations.json").write_text(json.dumps(data,separators=(",",":")))
    return {"images":len(data["images"]), "annotations":len(data["annotations"])}

if __name__ == "__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--source",required=True); p.add_argument("--split",choices=["train","val","test"],required=True)
    p.add_argument("--annotations",required=True)
    p.add_argument("--output",required=True); p.add_argument("--stride",type=int,default=1)
    p.add_argument("--limit",type=int)
    a=p.parse_args()
    print(json.dumps(convert(a.source,a.split,a.output,a.annotations,a.stride,a.limit)))
