"""Original Purdue time_layer annotations: 1-based frames and y1,x1,y2,x2 tuples."""
import hashlib
import re
import zipfile
from pathlib import Path

RAW_ANNOTATION_SHA256="abc9110cd287d76d2c6ed70f1163386d881bd567e39930431d393a2b383271b1"

def parse_time_layers(text):
    result={}
    for line in text.splitlines():
        if not line.strip():continue
        match=re.fullmatch(r"time_layer:\s*(\d+)\s+detections:\s*(.*)",line.strip())
        if not match:raise ValueError("unrecognized original annotation format")
        frame=int(match.group(1))
        if frame<1 or frame in result:raise ValueError("duplicate/invalid frame")
        pattern=r"\((-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\)"
        payload=match.group(2)
        if re.sub(pattern,"",payload).strip(" ,\t"):raise ValueError("malformed box tuple")
        result[frame]=[(int(x1),int(y1),int(x2),int(y2))
                       for y1,x1,y2,x2 in re.findall(pattern,payload)]
    return result

def load_annotations(path,expected_sha256=RAW_ANNOTATION_SHA256):
    path=Path(path)
    if hashlib.sha256(path.read_bytes()).hexdigest()!=expected_sha256:
        raise ValueError("original annotation archive SHA256 mismatch")
    result={}
    with zipfile.ZipFile(path) as z:
        for name in z.namelist():
            if "__MACOSX" in name:continue
            m=re.fullmatch(r"Clip_0*(\d+)_gt\.txt",Path(name).name)
            if m:
                clip=int(m.group(1))
                if clip in result:raise ValueError("duplicate clip")
                result[clip]=parse_time_layers(z.read(name).decode("utf-8-sig"))
    return result

def coco_boxes(boxes,original_size,output_size):
    ow,oh=original_size;w,h=output_size
    result=[]
    for x1,y1,x2,y2 in boxes:
        x1=max(0,min(ow,x1));x2=max(0,min(ow,x2))
        y1=max(0,min(oh,y1));y2=max(0,min(oh,y2))
        if x2<=x1 or y2<=y1:continue
        x1*=w/ow;x2*=w/ow;y1*=h/oh;y2*=h/oh
        result.append([x1,y1,x2-x1,y2-y1])
    return result
