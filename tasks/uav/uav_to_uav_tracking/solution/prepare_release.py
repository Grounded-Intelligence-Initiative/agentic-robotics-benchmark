"""Author-side export from verified original videos, with exact frame/label checks."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

import av
from raw_annotations import load_annotations, coco_boxes, RAW_ANNOTATION_SHA256

SPLITS = {'train': range(1, 37), 'val': range(37, 41), 'test': range(41, 51)}

def export_clip(args):
    clip, video, labels, output, stride = args
    output = Path(output)
    images, annotations, hashes = [], [], []
    count = 0
    with av.open(video) as container:
        stream = container.streams.video[0]
        stream.thread_type = 'SLICE'
        stream.codec_context.thread_count = 1
        for count, frame in enumerate(container.decode(stream), 1):
            if count not in labels:
                raise ValueError(f'Clip {clip}: frame {count} has no original annotation')
            if (count - 1) % stride:
                continue
            image = frame.to_image().convert('RGB')
            width, height = image.size
            relative = f'images/Clip_{clip}_{count:05d}.jpg'
            dest = output / relative
            image.save(dest, quality=95, subsampling=0, optimize=False)
            image_id = clip * 100000 + count
            images.append({'id': image_id, 'file_name': relative, 'width': width,
                'height': height, 'sequence_id': f'clip_{clip:02d}', 'frame_index': count})
            boxes = coco_boxes(labels[count], (width, height), (width, height))
            for box in boxes:
                annotations.append({'image_id': image_id, 'category_id': 1, 'bbox': box,
                                    'area': box[2] * box[3], 'iscrowd': 0})
            hashes.append({'file': relative, 'sha256': hashlib.sha256(dest.read_bytes()).hexdigest(),
                           'bytes': dest.stat().st_size, 'positive': bool(boxes)})
    if set(labels) != set(range(1, count + 1)):
        raise ValueError(f'Clip {clip}: decoded {count} frames vs {len(labels)} annotations')
    print(json.dumps({'clip': clip, 'decoded_frames': count, 'exported_frames': len(images),
                      'boxes': len(annotations)}), flush=True)
    return images, annotations, hashes, count

def export(source, annotations, output, splits, workers=4):
    source, output = Path(source), Path(output)
    truth = load_annotations(annotations)
    result = {}
    all_hashes = {}
    for split in splits:
        root = output / split
        root.mkdir(parents=True, exist_ok=True)
        (root / 'images').mkdir(exist_ok=True)
        if (root / 'annotations.json').exists():
            raise FileExistsError(f'Refusing to overwrite completed export: {root}')
        stride = 1 if split == 'train' else 5
        jobs = [(clip, str(source / f'Clip_{clip}.mov'), truth[clip], str(root), stride)
                for clip in SPLITS[split]]
        dataset = {'info': {'version': 'nps-time-layer-v1-jpeg95',
            'annotation_sha256': RAW_ANNOTATION_SHA256, 'source_split': split,
            'frame_stride': stride, 'image_encoding': 'Pillow 11.3.0 JPEG quality=95 subsampling=0',
            'decoder': 'PyAV 15.1.0'}, 'images': [], 'annotations': [],
            'categories': [{'id': 1, 'name': 'UAV'}]}
        hashes, decoded = [], 0
        with ProcessPoolExecutor(max_workers=workers) as pool:
            for images, boxes, checksums, count in pool.map(export_clip, jobs):
                dataset['images'].extend(images)
                dataset['annotations'].extend(boxes)
                hashes.extend(checksums)
                decoded += count
        for index, box in enumerate(dataset['annotations'], 1):
            box['id'] = index
        (root / 'annotations.json').write_text(json.dumps(dataset, separators=(',', ':')))
        (root / 'images.sha256.json').write_text(json.dumps(hashes, separators=(',', ':')))
        positive = {box['image_id'] for box in dataset['annotations']}
        result[split] = {'decoded_frames': decoded, 'images': len(dataset['images']),
            'boxes': len(dataset['annotations']), 'empty_images': len(dataset['images']) - len(positive),
            'image_bytes': sum(h['bytes'] for h in hashes),
            'annotation_sha256': hashlib.sha256((root / 'annotations.json').read_bytes()).hexdigest()}
        all_hashes[split] = {h['sha256'] for h in hashes}
    overlap = {}
    for index, left in enumerate(splits):
        for right in splits[index + 1:]:
            overlap[f'{left}:{right}'] = len(all_hashes[left] & all_hashes[right])
    report = {'splits': result, 'exact_image_hash_overlap': overlap,
              'near_duplicate_audit': 'Not established by exact hashes; review source sequences separately.'}
    (output / 'export_report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    return report

def archive_public(root, archive):
    root, archive = Path(root), Path(archive)
    # Explicit allowlist; test inputs, annotations and hash lists never enter this archive.
    allowed = ['train', 'val', 'DATA_CARD.md', 'LICENSE.txt']
    with tarfile.open(archive, 'w') as out:
        for name in allowed:
            path = root / name
            if not path.exists():
                raise FileNotFoundError(path)
            out.add(path, arcname=name, recursive=True)

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--videos', required=True)
    p.add_argument('--annotations', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--splits', nargs='+', choices=list(SPLITS), default=list(SPLITS))
    p.add_argument('--workers', type=int, default=4)
    a = p.parse_args()
    export(a.videos, a.annotations, a.output, a.splits, a.workers)
