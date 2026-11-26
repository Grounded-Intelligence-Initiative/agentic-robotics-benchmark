"""Mirror selected prefixes of NVIDIA's public Isaac asset bucket (the same source the merged Isaac task uses)."""
import os, sys, urllib.request, urllib.parse, xml.etree.ElementTree as ET
BUCKET = "https://omniverse-content-production.s3-us-west-2.amazonaws.com"; ROOT = "Assets/Isaac/5.1/"
ns = {"s3": "http://s3.amazonaws.com/doc/2006-03-01/"}
out, total, files = sys.argv[1], 0, 0
for pre in sys.argv[2:]:
    token = None
    while True:
        q = {"list-type": "2", "prefix": ROOT + pre, "max-keys": "1000"}
        if token: q["continuation-token"] = token
        root = ET.fromstring(urllib.request.urlopen(BUCKET + "/?" + urllib.parse.urlencode(q), timeout=60).read())
        for c in root.findall("s3:Contents", ns):
            key, size = c.find("s3:Key", ns).text, int(c.find("s3:Size", ns).text)
            if key.endswith("/"): continue
            dst = os.path.join(out, key[len(ROOT):])
            if os.path.exists(dst) and os.path.getsize(dst) == size: continue
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            for attempt in range(3):
                try: urllib.request.urlretrieve(BUCKET + "/" + urllib.parse.quote(key), dst); break
                except Exception:
                    if attempt == 2: raise
            total += size; files += 1
        t = root.find("s3:NextContinuationToken", ns); token = t.text if t is not None else None
        if token is None: break
print("MIRROR_DONE", files, "files", round(total / 1e6, 1), "MB")
