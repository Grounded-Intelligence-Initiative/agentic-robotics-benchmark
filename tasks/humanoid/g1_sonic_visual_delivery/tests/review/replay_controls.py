"""Replay a measured native action tape under both physical signals with an optional camera-responsive final perturbation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobile_score import AuditedSession
from sonic_sim import DeliverySession


def run(args, codes, hands, side, camera_edit):
    with DeliverySession("visual_delivery", args.seed, args.assets_root, start_mode=args.start,
                         shape=args.shape, target_side=side) as session:
        audit = AuditedSession(session)
        changed_row = None
        for i, (code, hand) in enumerate(zip(codes, hands)):
            if i == len(codes) - 2 and camera_edit:
                img = session.camera("overview", 320, 240)
                # React to live colour at the end of the taped movement, without redirecting it.
                code = code.copy()
                code[0] = hashlib.sha256(img.tobytes()).digest()[0] % 32
                changed_row = code.tolist()
            audit.step((code.astype(np.float32) - 16.) / 16., *hand)
            if session.state["done"]:
                break
        row = audit.report()
        row.update(target=side, reason=session.state["reason"], success=session.state["metrics"]["success"],
                   wrong_pad=session.state["metrics"]["wrong_pad"], changed_row=changed_row)
        return row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tape", type=Path, required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--start", choices=["near", "far"], required=True)
    ap.add_argument("--shape", choices=["box", "cylinder"], required=True)
    ap.add_argument("--assets-root", type=Path, default=Path("/opt/sonic/assets"))
    args = ap.parse_args()
    with np.load(args.tape, allow_pickle=False) as tape:
        codes, hands = tape["codes"], tape["hands"]
    if codes.ndim != 2 or codes.shape[1] != 64 or hands.shape != (len(codes), 2):
        raise ValueError("expected aligned N×64 codes and N×2 hands")
    if codes.dtype != np.uint8 or np.max(codes) > 31 or not np.all((hands >= 0) & (hands <= 1)):
        raise ValueError("tape contains an invalid action")
    report = {"tape_sha256": hashlib.sha256(args.tape.read_bytes()).hexdigest(), "seed": args.seed,
              "shape": args.shape, "start": args.start, "rows": len(codes)}
    for mode, edit in (("fixed", False), ("camera_edit", True)):
        episodes = [run(args, codes, hands, side, edit) for side in ("left", "right")]
        report[mode] = dict(episodes=episodes, paired_score=int(all(x["success"] and x["placed"] for x in episodes)))
    print(json.dumps(report, allow_nan=False))


if __name__ == "__main__":
    main()
