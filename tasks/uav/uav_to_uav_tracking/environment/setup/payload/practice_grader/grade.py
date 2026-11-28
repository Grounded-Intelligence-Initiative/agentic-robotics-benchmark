import argparse
import json
from pathlib import Path
from uav_eval import infer, metrics, SubmissionError

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--submission",required=True)
    p.add_argument("--data",required=True)
    p.add_argument("--device",default="cuda",choices=["cpu","cuda"])
    a=p.parse_args()
    try:
        dataset,predictions=infer(a.submission,a.data,Path(__file__).with_name("worker.py"),device=a.device)
        print(json.dumps(metrics(dataset,predictions),indent=2))
    except SubmissionError as exc:
        raise SystemExit(f"Invalid submission: {exc}")

if __name__=="__main__":
    main()
