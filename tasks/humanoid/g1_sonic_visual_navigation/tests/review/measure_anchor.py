"""Recover the registered full-credit count from the first eligible null-anchor oracle run."""
import argparse
import hashlib
import json
from pathlib import Path


def measure(path):
    raw = path.read_bytes()
    verdict = json.loads(raw)
    metrics = verdict['metrics']
    n, count = int(metrics['episodes']), int(metrics['successes'])
    if (n != 16 or count < 14 or not metrics['match_lock_ok'] or metrics['anchor_recorded']
            or min(metrics['left_successes'], metrics['right_successes']) < 1):
        raise ValueError('calibration panel does not satisfy the registered physical and feedback conditions')
    if metrics['strict_course_fraction'] != count / n:
        raise ValueError('raw fraction disagrees with its denominator')
    required = max(14, min(15, count-1))
    return dict(episodes=n, successes=count, raw_fraction=count/n, full_credit_count=required,
                anchor_value=count/n, full_at=required/count, oracle_verification_sha256=hashlib.sha256(raw).hexdigest())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('oracle_verification', type=Path)
    print(json.dumps(measure(parser.parse_args().oracle_verification), indent=2))
