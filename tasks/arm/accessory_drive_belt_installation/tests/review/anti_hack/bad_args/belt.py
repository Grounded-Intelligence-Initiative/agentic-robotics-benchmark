"""probe: out-of-range and non-finite arguments; each refusal is recorded; the NaN line goes last."""
import sys

from beltcell.client import CellError

BAD = [
    ("set_control", dict(arm="left", mode="cartesian", stiffness_scale=1e308)),
    ("set_control", dict(arm="middle", mode="joint", stiffness_scale=1.0)),
    ("set_control", dict(arm="left", mode="torque", stiffness_scale=1.0)),
    ("command", dict(left={"q": [0.0, 0.0, 0.0]})),
    ("command", dict(left={"q": [1e308] * 7})),
    ("command", dict(right={"width": -5.0})),
    ("command", dict(right={"width": 1e300})),
    ("command", dict(right={"tcp_pos": [0.4, 0.0, 0.4]})),
    ("step", dict(n_ticks=0)),
    ("step", dict(n_ticks=10 ** 400)),
    ("step", dict(n_ticks=-3)),
    ("get_camera", dict(name="rear", depth=True)),
    ("tare", dict(arm=None)),
]


def solve(cell):
    n = 0
    for meth, prm in BAD:
        try:
            cell._call(meth, **prm)
            print("probe bad_args: ACCEPTED", meth, prm, file=sys.stderr)
        except CellError as e:
            n += 1
            print("probe bad_args: refused", meth, str(e)[:80], file=sys.stderr)
        except Exception as e:  # noqa: BLE001
            print("probe bad_args: client exception", meth, type(e).__name__, file=sys.stderr)
    print("probe bad_args: refused %d of %d" % (n, len(BAD)), file=sys.stderr)
    cell.step(5)
    try:
        cell._call("command", left={"tcp_pos": [float("nan"), 0.0, 0.4], "tcp_quat": [1.0, 0.0, 0.0, 0.0]})
    except Exception as e:  # noqa: BLE001
        print("probe bad_args: NaN line ->", type(e).__name__, str(e)[:80], file=sys.stderr)
