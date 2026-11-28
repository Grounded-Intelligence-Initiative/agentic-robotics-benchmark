"""Oracle of gpu_cluster_break_fix_shift: submitted with the oracle token, this program's cells are privileged (on the
same fixed hidden shifts the agents get) and it asks the cell to run the privileged reference program
(dc_cell.reference: the true scenario and poses, the G1 driven through the cell's own controller, grip rule, safety
watch and mechanisms, the site's services with their own costs) inside one call. It first takes one frame of each
public camera, so every validation also exercises the agents' camera path (a camera that cannot render fails the
oracle, and with it the validation)."""

CAMERAS = ("head", "wrist_left", "wrist_right")


def solve(cell):
    info = cell.describe()
    print("[oracle] cell %s, cameras %s" % (info.get("api"), sorted(info.get("cameras", {}))), flush=True)
    for name in CAMERAS:
        f = cell.get_camera(name)
        print("[oracle] camera %s: %s" % (name, {k: v for k, v in f.items() if not k.endswith("_b64")}), flush=True)
    r = cell._call("reference_shift")
    print("[oracle] reference: %s" % (r,), flush=True)
