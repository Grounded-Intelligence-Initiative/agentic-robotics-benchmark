"""The observation a step receives (CONTRACT.md section 6; contract/schemas/obs.json).

    regs_block(raw, reads)          -> {key: [13 ints]} for every register lying wholly inside the read blocks
    tactile_block(frame)            -> None | {"seq", "t_capture_ms", "t_arrival_ms", "modules": {module: hex}}
    build(cycle=, t_ms=, period_ms=, late=, regs=, tactile=, cell=, refused=, event=) -> the observation

Register values are given signed as the vendor's table says (two's complement for the signed registers, plain for the
others), whatever integer form the bus layer hands over. No pose, no force and no truth is ever added here; a privileged
run adds its "truth_cmd" afterwards (acceptance).
"""
from contract import names as N


def as_register(value, reg):
    """An integer read from the bus in the register's own signedness and width."""
    v = int(value)
    bits = 8 * reg.size
    v &= (1 << bits) - 1
    if reg.signed and v >= 1 << (bits - 1):
        v -= 1 << bits
    return v


def regs_block(raw, reads):
    """The registers of the declared read blocks, by key, each a list of 13 integers."""
    out = {}
    for reg in N.registers_in(reads):
        vals = raw.get(reg.key)
        if vals is None or len(vals) != N.N_SERVOS:
            raise RuntimeError("the bus returned no 13 values for %s" % reg.key)
        out[reg.key] = [as_register(v, reg) for v in vals]
    return out


def frame_bytes_expected(module):
    from contract import constants as C
    return C.TACTILE["resultant_bytes"] + C.TACTILE["taxel_bytes"] * N.MODULE_TAXELS[module]


def tactile_block(frame):
    """The newest tactile frame as the observation carries it, or None."""
    if frame is None:
        return None
    seq, t_cap, t_arr, mods = frame
    out = {}
    for m in N.MODULES:
        b = bytes(mods[m])
        if len(b) != frame_bytes_expected(m):
            raise RuntimeError("tactile frame of %s has %d bytes" % (m, len(b)))
        out[m] = b.hex()
    return {"seq": int(seq), "t_capture_ms": float(t_cap), "t_arrival_ms": float(t_arr), "modules": out}


def build(*, cycle, t_ms, period_ms, late, regs, tactile, cell, refused, event):
    """The observation (a fresh dict of plain JSON values)."""
    return {"cycle": int(cycle), "t_ms": int(t_ms), "period_ms": int(period_ms), "late": int(late),
            "regs": {k: list(v) for k, v in regs.items()},
            "tactile": tactile,
            "cell": dict(cell, ticket=dict(cell["ticket"]) if cell.get("ticket") else None),
            "refused": [[str(f), int(i)] for f, i in refused],
            "event": dict(event) if event else None}
