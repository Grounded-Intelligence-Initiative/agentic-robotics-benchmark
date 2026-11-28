"""The controller's replies, checked against the runtime contract (CONTRACT.md sections 3 and 5).

    parse_setup(reply) -> {"period_ms": int, "reads": [[address, length], ...]}      raises SetupError
    parse_step(reply)  -> None (no command) | {"write": {field: [13 entries]}, "tactile_zero": [module, ...],
                                               "grasped": bool, "decline": bool}       raises ProtocolError

An integer is a Python int that is not a bool: a reply decoded from JSON gives a float for 600.0, which is not an
integer. Every write value must lie inside the register's printed range (contract.names.REGISTERS). A write key outside
the list of writable fields, a field whose value is not a list of 13 entries, an entry that is neither null nor an
integer in range, a tactile_zero that is not a list of distinct module names, or a grasped / decline flag that is
neither a boolean nor null is a protocol error. Other top-level keys of a step reply are ignored.
"""
from contract import constants as C
from contract import names as N


class ProtocolError(ValueError):
    """A step reply that breaks the contract: it ends the controller for the unit."""


class SetupError(ValueError):
    """A setup reply that breaks the contract: a failed setup."""


def is_int(v):
    return type(v) is int


def shown(v):
    """A short, safe description of a reply value for a message (never the repr of a nested object)."""
    if isinstance(v, str):
        return repr(v[:40])
    if v is None or isinstance(v, (bool, int, float)):
        return repr(v)[:40]
    return "a " + type(v).__name__


def parse_setup(reply):
    """The setup reply as {"period_ms", "reads"}; anything else raises SetupError."""
    if not isinstance(reply, dict):
        raise SetupError("the setup reply is not an object")
    if set(reply) != {"period_ms", "reads"}:
        raise SetupError("the setup reply must hold exactly period_ms and reads (got %s)" % sorted(str(k)[:40] for k in
                                                                                                  reply)[:8])
    p = reply["period_ms"]
    lo, hi = C.INTERFACE["period_ms"]
    if not is_int(p) or not lo <= p <= hi:
        raise SetupError("period_ms must be an integer in %d..%d" % (lo, hi))
    reads = reply["reads"]
    if not isinstance(reads, list) or not 1 <= len(reads) <= C.INTERFACE["read_blocks_max"]:
        raise SetupError("reads must be a list of 1 to %d blocks" % C.INTERFACE["read_blocks_max"])
    out = []
    first, last = C.INTERFACE["read_area"]
    for blk in reads:
        if not isinstance(blk, list) or len(blk) != 2 or not all(is_int(x) for x in blk):
            raise SetupError("a read block is a list [address, length] of two integers")
        a, n = blk
        if n < 1 or a < first or a + n > last + 1:
            raise SetupError("read block %r lies outside the RAM area %d..%d" % (blk, first, last))
        out.append([a, n])
    return {"period_ms": p, "reads": out}


def parse_step(reply):
    """The step reply in canonical form (see the module docstring); None = no command."""
    if reply is None:
        return None
    if not isinstance(reply, dict):
        raise ProtocolError("the step reply is neither an object nor null")
    write = reply.get("write")
    fields = {}
    if write is not None:
        if not isinstance(write, dict):
            raise ProtocolError("write is neither an object nor null")
        for key, vals in write.items():
            if key not in N.WRITE_ADDRESS:
                raise ProtocolError("write key %s is not a writable field" % shown(key))
            reg = N.REGISTER[key]
            if not isinstance(vals, list) or len(vals) != N.N_SERVOS:
                raise ProtocolError("write %s must be a list of %d entries" % (key, N.N_SERVOS))
            row = []
            for i, v in enumerate(vals):
                if v is None:
                    row.append(None)
                    continue
                if not is_int(v):
                    raise ProtocolError("write %s[%d] is not an integer" % (key, i))
                if not reg.lo <= v <= reg.hi:
                    raise ProtocolError("write %s[%d] = %d is outside %d..%d" % (key, i, v, reg.lo, reg.hi))
                row.append(v)
            if any(x is not None for x in row):
                fields[key] = row
    tz = reply.get("tactile_zero")
    zero = []
    if tz is not None:
        if not isinstance(tz, list):
            raise ProtocolError("tactile_zero is neither a list nor null")
        for m in tz:
            if not isinstance(m, str) or m not in N.MODULES:
                raise ProtocolError("tactile_zero names %s, not a module" % shown(m))
            if m in zero:
                raise ProtocolError("tactile_zero names %s twice" % m)
            zero.append(m)
    flags = {}
    for k in ("grasped", "decline"):
        v = reply.get(k)
        if v is None:
            flags[k] = False
        elif isinstance(v, bool):
            flags[k] = v
        else:
            raise ProtocolError("%s is neither a boolean nor null" % k)
    return {"write": fields, "tactile_zero": zero, "grasped": flags["grasped"], "decline": flags["decline"]}
