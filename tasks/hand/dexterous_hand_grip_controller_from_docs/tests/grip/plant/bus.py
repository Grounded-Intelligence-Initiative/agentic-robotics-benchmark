"""The servo bus as the host sees it: which read blocks are allowed, how long a cycle takes, what a sync read returns.

A cycle writes the previous step reply (one sync write) and reads each declared block from all thirteen servos (one
sync read per block). Its effective period is the contract's bus rule: the requested period, or the bus time of the
writes and the reads rounded up to whole milliseconds if that is longer. A sync read returns the bytes of the control
table; each register lying wholly inside a block is decoded with the table's size and signedness.
"""
import struct

from contract import constants as C
from contract import names as N

_FMT = {1: "b", 2: "h", 4: "i"}


def check_reads(reads):
    """Raise ValueError unless reads is 1..4 blocks [address, length] inside the RAM area."""
    if not isinstance(reads, (list, tuple)) or not 1 <= len(reads) <= C.INTERFACE["read_blocks_max"]:
        raise ValueError("reads: 1 to %d blocks" % C.INTERFACE["read_blocks_max"])
    for blk in reads:
        if not isinstance(blk, (list, tuple)) or len(blk) != 2:
            raise ValueError("a read block is [address, length]")
        a, n = blk
        if not all(isinstance(x, int) and not isinstance(x, bool) for x in (a, n)):
            raise ValueError("a read block holds integers")
        if n < 1 or a < N.RAM_FIRST or a + n > N.RAM_END:
            raise ValueError("read block outside %d..%d: %r" % (N.RAM_FIRST, N.RAM_END - 1, blk))
    return [[int(a), int(n)] for a, n in reads]


def period_ms(reads, requested=None):
    """The effective cycle period (ms) for the read blocks and a requested period (default: the shortest)."""
    req = C.INTERFACE["period_ms"][0] if requested is None else int(requested)
    return C.period_ms(req, reads)


def encode(value, size, signed):
    """The little-endian bytes of one register value (two's complement where the table says signed)."""
    fmt = "<" + (_FMT[size] if signed else _FMT[size].upper())
    if signed:
        lo, hi = -(1 << (8 * size - 1)), (1 << (8 * size - 1)) - 1
    else:
        lo, hi = 0, (1 << (8 * size)) - 1
    v = int(value)
    if not lo <= v <= hi:
        v = v & ((1 << (8 * size)) - 1)
        if signed and v > hi:
            v -= 1 << (8 * size)
    return struct.pack(fmt, v)


def decode(raw, signed):
    size = len(raw)
    fmt = "<" + (_FMT[size] if signed else _FMT[size].upper())
    return struct.unpack(fmt, raw)[0]


def sync_read(regfile, reads):
    """{key: [13 ints]} for every RAM register lying wholly inside the blocks, through the table's bytes.
    regfile: {key: sequence of 13 ints} holding at least those registers."""
    out = {}
    for r in N.registers_in(reads):
        vals = regfile[r.key]
        out[r.key] = [decode(encode(v, r.size, r.signed), r.signed) for v in vals]
    return out
