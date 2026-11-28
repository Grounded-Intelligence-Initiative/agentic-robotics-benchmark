"""Counter-based noise streams of the plant.

Each stream is keyed by its seed; the draw for (index, column) is row `index`, column `column` of a standard-normal table
that is generated block by block from a Philox counter set to the block number. A draw therefore depends only on its
seed, its index (a firmware tick or a tactile frame) and its column (a servo or a taxel): never on how many draws were
taken before, nor on what a host reads, so two read patterns see the same values on the registers they share.
"""
import numpy as np

BLOCK = 1024


class NormalStream:
    """Standard-normal draws z[index, column] for columns 0..width-1, keyed by an integer seed."""

    def __init__(self, seed, width, block=BLOCK):
        self.seed = int(seed) & ((1 << 128) - 1)
        self.width = int(width)
        self.block = int(block)
        self._b = None
        self._rows = None

    def _fill(self, b):
        bg = np.random.Philox(key=self.seed, counter=np.array([b, 0, 0, 0], dtype=np.uint64))
        self._rows = np.random.Generator(bg).standard_normal((self.block, self.width))
        self._b = b

    def row(self, index):
        """The draws of one index, as a read-only array of `width` floats."""
        index = int(index)
        if index < 0:
            raise ValueError("negative index")
        b, k = divmod(index, self.block)
        if b != self._b:
            self._fill(b)
        r = self._rows[k]
        r.flags.writeable = False
        return r
