"""furniture_client: the program-side library of the furniture cell (the proxy, frame decoding, the transports).

    from furniture_client import Cell, CellError, decode_frame, connect_practice, connect_graded, given_dir

The graded episode is reached with connect_graded() (a GradedCell: the same proxy plus status() and start()); the
practice lab with connect_practice().
"""

from .client import (BOOTSTRAP, GRADED_SOCKET, PRACTICE_SOCKET, PUBLIC_CAMERAS, Cell, CellError, GradedCell,
                     connect_graded, connect_practice, connect_stdio, decode_frame, given_dir, main)

__all__ = ["Cell", "CellError", "GradedCell", "decode_frame", "connect_stdio", "connect_practice", "connect_graded",
           "given_dir", "main", "BOOTSTRAP", "PUBLIC_CAMERAS", "PRACTICE_SOCKET", "GRADED_SOCKET"]
