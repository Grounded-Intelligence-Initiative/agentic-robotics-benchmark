"""house_client: the program-side library of the house cell (the proxy, frame decoding, the transport).

    from house_client import Cell, CellError, connect_graded, decode_frame, load_frame, given_dir

connect_graded() connects to this session's graded episode (a GradedCell: the Cell proxy plus start(), status(),
close()); load_frame() reads a survey photo as a get_camera reply.
"""

from .client import (BOOTSTRAP, GRADED_SOCKET, PUBLIC_CAMERAS, Cell, CellError, GradedCell, connect_graded, connect_stdio,
                     decode_frame, given_dir, load_frame, main)

__all__ = ["Cell", "CellError", "GradedCell", "connect_graded", "decode_frame", "load_frame", "connect_stdio", "given_dir",
           "main", "BOOTSTRAP", "PUBLIC_CAMERAS", "GRADED_SOCKET"]
