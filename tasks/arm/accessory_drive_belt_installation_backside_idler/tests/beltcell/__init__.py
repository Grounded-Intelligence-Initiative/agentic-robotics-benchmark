"""beltcell: the accessory-drive belt station (cell internals: the grader runs this package; the agent gets only
client.py).

Modules
    client      the agent-side proxy of the cell and the program launcher (JSON-lines wire on stdio)
    layout      the engine drawn from its seed (pose on the pallet, the belt's and the tensioner's spread, the drape)
    drive       the pulleys, the automatic tensioner, the belt chain, the acceptance geometry
    physics     the scene: base plate, the engine and its drive, the belt on its peg, two Pandas with hands and F/T
    control     the arms' joint / Cartesian impedance controller
    rules       the grip rule, the safety watch, damage
    cameras     RGB-D rendering, calibration and the sensor model
    episode     the acceptance after finish: run-in, inspection, the engine's score
    server      the cell API served over the wire (public calls only)
"""

CELL_VERSION = "belt-station/0.3.0"
