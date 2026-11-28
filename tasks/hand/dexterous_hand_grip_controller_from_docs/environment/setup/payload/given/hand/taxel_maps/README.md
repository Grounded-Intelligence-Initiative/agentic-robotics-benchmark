# Taxel maps

The makers' maps of the taxels' positions on the two module types, as the makers ship them with their driver:

| file | module | taxels |
|---|---|---|
| `finger_module.csv` | fingertip tactile module ZCP-F561 (index, middle and ring pads) | 52 |
| `thumb_module.csv` | thumb tactile module ZCP-DG1Z (thumb pad) | 127 |

Each row after the header is one taxel: its number (from 1) and its X, Y, Z position in mm in the module frame. Row n
is taxel n - 1 of the module's bytes (`../../tactile/tactile_interface.md`, section 2). The module frame of each pad
is its pad frame (from `../kinematics_sheet.md`, in H with every joint at 0 deg):

| module | link | origin (m) | normal (z) | x axis |
|---|---|---|---|---|
| thumb | thumb_ip | [-0.116753, 0.108755, -0.00601669] | [0.333991, 0.391215, -0.857555] | [-0.727676, -0.471263, -0.498396] |
| index | index_dip | [-0.0331232, 0.19312, -0.0152077] | [0.00253185, 0.212019, -0.977262] | [-0.999996, -0.000453988, -0.00268924] |
| middle | middle_dip | [-0.00252322, 0.19812, -0.0152077] | [0.00253185, 0.212019, -0.977262] | [-0.999996, -0.000453988, -0.00268924] |
| ring | ring_dip | [0.0280768, 0.19312, -0.0152077] | [0.00253185, 0.212019, -0.977262] | [-0.999996, -0.000453988, -0.00268924] |

The makers measured a taxel pitch of 2.82 mm on the fingertip module and 2.52 mm on the thumb module
(`../../makers/handoff.md`, section 2).
