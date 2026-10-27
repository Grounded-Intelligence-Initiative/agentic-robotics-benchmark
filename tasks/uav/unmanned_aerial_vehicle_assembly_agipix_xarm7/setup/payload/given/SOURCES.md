# Component provenance and adaptation

AGiPix source: https://github.com/SasaKuruppuarachchi/agipix/tree/147a736b9fea7ac0e8d08c9a5fb0b90e5baa224a (CC0-1.0; licence retained in `kit/AGIPIX_LICENSE.txt`). This kit preserves the source manufacturing outlines of the 4 mm lower chassis, two laminated 1.5 mm guard panels, 1.5 mm compute cover, 4 mm sensor-cage cheeks and 1.5 mm cage cap. The CW/CCW propeller surfaces come from the source simulation exports. Source-specific file hashes are recorded in `kit/kit.json`.

xArm 7 and gripper: https://github.com/google-deepmind/mujoco_menagerie/tree/f3b6013629c7b94f8ee93fd977b161fa8350f9bc/ufactory_xarm7 (BSD-3-Clause; licence retained in `xarm7/LICENSE`). The original joint chain, gripper linkage, inertia and bounded actuators are used.

This is an AGiPix-derived mechanical assembly variant, not a manufacturing certification or an exact replica. The combined upstream body mesh is fused and is not used as a collection of loose parts. Purchased avionics use the supplied rigid engineering envelopes, with stated mass and dimensions; they are not detailed manufacturer CAD. The sensor cage, top-mounted battery, avionics positions, captive mounting posts and landing struts are adapted for the workcell. Screws, damping films and small captive spacers are included in their parent component. Flexible wiring, compliant deformation, electronics operation and flight are not modelled.

The two laminations form one purchased guard panel. Motor cases are loose components; each has a separate unpowered rotor. A mounted propeller rotates with that rotor rather than being welded to the fixed chassis. Propeller yaw is mechanically free, but handedness must match the required rotor station.

The bench CAD includes collision proxies as well as visual meshes. Any source/variant differences needed for assembly are described in the supplied manual and drawings.
