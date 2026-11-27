# Defects beyond the first-order model

`arm_model.py` is the manufacturer's first-order accuracy model of the arm. A real unit follows it to first order,
with its own values of the 84 parameters (within the tolerances of `cell.md`, section 3), and in addition has
**some** of the six defects below. Which defects a unit has, on which joints, and how large they are is not told and
differs from unit to unit; the practice unit is drawn the same way as the graded units and in general does not have
the same defects as any of them. A defect that a unit does not have is exactly zero. Every magnitude is at most the
bound stated with it. The service of `cell.md`, section 12 (one joint's motor and encoder replaced, its gear mesh set
anew) changes none of these defects: it changes only that joint's zero offset and backlash, both first-order
parameters.

## Notation

As in `arm_model.py` and `cell.md`, section 1: joints j = 0..5 from the base to the flange (joint 0 about the
vertical axis, 1 the shoulder, 2 the elbow, 3 the forearm's roll, 4 the wrist's bend, 5 the flange's roll), the
index j of every per-joint array. q_j is joint j's encoder reading, temp_j link temperature j, T0 = 20 °C, theta_j
the joint angle that enters the chain, frame j the frame after joint j's rotation, R_j its rotation and x_j, y_j its
x and y axes in the datum frame. "Link j" is the link that turns with joint j (`LINK_MASS[j]`, `LINK_COM[j]`). The
nominal chain is the drawing's: nominal xyz and rpy, theta = q, no thermal scaling.

For a unit with defects, theta_j is the first-order model's

    q_j + b_j s_j / 2 + h_j[0] sin q_j + h_j[1] cos q_j + h_j[2] sin 2q_j + h_j[3] cos 2q_j + c_j tau_j

plus the terms of defects 1, 2, 4 and 5; defects 3 and 6 add rotations to the chain. All terms use the encoder
readings q (not theta) and the true link temperatures. The gravity moments below are computed like
`gravity_torques`: on the nominal chain at the encoder readings, with the weights (mass times G, along -z of the
datum frame) of the links named and of the mounted tool (its certificate's mass at its centre of gravity).

## 1. Axis coupling in the wrist

The wrist's hollow-shaft transmissions couple each wrist axis to the one before it:

    theta_4 += k45 · q_3
    theta_5 += k56 · q_4

|k45| ≤ 8.0e-4 and |k56| ≤ 8.0e-4 (rad per rad), either sign.

## 2. Transmission ratio error

On one or two of the joints 0, 3 and 5 (the joints with the widest working ranges), the transmission ratio differs
from its nominal value:

    theta_j += e_j · q_j

|e_j| ≤ 1.2e-3 (rad per rad), either sign. The other joints have none.

## 3. Link torsion and bending under load

The upper arm (link 1, from joint 1 to joint 2; its long axis is x_1) and the forearm (from joint 2 to the wrist
centre, the origin of frame 4; its long axis is x_2) twist about their long axis and bend out of the plane of
motion, in proportion to the gravity moment they carry.

- Upper arm: M_u is the gravity moment of links 1 to 5 and the tool about the origin of frame 1. Its twist moment
  is T_u = M_u · x_1 and its bending moment B_u = M_u · y_1 (x_1, y_1 of the nominal chain).
- Forearm: M_f is the gravity moment of links 2 to 5 and the tool about the origin of frame 2; T_f = M_f · x_2,
  B_f = M_f · y_2 (x_2, y_2 of the nominal chain).

The chain is rotated at each part's far end:

- at the end of the upper arm (after the translation xyz_2 of T_2 and before its rotations Rx(rpy_2[0]) ...), by
  Rx(kt_u T_u) Ry(kb_u B_u): rotations about the x and y axes of the unit's actual frame 1, about the origin of
  frame 2;
- at the end of the forearm (the wrist centre: after the translation xyz_4 of T_4 and before its rotations), by
  Rx(kt_f T_f) Ry(kb_f B_f) about the x and y axes of the unit's actual frame 2: the rest of the chain is rotated
  by R_2 Rx(kt_f T_f) Ry(kb_f B_f) R_2^T in the datum frame about the wrist centre, R_2 the actual rotation of
  frame 2.

The four compliances kt_u, kb_u, kt_f, kb_f are each between 0 and 3.0e-4 rad / (N m).

## 4. A higher-order cyclic transmission error

On one of the joints 0, 1, 2, a harmonic of integer order n, 3 ≤ n ≤ 8, beyond the first and second of the
first-order model:

    theta_j += a · sin(n q_j + phi)

with amplitude 0 < a ≤ 1.0e-3 rad and any phase phi. The joint, the order, the amplitude and the phase are not told.

## 5. Thermal drift of a joint's zero

On one or two of the joints 1, 2 and 4, the joint's zero moves with its link temperature:

    theta_j += d_j · (temp_j - T0)

|d_1| ≤ 5.0e-5, |d_2| ≤ 6.0e-5, |d_4| ≤ 1.0e-4 (rad / K), either sign.

## 6. Pedestal compliance

The pedestal tilts the whole arm about the two horizontal axes of the datum frame through the datum origin, in
proportion to the overturning moment: with M_0 the gravity moment of links 0 to 5 and the tool about the datum
origin, the whole chain (joint 0's frame and everything after it) is rotated by

    Rx(kx · M_0,x) Ry(ky · M_0,y)

before T_0, about the datum frame's x and y axes. kx and ky are each between 0 and 5.0e-5 rad / (N m).

## The camera

The wrist camera is fixed to the flange: its pose is the flange's true pose (with the defects) followed by its
mounting, as in `arm_model.camera_pose`. The camera test of the acceptance places the camera-frame points with the
true camera pose.
