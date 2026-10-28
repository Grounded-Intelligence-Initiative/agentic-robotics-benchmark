# Customer brief / revision C

B-01. Supply eight replicas of U-17 in its final configuration after the 25 September rebuild and subsequent survey.
Retain that configuration's fabricated guarded core, mounting positions, battery cage and
ballast-pod position. Retain the PX4 / ROS 2 split, sensors and compute. Purchase only the seven BOM lines in B-03.

B-02. Fly with the replacement ballast pod shown on the scale in PH-12. The display is grams, tare zero.
Use the survey register and build history to establish its final installed position and box extents;
its centre of mass is its box centre. The photographs span more than one build stage.
The pod and scanner box edges are parallel to the vehicle axes. Their external outlines must be measured.
Every replica carries this mass throughout the duty. The old shipping note's 250 g is obsolete.

B-03. Four independent motor lines, stations FL/FR/RL/RR, one shared propeller line (4 units, two of each handedness),
one 4-in-1 ESC line, one 6S battery line. No stacking batteries, mixed propeller types or modification of the core.
Prices and delivery lot are fixed by Q-01. All quoted options may be evaluated, even if one cannot fit this unit.

B-04. Coordinates: x forward (toward scanner), y left, z up. Origin is the midpoint of the four rotor shafts in
the plane of the upper flat circular faces of the reference motor cases, excluding propeller hubs and nuts.
This is the contractual thrust plane. FL is x positive/y positive; FR positive/negative; RL negative/positive; RR negative/negative.
Spin is viewed from above. The body yaw-reaction signs in this station order are -1,+1,+1,-1.
Install CCW propellers at FL/RR and CW at FR/RL. Full-row shaft spacing is twice the corresponding half-span.

B-05. Propeller swept diameter plus 2 mm clearance at EACH tip must fit the smallest shaft-centred inscribed
circle in any guard. The guard aperture can be offset from its shaft; use the limiting shaft-to-inner-edge distance,
not the guard outer diameter. Battery boxes must fit the cage's internal x/y/z dimensions without rotation.
Motors must share this unit's 16 mm mounting-hole pitch. Frame or cage rework is disallowed.

B-06. Use the delivered lot B. Lot A is retained for comparison only. For each motor SKU, retained thrust multiplier
is the minimum of its three normalized samples minus 0.006; retained electrical-power multiplier is the maximum
plus 0.008. Battery capacity factor is minimum measured retained fraction minus 0.005; resistance factor is maximum
measured multiple plus 0.02. ESC limit factor is minimum measured retained fraction minus 0.01. The vendor-page
erratum takes precedence over the preliminary page. Apply lot factors once, after the erratum.

B-07. Evaluate a 390 s hover and all eight 4 s manoeuvre states: az=3.1 m/s^2, ax=5.4 m/s^2;
omega=(s*2.2,p*2.7,y*1.8) rad/s and alpha=(s*8.2,p*10.7,y*1.2) rad/s^2 for every s,p,y in {-1,+1}.
Each manoeuvre includes a prescribed lateral gust G=(-s*6,p*4,0) N acting at the payload box centre.
Hover has zero acceleration/rate and zero gust. No takeoff, cruise or reserve time is silently added.
Auxiliary battery-side power is 38 W throughout; g=9.80665 m/s^2. Energy reserve is 20% of retained battery energy.

B-08. Use the vendor bench fit at 22.2 V, not throttle percentages. Retained maximum thrust scales with V^2.
Per-motor electrical power is k*T^(3/2), including the propeller and delivered-lot power factors; T is newtons,
k has units W/N^(3/2). Divide by ESC efficiency to obtain battery-side motor power; add B-07 auxiliary power.
At end-of-mission test open-circuit voltage, solve V=Voc-R*P/V using the HIGH-voltage root. No real root is failure.
Total battery current is P/V; each ESC-channel current excludes auxiliary current.

B-09. Contractual rigid-body reduction: reconstruct the final dry-core mass using BL-01, then use it as
a uniform box centred at (0,0,-4 mm). Use the scanner
as a uniform box with its photographed extents and centre (mass from DS-08), the payload as the B-02 box,
and the chosen battery/ESC as
their datasheet boxes. The ESC centre is (0,0,-8 mm). Motors plus propellers are point masses at the four shafts.
Compute the combined CoM and full inertia tensor ABOUT THAT CoM, including parallel-axis terms.
For each duty state, total thrust F=sqrt((m*(g+az))^2+(m*ax-Gx)^2+Gy^2). The required rotor moment
about the shaft midpoint is I*alpha + omega cross (I*omega) + CoM cross (0,0,F)
- (payload_centre-CoM) cross G. Forces and moments in this envelope are expressed in the vehicle axes.
Solve the four-rotor allocation with rows [1], [y_i], [-x_i], [reaction_sign_i*propeller_reaction_arm].
This prescribes the sizing envelope; aerodynamic drag and detailed CAD density are outside this contract.

B-10. Every allocated thrust must be nonnegative and below its retained voltage-dependent capacity. Check motor
current and ESC peak channel, time-weighted RMS channel and sum-of-channels board limits, applying the ESC lot
factor to all three ESC limits. Also check battery peak current, loaded voltage against both cutoff limits,
and sum(P*seconds)/3600 against 80% of retained energy. All nine states must pass simultaneously.

B-11. Select the least-cost passing BOM. Economy uses the cheapest passing design's cost J*. For each passing
repair of your BOM, add 12 USD for each changed motor station, shared propeller line, ESC line or battery line.
Effective cost is the minimum of those repaired costs. Economy decreases linearly from 1 at J* to 0 at J*+12 USD.
For an already passing BOM, retaining it is an available zero-change repair. Fixed core costs cancel out.

B-12. Submit the three files using the schema and trace every reported quantity to clause/photo/page IDs.
Do not rely on catalogue prices or nominal upstream dimensions as measurements of this fabricated unit.
