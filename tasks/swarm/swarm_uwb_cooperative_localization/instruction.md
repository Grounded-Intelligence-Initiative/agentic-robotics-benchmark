# Localize a drone swarm in a warehouse from its own radios

A logistics operator runs eight small inventory drones in a warehouse. They fly the aisles to
scan shelf labels and hop over the racks between aisles. There is no GPS indoors and no
motion-capture system in the hall: every drone has to know where it is from what it carries: UWB radios that range to each other and to six
beacons on the walls, a downward optical-flow sensor with a ToF rangefinder, and a gyro.

The positions this gives today are off by decimetres and more, not good enough to tell which
shelf slot a scan belongs to. None of the sensors has been calibrated in this installation, and
nothing has ever measured where the drones actually were: all there is to go on is the logs.

Write the swarm's **localization program**: given the logs of one flight of all eight drones, it
reconstructs every drone's trajectory.

You have 3 hours.

## What you have (in `/home/user`)

- `README.md`: the site, the sensors and the log formats. Read it first; it is part of this
  task's specification.
- `flights/flight_01/`, `flights/flight_02/`, `flights/flight_03/`: three flights of the whole
  swarm, in exactly the format of the flights your program is graded on. There is no ground
  truth for any of them.
- `tools/check_output.py`: is your output complete and well-formed (it knows nothing about
  accuracy).

Python 3.12 with numpy, scipy, pandas and matplotlib is installed (`python3`). There is no
network.

## What you submit

A program in `/home/user/submission/` that runs as

```
bash /home/user/submission/run.sh <flight_dir> <out_dir>
```

with `/home/user/submission/` as its working directory. `<flight_dir>` is a flight folder in the
format of `flights/flight_01/`; `<out_dir>` exists and is empty. The program
writes `<out_dir>/trajectories.csv`:

```
t_s,drone,x_m,y_m,z_m
0.00,0,36.0000,4.0000,0.0000
0.00,1,37.2000,4.0000,0.0000
...
```

one row per drone (0 to 7) and per output instant `t_s = 0.0, 0.1, ..., duration_s - 0.1`
(`flight.json`), with the drone's position in the warehouse frame in metres. Put everything the
program needs (code, any calibration you derived) in `/home/user/submission/`: when it is graded
it cannot read `~/flights` or anything else outside the submission folder. There is no submit
step: what is in the folder when your session ends is graded.

## How it is graded

After your session, three hidden 150-second flights of the same eight drones and six beacons in
the same warehouse are logged, with new flight plans of the same kind as the three you have.
Like those, each has its own gyro biases, starting headings, board temperatures at take-off and
stock on the racks. Your program is run once per flight, at your account's privileges, and must
finish each within 10 minutes on 2 CPU cores.

The score is the **root-mean-square 3-D position error in metres** over all drones, all output
instants and all three flights: lower is better, and it is compared with what a careful
reference localizer achieves. A position more than 10 m off counts as 10 m, and a missing,
malformed or non-finite row counts as 10 m off. If the program produces no usable trajectory in
any flight, the score is zero.
