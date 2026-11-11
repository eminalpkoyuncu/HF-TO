# Heron Gazebo (headless) for the HF-TO / MPC-Graph runs

ROS Melodic + Gazebo 9 in Docker with Clearpath `heron` (melodic-devel 3ccbef6) and
`heron_simulator` (kinetic-devel 46cb837), the same versions and EKF config as
`~/CHOMP_ws`. Gazebo runs as `gzserver` only, on host networking, so MATLAB on the
host connects to `http://localhost:11311`.

```bash
./gz.sh build     # once (~10 min)
./gz.sh up        # headless sim, waits until /odometry/base is up
./gz.sh test      # optional: drives the boat from inside the container
./gz.sh restart   # reset the boat to the origin between runs
./gz.sh down
./gz.sh gui       # same sim with the Gazebo window, if you want to watch
```

Then in MATLAB (after `setup`): `run_hfto_gazebo` or `run_mpcgraph_gazebo`
(`experiments/`). `gazebo_smoke_test` checks the link open loop.

Topics used by `ros_io.m`:

| topic | type | |
|---|---|---|
| `/heron/thrusters/0/input` | uuv FloatStamped | right thruster, normalized [-1, 1] |
| `/heron/thrusters/1/input` | uuv FloatStamped | left thruster |
| `/odometry/base` | nav_msgs/Odometry | ground truth (`/pose_gt`), twist in body frame (u, v, r); from `ocean_gazebo/scripts/base_odometry.py` |
| `/clock` | rosgraph_msgs/Clock | sim time |

`matlab_msgs/` holds the FloatStamped definition and its `rosgenmsg` output;
`ros_io.m` adds it to the MATLAB path. To regenerate it (MATLAB's ROS venv was
created without ensurepip, see below):

```matlab
cd Gazebo_ws/matlab_msgs; rosgenmsg(pwd)
```

MATLAB ROS venv: `~/.matlab/R2025a/ros1/glnxa64/venv` was made with
`python3 -m venv --without-pip` plus pip from `.venv/pip.pyz`, because
`python3.10-venv` is not installed (installing it with apt also works).
