# Trajectory optimization within sequentially connected funnels for underactuated USVs

MATLAB code of the paper

> E. Koyuncu and M. M. Ankarali, *Trajectory Optimization within Sequentially
> Connected Funnels for Underactuated USVs*, Ocean Engineering (under review).

The method plans and tracks a trajectory for an underactuated unmanned
surface vehicle (two thrusters, no sway actuation) through a sequence of
obstacle-free convex regions, the *hollow funnels*:

1. **Funnels and graph search.** The free workspace is covered by rectangular
   funnels (sparse neighborhood graph), and Dijkstra finds the lowest-cost
   funnel sequence from the start to the goal.
2. **Trajectory optimization.** A receding-horizon trajectory optimization
   (direct transcription, `fmincon` with exact derivatives) computes a
   dynamically feasible nominal trajectory through the sequence, two funnels
   at a time, with every step constrained to its funnel.
3. **Tracking.** An LTV-MPC linearized along the nominal tracks it, with the
   predicted positions constrained to the same funnels (a convex QP).
   Optionally, an extended Kalman filter estimates a constant current, which
   enters the MPC prediction.
4. **Safety certificate.** For a bounded thrust disturbance, a zonotopic error
   tube certifies that the vessel stays inside the funnels at all times.

The repository also contains the MPC-Graph baseline (Atasoy et al., 2023),
re-implemented for the same underactuated vessel, and the Gazebo setup of the
Clearpath Heron used in the paper.

## Requirements

- MATLAB (tested with R2025a)
- Optimization Toolbox (`fmincon`, `quadprog`)
- Mapping Toolbox (`polybool`, `poly2cw`; funnel overlaps)
- Statistics and Machine Learning Toolbox (`pdist2`)
- Control System Toolbox (`dare`; MPC-Graph terminal cost)
- Parallel Computing Toolbox (optional; Monte Carlo runs in parallel)
- ROS Toolbox and Docker (only for the Gazebo experiments)

## Quick start

```bash
git clone https://github.com/eminalpkoyuncu/HF-TO.git
```

```matlab
cd HF-TO
setup                                   % adds the code to the path
cfgOverride = struct('mapNo',13,'nRuns',20);
run_hfto_mc                             % our method, Map 2, 20 runs
cfgOverride = struct('mapNo',13,'nRuns',20);
run_mpcgraph_mc                         % MPC-Graph on the same runs
summarize_results(13)                   % side-by-side table
```

The first call plans the nominal trajectory (about 25 s for Map 2 with the
Fossen model) and saves it in `results/`; later runs reuse it. All settings
are in `config/default_config.m` and can be overridden by defining a struct
`cfgOverride` before running an experiment script.

## Usage

Run `setup` once per MATLAB session. Every experiment is a script in
`experiments/`; it reads its settings from `config/default_config.m` and
applies the fields of a struct `cfgOverride`, if one exists in the workspace.
The scripts clear the workspace but keep `cfgOverride`, so change it (or
`clear cfgOverride`) before running the next script.

### Monte Carlo experiments

| Script | What it runs |
|---|---|
| `run_hfto_mc` | our method on one map and model: plans once, then `cfg.nRuns` closed-loop runs |
| `run_mpcgraph_mc` | MPC-Graph on the same runs (same seeds) |
| `run_monte_carlo` | all sets of the paper: both methods, both maps, both models, all cases |
| `summarize_results(mapNo)` | table of both methods from the files in `results/` |

```matlab
cfgOverride = struct('mapNo',3,'model','identified','nRuns',100);
run_hfto_mc
```

`run_monte_carlo` takes a struct `batch` instead, to restrict what it runs:

```matlab
batch.maps = 13; batch.models = {'fossen'}; batch.cases = {'baseline'};
batch.est = false; batch.methods = {'hfto'}; batch.nRuns = 50;
run_monte_carlo
```

### Settings

The most used fields of `cfgOverride` (all of them, with their defaults, are
in `config/default_config.m`):

| Field | Default | Meaning |
|---|---|---|
| `mapNo` | `13` | `3` = Map 1 of the paper, `13` = Map 2, `14` = Gazebo map |
| `model` | `'fossen'` | vessel model of the planner, the MPC and the simulated plant: `'fossen'` or `'identified'` (Heron). With `'identified'` the thrust limits switch to those of the Heron |
| `nRuns` | `500` | number of Monte Carlo runs |
| `noiseLevel` | `1` | thrust disturbance level (`0` = none) |
| `paramPert` | `0` | plant parameters perturbed by this fraction (`0.2` = ±20 %) |
| `current`, `currentStd` | `0`, `0` | mean and standard deviation of the speed of a constant current [m/s] |
| `est.on` | `false` | current estimate (EKF) in the MPC |
| `posMargin` | `0.15` | funnels tightened by this distance [m] |
| `vnom`, `vthr`, `vpark` | `1`, `0.5`, `0.3` | planning speeds [m/s]: middle, first and last funnel |
| `useParallel` | `true` | run the Monte Carlo runs in `parfor` |

Nested fields are set as `cfgOverride.est.on = true`. The nominal plan is
saved in `results/plan_map<N>_<model>.mat` and reused as long as the planning
settings are the same; otherwise it is computed again.

### Gazebo

```bash
cd gazebo
./gz.sh build      # once, about 10 min
./gz.sh up         # start the simulator (./gz.sh gui to see the Gazebo window)
```

```matlab
setup
run_hfto_gazebo                         % our method; run_mpcgraph_gazebo for MPC-Graph
summarize_results(14,'gazebo','identified')
```

The Gazebo scripts use the `'gazebo'` preset (map 14, identified model, no
added disturbance, higher planning speeds). After a run the boat stays at the
goal: `./gz.sh restart` puts it back to the start, `./gz.sh down` stops the
simulator. `cfgOverride.dryRun = true` runs the same script against the model
without Gazebo. Run the Gazebo experiments on an otherwise idle machine: the
controller runs in real time at 10 Hz. The simulator is headless in Docker
(ROS Melodic, Gazebo 9) and MATLAB on the host connects to it through ROS;
details and the ROS topics are in [`gazebo/README.md`](gazebo/README.md).

## Reproducing the paper

| Paper | Script | Output in `results/` |
|---|---|---|
| Monte Carlo experiments, Table 4, Figs. 7–9 | `run_monte_carlo` with `batch.cases = {'baseline'}; batch.est = false;` | `hfto_mce_map<N>_<model>.mat`, `mg_mce_*.mat` |
| Robustness (parameters, currents, current estimate), Table 5 | `run_monte_carlo` | `*_param*.mat`, `*_current*.mat`, `*_est*.mat` |
| MPC-Graph heading weight | `tune_mpcgraph_heading` | `tune_mpcgraph_heading.mat` |
| Safety certificate, Table 6, Fig. 10 | `run_certificate` | `optionA_map<N>_<model>.mat` |
| Gazebo experiment, Fig. 6 | `run_hfto_gazebo`, `run_mpcgraph_gazebo` (see Usage) | `*_gazebo_*.mat` |
| Computation times, Table 7 | `run_timing` (idle machine) | `timing.mat` |
| Figures | `paper_figures` | `results/figures/*.pdf` |

`run_monte_carlo` runs all 48 sets of 500 runs (about 30 min per set on 8
workers); see its header to restrict it. Run `k` of every experiment uses the
random seed `1 + k` for both methods. Note that MATLAB's parallel workers use a
different default random generator than the MATLAB client, so a run repeated
in a plain `for` loop sees a different disturbance sequence than in `parfor`;
the paper's results were computed with `parfor` (`cfg.useParallel = true`).

Maps are numbered as in the code: map 3 is Map 1 of the paper, map 13 is
Map 2, and map 14 is the Gazebo map (`src/maps/init_arena.m`). Models:
`'fossen'` (3-DoF model of Fossen with the parameters of the paper) and
`'identified'` (discrete-time model identified from Gazebo data of the Heron,
`src/models/heron_identified_params.csv`).

## Repository layout

```
setup.m                    adds the code to the MATLAB path
config/default_config.m    all settings (experiments, planner, MPC, MPC-Graph, Gazebo)
src/maps/                  maps of the paper
src/graph/                 funnel generation and graph search
src/models/                Fossen and identified vessel models, plant mismatch, current
src/planning/              trajectory optimization in the funnel sequence
src/control/               LTV-MPC tracking controller and current estimator
src/certificate/           TV-LQR gains, zonotope tube, certificate
src/sim/                   closed-loop simulation, metrics, plots
src/ros/                   ROS interface for Gazebo
baselines/mpc_graph/       MPC-Graph on the underactuated vessel
experiments/               scripts that produce the results of the paper
gazebo/                    Docker setup of the Heron simulator
```

## Citation

```bibtex
@article{koyuncu2026funnels,
  title   = {Trajectory Optimization within Sequentially Connected Funnels for Underactuated {USVs}},
  author  = {Koyuncu, Eminalp and Ankarali, Mustafa Mert},
  journal = {Ocean Engineering},
  note    = {Under review},
  year    = {2026}
}
```

The funnel generation and the MPC-Graph baseline follow

> S. Atasoy, O. K. Karagöz, M. M. Ankarali, Trajectory-free motion planning of
> an unmanned surface vehicle based on MPC and sparse neighborhood graph,
> IEEE Access 11 (2023).

## License

MIT, see [LICENSE](LICENSE).
