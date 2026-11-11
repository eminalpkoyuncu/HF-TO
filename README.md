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

```matlab
cd hf-to-usv
setup                                   % adds the code to the path
cfgOverride = struct('mapNo',13,'nRuns',20);
run_hfto_mc                            % our method, Map 2, 20 runs
cfgOverride = struct('mapNo',13,'nRuns',20);
run_mpcgraph_mc                         % MPC-Graph on the same runs
summarize_results(13)                   % side-by-side table
```

The first call plans the nominal trajectory (about 25 s for Map 2 with the
Fossen model) and saves it in `results/`; later runs reuse it. All settings
are in `config/default_config.m` and can be overridden by defining a struct
`cfgOverride` before running an experiment script.

## Reproducing the paper

| Paper | Script | Output in `results/` |
|---|---|---|
| Monte Carlo experiments, Table 4, Figs. 7–9 | `run_monte_carlo` with `batch.cases = {'baseline'}; batch.est = false;` | `hfto_mce_map<N>_<model>.mat`, `mg_mce_*.mat` |
| Robustness (parameters, currents, current estimate), Table 5 | `run_monte_carlo` | `*_param*.mat`, `*_current*.mat`, `*_est*.mat` |
| MPC-Graph heading weight | `tune_mpcgraph_heading` | `tune_mpcgraph_heading.mat` |
| Safety certificate, Table 6, Fig. 10 | `run_certificate` | `optionA_map<N>_<model>.mat` |
| Gazebo experiment, Fig. 6 | `run_hfto_gazebo`, `run_mpcgraph_gazebo` (see below) | `*_gazebo_*.mat` |
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

## Gazebo

The Heron simulator runs headless in Docker (ROS Melodic, Gazebo 9); MATLAB on
the host connects to it through ROS. See [`gazebo/README.md`](gazebo/README.md).

```bash
cd gazebo
./gz.sh build      # once
./gz.sh up         # start the simulator
```

```matlab
run_hfto_gazebo        % or run_mpcgraph_gazebo; ./gz.sh restart between runs
```

Run the Gazebo experiments on an otherwise idle machine: the controller runs
in real time at 10 Hz.

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
