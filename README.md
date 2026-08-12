# RA-ALA: Reproducible UAV Path-Planning Experiments

Repository: <https://github.com/danyoudechihuo416/RA-ALA>

This repository contains the MATLAB implementation and reproducibility
artifacts for the manuscript **"Unified-Evaluation-Driven RA-ALA for
Three-Dimensional UAV Path Planning in Time-Varying Urban Low-Altitude
Environments"**.

RA-ALA is a simulation-level hybrid path-generation pipeline. It combines an
Energy-A* warm start, continuous waypoint search, Top-K path-variant
re-evaluation, feasibility-first selection, and conditional recovery under a
shared arrival-time-recursive evaluator. The released version is the final
**no-WindBias** implementation used for the revised experiments: the wind
field, wind-dependent propulsion model, and headwind look-ahead term remain
active, while the wind-biased ALA walk displacement is disabled.

## Scope

The software evaluates routes under the modeled building-clearance, moving-
obstacle, active no-fly-zone, wind, propulsion-energy, altitude, and battery
constraints. A route reported as feasible is therefore
**simulation-/evaluator-feasible**. The code does not model closed-loop vehicle
dynamics, state estimation, sensing uncertainty, tracking error, or
communication effects, and it is not flight-control software.

## Requirements

- MATLAB R2024b (the archived experiments used 24.2.0.2712019)
- Image Processing Toolbox (`imgaussfilt` in the environment generator)
- Statistics and Machine Learning Toolbox (box plots and normal-CDF fallback)
- A machine with sufficient memory for the 1000 x 1000 environment maps

The archived runs used Windows, a 16-logical-processor AMD64 CPU, 31.1 GB RAM,
and no active parallel pool. Exact hardware metadata is provided in
`reviewer_budget_output/reviewer_hardware.csv`.

## Quick Start

From MATLAB:

```matlab
cd('path/to/RA-ALA');
addpath(genpath(pwd));
verify_reproducibility_package;
```

The verification script checks the released source tree and human-readable result
files. If the optional `section55_same_cohort_data.mat` archive is present, it
also verifies the fixed cohort, re-runs the environment-level statistics, and
regenerates the manuscript figures. It never re-runs the hours-long planners.

For an interactive single-environment demonstration:

```matlab
RA_ALA_demo
```

This script also contains the full experiment sequence and can take several
hours, depending on the machine.

## Reproducing the Revised Analyses

### From optional archived planner outputs

Place `section55_same_cohort_data.mat` in the repository root, then run:

```matlab
runClusterAwareStatistics('section55_same_cohort_data.mat');
runSpatialResolutionSensitivity('section55_same_cohort_data.mat');
runWeightSensitivityAnalysis('section55_same_cohort_data.mat');
summarizeReviewerOutcomes;
regen_figures_11_13;
regen_fig9;
```

`runSpatialResolutionSensitivity` re-evaluates the saved paths and may take
longer than the other post-processing steps.

### Full experiment suite

```matlab
results = runAllReviewerRevisionExperiments();
```

To reuse the released main cohort and skip the longest stage:

```matlab
opts = struct('RunMain', false);
results = runAllReviewerRevisionExperiments(opts);
```

The fixed High-complexity environment seeds are:

```text
483, 638, 855, 948, 1041, 1103, 1227, 1475, 2312, 2560
```

Each environment has three pre-specified algorithmic runs. The environment is
the inferential unit (`N = 10`); the three runs are aggregated within each
environment for cluster-aware inference.

## Main Files

- `RA_ALA_demo.m`: manuscript experiment driver and fixed cohort definition.
- `runRA_ALA.m`: complete RA-ALA pipeline with timing and recovery counters.
- `UnifiedCostModel.m`: shared arrival-time-recursive route evaluator.
- `CityEnvironment.m`: seeded urban environment, wind, obstacle, and NFZ
  generation.
- `PathPlanners.m`: Energy-A*, Informed-RRT*, ST-EA*, and Greedy baselines.
- `evalRA_v2.m`: bounded search-stage guidance used by RA-ALA.
- `runClusterAwareStatistics.m`: environment-level paired inference.
- `runSpatialResolutionSensitivity.m`: 12/6/3/1.5/0.75 m resolution analysis.
- `runWeightSensitivityAnalysis.m`: objective-weight sensitivity analysis.
- `runReviewerBudgetAnalysis.m`: evaluator-call, runtime, and planner-permission
  audit.
- `exportDepartureTimeLatency.m`: stage-level latency and Rescue accounting.

## Released Data and Results

- `section55_same_cohort_data.mat`: complete paired main-experiment cohort,
  including paths, seeds, metrics, and feasibility labels.
- `reviewer_outcome_summary/case_level_evaluator_outputs.csv`: case-level
  decomposed evaluator outputs.
- `cluster_statistics_output/`: environment-level summaries, confidence
  intervals, omnibus tests, adjusted pairwise tests, and paired effect sizes.
- `spatial_resolution_output/`: spatial-resolution case results and stability
  summaries.
- `resolution_selection_output/`: targeted 1.5 m versus 0.75 m audit.
- `weight_sensitivity_results/`: weight configurations, case-level outcomes,
  summaries, and rank stability.
- `reviewer_budget_output/`: algorithm permissions, evaluator calls, runtime,
  failure rules, and hardware metadata.
- `departure_time_latency_details.csv`: optimization, Top-K, smoothing,
  RescueA, and RescueB timing/counts for the departure-time cases.

CSV files are included to permit inspection without MATLAB. MAT files preserve
the corresponding MATLAB structures and saved paths.

## Reproducibility Notes

- Main collision and constraint checks use 1.5 m spatial sampling with at least
  three samples per segment.
- Strict feasibility requires zero physical hard-constraint violations; a
  tolerance of `1e-12` is used only to recognize floating-point zero.
- The 30 runs are not treated as 30 independent environments.
- Runtime values are hardware dependent. Statistical and physical outcomes are
  deterministic for the released seeds, subject to MATLAB-version numerical
  differences.

## Citation

Please cite the associated article after publication. A placeholder citation is
provided in `CITATION.cff` and should be updated with the final DOI and
publication metadata.

## License

The source code is released under the MIT License. See `LICENSE`.

