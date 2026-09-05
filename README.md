# RA-ALA: Reproducible UAV Path-Planning Experiments

Repository: <https://github.com/danyoudechihuo416/RA-ALA>

Versioned release: <https://github.com/danyoudechihuo416/RA-ALA/tree/v1.1.1>

This repository contains the MATLAB implementation and released analysis
artifacts for the RA-ALA manuscript.

RA-ALA combines an Energy-A* warm start, continuous waypoint search, Top-K
candidate re-evaluation, feasibility-first selection, and conditional local
recovery under a shared execution-level evaluator.

The released implementation does not apply a separate WindBias walk
displacement. Wind remains active in the environment, arrival-time solution,
propulsion model, and guidance.

## Scope

Reported feasibility is limited to the sampled simulation evaluator. The code
does not model closed-loop control, sensing, or tracking error and is not
flight-control software.

## Requirements

- MATLAB R2024b
- Image Processing Toolbox
- Statistics and Machine Learning Toolbox
- Sufficient memory for 1000 x 1000 maps

Released runs used Windows, an AMD Ryzen 7 9800X3D CPU, 31.1 GiB RAM, and no
active parallel pool.

## Quick Verification

```matlab
cd('path/to/RA-ALA');
addpath(genpath(pwd));
verify_reproducibility_package;
```

This checks the 0.75 m archive, runs targeted tests, refreshes statistics, and
redraws Figures 7-9. It does not rerun the hours-long planners.

## Entry Points

- `runAllValidationExperiments`: complete ordered suite; reruns planners.
- `runFigure7And8Experiment`: authoritative 0.75 m cohort only; reruns planners.
- `runRemainingValidationExperiments`: post-cohort analyses; no planner rerun
  by default.
- `runClusterAwareStatistics`: environment-level inference from the archive.
- `runFixedPathWeightSensitivityAnalysis`: rescore archived physical outputs.
- `runSpatialResolutionSensitivity`: fixed-path spacing audit by default.
- `runWeightSensitivityAnalysis`: 11-setting RA-ALA re-optimization audit.
- `runPlanningResolutionPilot`: small paired 1.5/0.75 m planning pilot.
- `extractCompletedManuscriptResults`: export outcome, failure, runtime, and
  evaluator-call summaries without replanning.
- `runComputationalBudgetAnalysis`: independent fresh budget audit; reruns
  planners and is not required to reproduce the archived runtime table.

The fixed-path weight analysis is the inexpensive manuscript sensitivity
analysis. `runWeightSensitivityAnalysis` instead performs new RA-ALA searches
and is a within-method audit, not a planner comparison.

## Reproduce Analyses without Replanning

```matlab
cohort = 'main_experiment_cohort.mat';
runClusterAwareStatistics(cohort);
runFixedPathWeightSensitivityAnalysis(cohort);
runSpatialResolutionSensitivity(cohort);
summarizeExperimentOutcomes(cohort,'experiment_outcome_summary');
extractCompletedManuscriptResults(cohort,'manuscript_completed_results');
```

These commands preserve archived paths and failures. The resolution audit uses
6, 3, 1.5, 0.75, and 0.375 m evaluator spacings and does not feed the finer
audit back into selection, re-optimization, or Rescue.

The post-cohort wrapper runs fixed-path weights, resolution, and summaries:

```matlab
results = runRemainingValidationExperiments();
```

Its budget stage is disabled by default because that stage calls planners.
Use `struct('CheckOnly',true)` to validate inputs without writing outputs.

## Primary Figure 7-8 Experiment

```matlab
results = runFigure7And8Experiment();
```

It checkpoints every unique planner run. On completion, the authoritative
cohort and Figure 7-8 PNG/PDF files are synchronized to the repository root.

## Complete Suite

```matlab
results = runAllValidationExperiments();
```

The order is Figures 1-6, the 0.75 m Figure 7-8 cohort, same-cohort ablation,
clustered inference, fixed-path weights, fixed-path resolution, outcome
summaries, and a fresh budget audit.

The full default is expensive. To omit the duplicate fresh budget audit:

```matlab
results = runAllValidationExperiments(struct('RunBudget',false));
```

To reuse the release archive, set `RunFigure78=false`, `RunMain=false`, and
`CohortFile='main_experiment_cohort.mat'`, then enable only the desired
post-processing switches.

Individual illustrative modes in `runMainExperiments.m` are:

- `departure-time-only` and `replot-departure-time`
- `path-comparison-only` and `replot-path-comparison`
- `pre-cohort` and `post-cohort` for the full orchestrator

The two `replot-*` modes use saved MAT data and do not call planners.

## Experimental Design

The authoritative cohort uses environment seeds 483, 638, 855, 948, 1041,
1103, 1227, 1475, 2312, and 2560. RA-ALA and Informed-RRT* use three stochastic
runs per environment. Energy-A*, ST-EA*, and Greedy use one unique deterministic
run per environment.

Repeated plotting slots for deterministic methods are marked non-unique and
excluded from inferential sample counts. The environment is the inferential
unit. All comparisons use prespecified method-specific algorithm budgets; they
do not claim equal evaluator-call or operator-level budgets.

Primary cohort planning and final evaluation both use 0.75 m spacing. The
0.375 m fixed-path audit is not a 0.375 m planning-convergence claim.

## Main Source Files

- `CityEnvironment.m`: buildings, bounded wind, moving obstacles, and NFZs.
- `UnifiedCostModel.m`: execution-level evaluator.
- `solveArrivalTimeStep.m`: implicit arrival-time solution and recovery.
- `computeTrackHoldingKinematics.m`: prescribed-airspeed track holding.
- `PathPlanners.m`: baseline planners.
- `runRA_ALA.m`: RA-ALA search, Top-K selection, and recovery.

## Released Results

- `main_experiment_cohort.mat`: authoritative 0.75/0.75 m cohort.
- `cluster_statistics_output/`: environment-level paired inference.
- `fixed_path_weight_sensitivity_results/`: 11-setting score sensitivity.
- `spatial_resolution_output/`: fixed-path spacing audit.
- `experiment_outcome_summary/`: case outputs and violation counts.
- `manuscript_completed_results/`: outcome, failure, runtime, and call summaries.
- `ablation_same_cohort_results.mat`: paired component ablation.
- `optimizer_benchmark_archive/`: source, CSV, and figures for the independent
  20-run optimizer benchmark.

The manuscript runtime table is extracted from the authoritative cohort.
`computational_budget_output/` is a separate fresh-run audit.

### Human-readable output semantics

`experiment_outcome_summary/case_level_evaluator_outputs.csv` contains one row
per unique planner trial; repeated deterministic plotting copies are omitted.
Its status fields have the following meanings:

- `UniqueTrial`: the row is an independent stochastic run or the single unique
  deterministic run for that environment.
- `ReachedGoal`: the planner returned a finite three-dimensional path ending at
  the prescribed goal.
- `GenerationSuccess`: the planner generated such a goal-reaching path. In the
  released cohort this is equivalent to `ReachedGoal`; it is distinct from
  evaluator feasibility.
- `EvaluationStatus`: `evaluated` for a numerically evaluated path, `no_path`
  when generation failed, or `time_solver_failure` when arrival-time propagation
  did not produce a finite executable evaluation.
- `Feasible`: the evaluated path has no positive physical hard-penalty component.
  A generation or numerical failure is not feasible.

`PsceneEntry` is the sampled scene-entry component returned by the common
collision query and can be triggered by a moving obstacle or an active temporary
NFZ. `PNFZ` separately measures active-NFZ penetration, so one NFZ event can make
both components positive. `Multiple` counts trials with more than one positive
component among `Pheight`, `Pstatic`, `PsceneEntry`, `PNFZ`, `Pbattery`, and
`Pkinematic`; it does not imply independent physical hazards. No-path generation
failures have no finite score or energy and are reported through status and
failure counts rather than included in continuous-outcome distributions.

`weight_sensitivity_results/` and `resolution_selection_output/` are retained
as earlier auxiliary audits. Current manuscript-facing outputs use the
`fixed_path_weight_sensitivity_results/` and `spatial_resolution_output/`
directories. The optimizer benchmark is archived separately and is not invoked
by `runAllValidationExperiments`.

## Evaluator Notes

The medium wind preset uses a 3 m/s reference wind with bounded modulation and
canyon amplification. These are prescribed synthetic settings, not a calibrated
weather distribution or vehicle safety limit.

Arrival time is solved consistently for wind, energy, moving obstacles, and
active NFZ checks. Unresolved equations are numerical failures, not finite
executable routes.

Strict feasibility and all violation summaries use a strict component-wise
zero test: any positive physical hard-penalty component counts as a violation.
Source fingerprints include the arrival-
time solver, and incompatible checkpoints are rejected. Restart MATLAB after
changing class files before beginning a fresh experiment.

Archive sizes and SHA-256 checksums are listed in `MAT_ARCHIVES.md`.

Targeted tests:

```matlab
r = runtests({'testTrackHoldingKinematics.m', ...
    'testArrivalTimeSolver.m','testValidationInterfaces.m'});
assert(all([r.Passed]));
```

## Citation and License

Use `CITATION.cff` for software citation metadata. The source is released
under the MIT License; see `LICENSE`.
