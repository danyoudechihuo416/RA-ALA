# Independent 20-Run Optimizer Benchmark

This directory preserves the script, CSV output, and convergence figures used
for the independent complete-configuration optimizer benchmark.

It is intentionally separate from the environment-level planner cohort and is
not called by `runAllValidationExperiments`.

Files:

- `metaheuristic_benchmark.m`: archived benchmark driver.
- `evalRA_v2.m`: archived benchmark evaluator wrapper.
- `metaheuristic_benchmark.csv`: released 20-run summary.
- `fig_convergence_comparison.png`: best-so-far iteration traces.
- `fig_convergence_NFE.png`: evaluator-call convergence traces.

The archived script records the benchmark configuration that produced these
artifacts. It is provided for provenance and should not be mixed silently with
a later evaluator or environment implementation. A fresh benchmark under
changed source must be reported as a new experiment.
