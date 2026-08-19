# MATLAB Result Archives

The released `.mat` files preserve MATLAB structures and saved paths used by
the manuscript analyses. They contain simulation outputs rather than source
code. Human-readable CSV/TXT counterparts are provided in the same result
directories where applicable.

| Archive | Purpose | Bytes | SHA-256 |
|---|---|---:|---|
| `main_experiment_cohort.mat` | Fixed 10-environment, 3-run main cohort, including paths, seeds, metrics, and feasibility labels | 52,508 | `372ad810a0f08f86a830885c13a7dbc03d1db08a505dc650378c847436767819` |
| `departure_time_case_data.mat` | Five predefined departure-time paths, evaluator decompositions, seeds, and figure metadata for the illustrative case study | 25,692 | `054e3c53998e795c7f3454287fdfa8da93d526388e7b45e1b3d3406c659b46da` |
| `representative_path_case_data.mat` | Predefined seed-42 High-complexity paths used consistently in the 2D and 3D representative comparison panels | 12,347 | `2666ce31083bfab0827f32ced6e19e0f099f83bcd7771ea2162bfc6be0a54665` |
| `ablation_same_cohort_results.mat` | Same-cohort ablation outputs used for Figure 9 and its table | 6,753 | `405fb18e442d2d217b79188c0ea962fd01afba0346718f61fa24d27c72e4829d` |
| `validation_suite_results.mat` | Master index of validation analyses | 46,431 | `715f78721c3e999abae1e2c549a0bb8688d05b3eae7e8783d5bd85211fa902ac` |
| `cluster_statistics_output/cluster_aware_statistics.mat` | Environment-level clustered statistical results | 7,689 | `0e013f82b32560d8ef87c821cb48a39d8be850793d2c8a277ada0cde10adc776` |
| `resolution_selection_output/targeted_resolution_results.mat` | Targeted 1.5 m versus 0.75 m resolution audit | 7,654 | `af51b6ad468de6df1fcd8dee24ca26be9cb43dadd23615377d927ab2c60c893e` |
| `computational_budget_output/computational_budget_results.mat` | Planner budget, evaluator-call, runtime, and hardware audit | 10,835 | `3493eb9fd661570c6c6939848b4d7261a3f68a652844e962b805598d30d9ed6a` |
| `experiment_outcome_summary/experiment_outcome_summary.mat` | Feasibility and jointly feasible outcome summaries | 6,267 | `994769fd1f211c18c9e784ce673873532ee235098fed59c76687a3242fb2eb70` |
| `spatial_resolution_output/spatial_resolution_results.mat` | Full spatial-resolution sensitivity results | 31,099 | `56416e3899313eca5fee2fdc026d6436a6fddb711ea0fb607c0e75e928dbbe35` |
| `weight_sensitivity_results/weight_sensitivity_results.mat` | Cost-weight sensitivity outputs and rank-stability results | 26,238 | `1fb8e34a9bfd81cb731c7e63662c52a0678a3d0db0817cc4a8566f8e43b099b4` |

To verify an archive on Windows PowerShell:

```powershell
Get-FileHash -Algorithm SHA256 main_experiment_cohort.mat
```

To inspect variables in MATLAB:

```matlab
whos('-file', 'main_experiment_cohort.mat')
```
