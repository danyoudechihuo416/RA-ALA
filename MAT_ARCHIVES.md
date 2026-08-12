# MATLAB Result Archives

The released `.mat` files preserve MATLAB structures and saved paths used by
the manuscript analyses. They contain simulation outputs rather than source
code. Human-readable CSV/TXT counterparts are provided in the same result
directories where applicable.

| Archive | Purpose | Bytes | SHA-256 |
|---|---|---:|---|
| `section55_same_cohort_data.mat` | Fixed 10-environment, 3-run main cohort, including paths, seeds, metrics, and feasibility labels | 52,508 | `372ad810a0f08f86a830885c13a7dbc03d1db08a505dc650378c847436767819` |
| `fig9_ablation_same_cohort_data.mat` | Same-cohort ablation outputs used for Figure 9 and its table | 6,753 | `405fb18e442d2d217b79188c0ea962fd01afba0346718f61fa24d27c72e4829d` |
| `reviewer_revision_master_results.mat` | Master index of reviewer-revision analyses | 46,431 | `715f78721c3e999abae1e2c549a0bb8688d05b3eae7e8783d5bd85211fa902ac` |
| `cluster_statistics_output/cluster_aware_statistics.mat` | Environment-level clustered statistical results | 7,689 | `0e013f82b32560d8ef87c821cb48a39d8be850793d2c8a277ada0cde10adc776` |
| `resolution_selection_output/targeted_resolution_results.mat` | Targeted 1.5 m versus 0.75 m resolution audit | 7,654 | `af51b6ad468de6df1fcd8dee24ca26be9cb43dadd23615377d927ab2c60c893e` |
| `reviewer_budget_output/reviewer_budget_results.mat` | Planner budget, evaluator-call, runtime, and hardware audit | 10,845 | `f7d5d4c74219258788b7875afcb9e04366f0559cbd69f5e5d2c2828aec768cd1` |
| `reviewer_outcome_summary/reviewer_outcome_summary.mat` | Feasibility and jointly feasible outcome summaries | 6,267 | `994769fd1f211c18c9e784ce673873532ee235098fed59c76687a3242fb2eb70` |
| `spatial_resolution_output/spatial_resolution_results.mat` | Full spatial-resolution sensitivity results | 31,099 | `56416e3899313eca5fee2fdc026d6436a6fddb711ea0fb607c0e75e928dbbe35` |
| `weight_sensitivity_results/weight_sensitivity_results.mat` | Cost-weight sensitivity outputs and rank-stability results | 26,238 | `1fb8e34a9bfd81cb731c7e63662c52a0678a3d0db0817cc4a8566f8e43b099b4` |

To verify an archive on Windows PowerShell:

```powershell
Get-FileHash -Algorithm SHA256 section55_same_cohort_data.mat
```

To inspect variables in MATLAB:

```matlab
whos('-file', 'section55_same_cohort_data.mat')
```
