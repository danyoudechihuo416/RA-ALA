# MATLAB Result Archives

The released MAT files preserve simulation outputs, paths, seeds, and analysis
structures. CSV/TXT counterparts are provided where practical.

| Archive | Bytes | SHA-256 |
|---|---:|---|
| `main_experiment_cohort.mat` | 7,176,112 | `235c0aa381939acbb4013e85c5810b10270be316acaed7cf55f57472c8cc6ff4` |
| `departure_time_case_data.mat` | 33,299 | `56bde795e4d012d7042605a14154710f415933c85269be7878b8d3ece2c698b7` |
| `representative_path_case_data.mat` | 12,347 | `2666ce31083bfab0827f32ced6e19e0f099f83bcd7771ea2162bfc6be0a54665` |
| `ablation_same_cohort_results.mat` | 148,046 | `cfe8d0f33fd73614c963ac47f1a8398095e632371bb0b9144771bb447cff6bc2` |
| `validation_suite_results.mat` | 50,182 | `79ab7c4607c42c10fa06d715c535338ce0dfbfb411e0e6b4af487b79525898e2` |
| `cluster_statistics_output/cluster_aware_statistics.mat` | 7,185 | `be7261f0ce870a8c1dab3cbdb8eb063d91dfb37776cc5e832e092a0401732abb` |
| `spatial_resolution_output/spatial_resolution_results.mat` | 31,131 | `28f950b38e0a8c8d26a7726f26bbd1328dc5ce2745866543ec8212f3e8c99852` |
| `fixed_path_weight_sensitivity_results/fixed_path_weight_sensitivity_results.mat` | 20,137 | `00ce37d10e9e2a3b2225d6dc198d8f802f23f8803f1b158e2a7b8bf27b46a3a6` |
| `experiment_outcome_summary/experiment_outcome_summary.mat` | 7,460 | `5fddf2b87ba86a29086c96730763291aa01bdefd14f656ac3730fe1498373c34` |

The main cohort is the authoritative 0.75 m planning/final-evaluation archive.
It preserves the original source fingerprint and the transparent v1.1.0
comment/encoding metadata refresh record.

Verify on Windows:

```powershell
Get-FileHash -Algorithm SHA256 main_experiment_cohort.mat
```

Inspect in MATLAB:

```matlab
whos('-file','main_experiment_cohort.mat')
```
