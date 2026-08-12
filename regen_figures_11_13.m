% Regenerate manuscript Figures 11--13 from saved experiment results only.
close all;

S = load('section55_same_cohort_data.mat');
plotDistributionalRobustness(S.stat_J,S.stat_E,S.stat_feasible,S.stat_env, ...
    S.env_seeds_used,S.algNames,'fig7_distributional_robustness.png');

clusterStats = runClusterAwareStatistics('section55_same_cohort_data.mat');
plotClusterAwareStatistics(clusterStats,'fig8_statistical_significance.png');

A = load('fig9_ablation_same_cohort_data.mat');
plotAblationStudyFigure(A.abl_J,A.abl_E,A.abl_T,A.abl_R,A.abl_Pen, ...
    A.ablationNames,A.N_ABL,'fig9_ablation_study.png');

close all;
fprintf('Figures 11--13 regenerated from saved results.\n');
