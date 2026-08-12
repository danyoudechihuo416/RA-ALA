%% Regenerate Figure 9 from the completed four-variant ablation data.
data_file = 'fig9_ablation_same_cohort_data.mat';
if ~isfile(data_file)
    error('Missing %s. Run run_headwind_ablation_incremental first.', data_file);
end
load(data_file, 'abl_J','abl_E','abl_T','abl_R','abl_Pen', ...
    'abl_feasible','ablationNames','N_ABL');
nAbl = numel(ablationNames);
if nAbl ~= 4 || size(abl_J,1) ~= 4
    error('Expected four valid variants in %s.', data_file);
end
fprintf('Loaded four valid variants from %s.\n', data_file);

abl_colors = [0.82 0.10 0.10;
              0.25 0.45 0.78;
              0.18 0.65 0.32;
              0.50 0.50 0.50];
plotAblationStudyFigure(abl_J, abl_E, abl_T, abl_R, abl_Pen, ...
    ablationNames, N_ABL, 'fig9_ablation_study.png');
fprintf('Figure 9 regenerated as fig9_ablation_study.png and vector PDF.\n');
