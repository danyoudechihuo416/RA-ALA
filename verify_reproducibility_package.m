function verify_reproducibility_package()
%VERIFY_REPRODUCIBILITY_PACKAGE Fast source and optional archive check.
% This check does not rerun the hours-long planner experiments.

root = fileparts(mfilename('fullpath'));
oldDir = pwd;
cleanup = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(root);
addpath(genpath(root));

requiredFiles = {
    'CityEnvironment.m'
    'UnifiedCostModel.m'
    'PathPlanners.m'
    'runRA_ALA.m'
    'evalRA_v2.m'
    fullfile('reviewer_outcome_summary','case_level_evaluator_outputs.csv')
    fullfile('cluster_statistics_output','environment_level_summary.csv')
    fullfile('spatial_resolution_output','spatial_resolution_case_results.csv')
    fullfile('weight_sensitivity_results','weight_sensitivity_summary.csv')
    fullfile('reviewer_budget_output','reviewer_runtime_summary.csv')};

missing = requiredFiles(~cellfun(@(f) isfile(fullfile(root,f)),requiredFiles));
assert(isempty(missing),'Missing required package file(s): %s', ...
    strjoin(missing,', '));
fprintf('Released source tree and readable result summaries: OK\n');

archive = 'section55_same_cohort_data.mat';
if ~isfile(archive)
    fprintf(['Optional cohort archive not found; archived-output statistics ', ...
        'and figure regeneration were skipped.\n']);
    fprintf('RA-ALA source-package verification completed successfully.\n');
    return;
end

S = load(archive);
requiredVars = {'env_seeds_used','stat_env','stat_ra_seed','stat_J', ...
    'stat_E','stat_P','stat_feasible','stat_paths','algNames', ...
    'N_ENV','N_SEED','N_STAT'};
missingVars = requiredVars(~isfield(S,requiredVars));
assert(isempty(missingVars),'Missing cohort variable(s): %s', ...
    strjoin(missingVars,', '));
expectedSeeds = [483,638,855,948,1041,1103,1227,1475,2312,2560];
assert(isequal(S.env_seeds_used(:)',expectedSeeds), ...
    'The archived environment cohort does not match the documented seeds.');
assert(S.N_ENV==10 && S.N_SEED==3 && S.N_STAT==30, ...
    'Expected a 10-environment x 3-run cohort.');

fprintf('Optional archived cohort: OK\n');
runClusterAwareStatistics(archive);
regen_figures_11_13;
regen_fig9;
fprintf('RA-ALA archive-assisted verification completed successfully.\n');
end
