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
    'computeTrackHoldingKinematics.m'
    'solveArrivalTimeStep.m'
    'testArrivalTimeSolver.m'
    'testTrackHoldingKinematics.m'
    'resolveCohortEvaluationSettings.m'
    'validationProvenance.m'
    'testValidationInterfaces.m'
    'runRA_ALA.m'
    'evaluateRAALASearchFitness.m'
    'renderDepartureTimeAdaptation.m'
    'departure_time_case_data.mat'
    'main_experiment_cohort.mat'
    'ablation_same_cohort_results.mat'
    fullfile('experiment_outcome_summary','case_level_evaluator_outputs.csv')
    fullfile('cluster_statistics_output','environment_level_summary.csv')
    fullfile('spatial_resolution_output','spatial_resolution_case_results.csv')
    fullfile('fixed_path_weight_sensitivity_results','fixed_path_weight_summary.csv')
    fullfile('manuscript_completed_results','planner_runtime_summary.csv')};

missing = requiredFiles(~cellfun(@(f) isfile(fullfile(root,f)),requiredFiles));
assert(isempty(missing),'Missing required package file(s): %s', ...
    strjoin(missing,', '));
fprintf('Released source tree and readable result summaries: OK\n');

archive = 'main_experiment_cohort.mat';
assert(isfile(archive), 'Missing released cohort archive: %s', archive);
S = load(archive);
resolveCohortEvaluationSettings(S);
requiredVars = {'env_seeds_used','stat_env','stat_ra_seed','stat_J', ...
    'stat_E','stat_P','stat_feasible','stat_paths','algNames', ...
    'stat_search_J','stat_search_E','stat_search_P','stat_algorithm_seed', ...
    'stat_planner_success','stat_reached_goal','stat_failure_reason', ...
    'stat_wait_schedules','stat_is_unique_trial', ...
    'stat_search_evaluation_details','stat_final_evaluation_details', ...
    'planning_collision_sample_spacing_m','final_verification_spacing_m', ...
    'budget_interpretation','N_ENV','N_SEED','N_STAT'};
missingVars = requiredVars(~isfield(S,requiredVars));
assert(isempty(missingVars),'Missing cohort variable(s): %s', ...
    strjoin(missingVars,', '));
expectedSeeds = [483,638,855,948,1041,1103,1227,1475,2312,2560];
assert(isequal(S.env_seeds_used(:)',expectedSeeds), ...
    'The archived environment cohort does not match the documented seeds.');
assert(S.N_ENV==10 && S.N_SEED==3 && S.N_STAT==30, ...
    'Expected a 10-environment x 3-run cohort.');
assert(abs(S.planning_collision_sample_spacing_m-0.75)<eps && ...
       abs(S.final_verification_spacing_m-0.75)<eps, ...
    'Expected 0.75 m planning and final-evaluation spacing.');
assert(isequal(size(S.stat_is_unique_trial),size(S.stat_feasible)), ...
    'Unique-trial mask size does not match the feasibility matrix.');
expectedUnique = [30;10;30;10;10];
assert(isequal(sum(logical(S.stat_is_unique_trial),2),expectedUnique), ...
    'Unexpected numbers of independent trials by planner.');
assert(all(all(isnan(S.stat_algorithm_seed([2,4,5],:)))), ...
    'Deterministic planners must not be assigned artificial random seeds.');
assert(contains(S.budget_interpretation,'method-specific'), ...
    'Budget interpretation must state the method-specific comparison scope.');
testResults = runtests({'testTrackHoldingKinematics.m','testArrivalTimeSolver.m'});
assert(all([testResults.Passed]),'Wind/time/kinematics unit tests failed.');

fprintf('Released archived cohort: OK\n');
runClusterAwareStatistics(archive);
plotStatisticalFiguresFromArchive;
plotAblationFigureFromArchive;
fprintf('RA-ALA reproducibility-package verification completed successfully.\n');
end
