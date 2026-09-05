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
    fullfile('experiment_outcome_summary','hard_violation_counts.csv')
    fullfile('cluster_statistics_output','environment_level_summary.csv')
    fullfile('spatial_resolution_output','spatial_resolution_case_results.csv')
    fullfile('fixed_path_weight_sensitivity_results','fixed_path_weight_summary.csv')
    fullfile('manuscript_completed_results','planner_runtime_summary.csv')
    fullfile('manuscript_completed_results','planner_outcome_counts.csv')};

missing = requiredFiles(~cellfun(@(f) isfile(fullfile(root,f)),requiredFiles));
assert(isempty(missing),'Missing required package file(s): %s', ...
    strjoin(missing,', '));
fprintf('Released source tree and readable result summaries: OK\n');

caseFile = fullfile(root,'experiment_outcome_summary','case_level_evaluator_outputs.csv');
caseT = readtable(caseFile,'TextType','string');
caseVars = {'Algorithm','EnvironmentSeed','AlgorithmSeed','UniqueTrial', ...
    'ReachedGoal','GenerationSuccess','FailureReason','J','EnergyWh','TimeS', ...
    'DynamicRisk','Pheight','Pstatic','PsceneEntry','PNFZ','Pbattery', ...
    'Pkinematic','Feasible','EvaluationStatus','NumericalFailure'};
assert(all(ismember(caseVars,caseT.Properties.VariableNames)), ...
    'Case-level CSV is missing required semantic fields.');
assert(~ismember('PlannerSuccess',caseT.Properties.VariableNames) && ...
       ~ismember('Pdynamic',caseT.Properties.VariableNames), ...
    'Case-level CSV retains deprecated field names.');
assert(all(logical(caseT.UniqueTrial)), ...
    'Human-readable case output must omit non-unique deterministic copies.');
generated = logical(caseT.GenerationSuccess);
reached = logical(caseT.ReachedGoal);
feasibleCsv = logical(caseT.Feasible);
status = string(caseT.EvaluationStatus);
assert(isequal(generated,reached), ...
    'GenerationSuccess and ReachedGoal must agree for the released cohort.');
assert(all(~feasibleCsv | (generated & status=="evaluated")), ...
    'A feasible row must be a generated and numerically evaluated path.');
noPath = ~generated;
assert(all(status(noPath)=="no_path"), ...
    'Generation failures must have EvaluationStatus=no_path.');
assert(all(~isfinite(caseT.J(noPath)) & ~isfinite(caseT.EnergyWh(noPath))), ...
    'No-path failures must not contain finite score or energy values.');

hardFile = fullfile(root,'experiment_outcome_summary','hard_violation_counts.csv');
hardT = readtable(hardFile,'TextType','string');
hardVars = {'Algorithm','N','Height','Static','SceneEntry','ActiveNFZ', ...
    'Battery','Kinematic','Multiple','Any','NumericalFailure','NoPath','AnyPhysical'};
assert(all(ismember(hardVars,hardT.Properties.VariableNames)) && ...
       ~ismember('Dynamic',hardT.Properties.VariableNames), ...
    'Hard-violation CSV fields are not synchronized with manuscript terminology.');
positiveComponents = (caseT.Pheight>0)+(caseT.Pstatic>0)+ ...
    (caseT.PsceneEntry>0)+(caseT.PNFZ>0)+(caseT.Pbattery>0)+ ...
    (caseT.Pkinematic>0);
for a = 1:height(hardT)
    use = caseT.Algorithm==hardT.Algorithm(a);
    penaltyFields = {'Pheight','Pstatic','PsceneEntry','PNFZ','Pbattery','Pkinematic'};
    countFields = {'Height','Static','SceneEntry','ActiveNFZ','Battery','Kinematic'};
    for k = 1:numel(penaltyFields)
        assert(sum(caseT.(penaltyFields{k})(use)>0)==hardT.(countFields{k})(a), ...
            'Violation counts must include every strictly positive penalty.');
    end
    assert(sum(use)==hardT.N(a) && ...
           sum(positiveComponents(use)>0)==hardT.AnyPhysical(a), ...
        'Unique-trial and physical-violation totals disagree with case outputs.');
    assert(sum(positiveComponents(use)>1)==hardT.Multiple(a), ...
        'Multiple must count trials with more than one positive hard-penalty component.');
end

outcomeFile = fullfile(root,'manuscript_completed_results','planner_outcome_counts.csv');
outcomeT = readtable(outcomeFile,'TextType','string');
outcomeVars = {'scene_entry_violations','kinematic_violations', ...
    'multiple_positive_hard_penalty_components'};
assert(all(ismember(outcomeVars,outcomeT.Properties.VariableNames)) && ...
       ~ismember('dynamic_violations',outcomeT.Properties.VariableNames), ...
    'Planner-outcome CSV fields are not synchronized with manuscript terminology.');
fprintf('Human-readable output semantics: OK\n');

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
testResults = runtests({'testTrackHoldingKinematics.m','testArrivalTimeSolver.m', ...
    'testValidationInterfaces.m'});
assert(all([testResults.Passed]),'Wind/time/kinematics unit tests failed.');

fprintf('Released archived cohort: OK\n');
runClusterAwareStatistics(archive);
plotStatisticalFiguresFromArchive;
plotAblationFigureFromArchive;
fprintf('RA-ALA reproducibility-package verification completed successfully.\n');
end
