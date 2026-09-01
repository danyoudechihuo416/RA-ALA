function results = runAllValidationExperiments(userOpts)
%RUNALLVALIDATIONEXPERIMENTS Run the complete validation experiment suite.
%   results = runAllValidationExperiments();
%
%   Execution order:
%     1. Original Experiments 1-3 and Figures 1-6.
%     2. Authoritative Figure 7-8 cohort (planning and final evaluation at 0.75 m).
%     3. Ablation study reusing that cohort.
%     4. Environment-level inference on the authoritative cohort.
%     5. Fixed-path post hoc weight sensitivity without re-optimization.
%     6. Fixed-path audit, outcome summaries, and budget measurements.
%   CheckOnly=true validates inputs without starting experiments or writing outputs.
%
%   To reuse an existing authoritative cohort:
%     opts = struct('RunFigure78',false,'RunMain',false, ...
%         'CohortFile','full_path_to_cohort.mat');
%     results = runAllValidationExperiments(opts);

    if nargin < 1, userOpts = struct(); end
    opts = struct('RunFigure78',true,'RunMain',true, ...
        'RunClusterStats',true,'RunWeights',true, ...
        'RunResolution',true,'RunSummary',true,'RunBudget',true,'CheckOnly',false, ...
        'CohortFile','','Figure78Options',struct(), ...
        'ClusterOptions',struct(),'WeightOptions',struct(), ...
        'ResolutionOptions',struct(),'BudgetOptions',struct());
    names = fieldnames(userOpts);
    for i=1:numel(names)
        if ~isfield(opts,names{i}), error('ValidationSuite:UnknownOption','Unknown option: %s',names{i}); end
        opts.(names{i})=userOpts.(names{i});
    end
    switches = {'RunFigure78','RunMain','RunClusterStats','RunWeights','RunResolution','RunSummary','RunBudget','CheckOnly'};
    for i=1:numel(switches)
        validateattributes(opts.(switches{i}),{'logical','numeric'},{'scalar','binary'},mfilename,switches{i});
    end

    projectDir = fileparts(mfilename('fullpath'));
    oldDir = pwd;
    cleanupDir = onCleanup(@()cd(oldDir)); %#ok<NASGU>
    cd(projectDir);
    warningState = warning;
    cleanupWarning = onCleanup(@()warning(warningState)); %#ok<NASGU>
    defaultCohort = fullfile(projectDir,'main_experiment_cohort.mat');
    if isempty(opts.CohortFile)
        cohortFile = defaultCohort;
    else
        cohortFile = char(opts.CohortFile);
    end
    if ~opts.RunFigure78
        if ~isfile(cohortFile)
            error('ValidationSuite:MissingCohort', ...
                'The requested authoritative cohort does not exist: %s',cohortFile);
        end
        resolveCohortEvaluationSettings(load(cohortFile));
    end
    if opts.CheckOnly
        results = struct('preflight_passed',true,'options',opts,'main_completed',false);
        fprintf('Input checks passed. No experiments started and no outputs written.\n');
        return;
    end
    logFile = fullfile(projectDir,'validation_suite_run.log');
    diary(logFile);
    cleanupDiary = onCleanup(@()diary('off')); %#ok<NASGU>

    fprintf('\n============================================================\n');
    fprintf('Validation experiment suite started: %s\n', ...
        char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
    fprintf('Project: %s\n',projectDir);
    fprintf('============================================================\n');

    results = struct('figure78',[],'pre_cohort_completed',false, ...
        'main_completed',false,'cluster_statistics',[], ...
        'weights',[],'resolution',[],'summary_completed',false,'budget',[], ...
        'cohort_file',cohortFile,'options',opts,'log_file',logFile, ...
        'last_completed_stage','none');

    if opts.RunMain
        fprintf('\nMain pre-cohort stage: running Experiments 1-3 and Figures 1-6 ...\n');
        localRunMain(projectDir,'pre-cohort','');
        warning(warningState);
        results.pre_cohort_completed = true;
    else
        fprintf('\nMain pre-cohort stage skipped by user option.\n');
    end
    results.last_completed_stage = 'pre_cohort_experiments';
    localSaveProgress(projectDir,results,opts);

    if opts.RunFigure78
        fprintf('\nRunning/resuming the authoritative 0.75 m Figure 7-8 cohort ...\n');
        figureOpts = localSetDefault(opts.Figure78Options,'OutputDir', ...
            fullfile(projectDir,'validation_round_0p75m','figure7_8'));
        results.figure78 = runFigure7And8Experiment(figureOpts);
        localPublishFigureArtifacts(projectDir,results.figure78);
        cohortFile = results.figure78.cohort_file;
        results.cohort_file = cohortFile;
    else
        fprintf('\nFigure 7-8 cohort generation skipped; using: %s\n',cohortFile);
    end
    resolveCohortEvaluationSettings(load(cohortFile));
    results.last_completed_stage = 'figure7_8_cohort';
    localSaveProgress(projectDir,results,opts);

    if opts.RunMain
        fprintf('\nMain post-cohort stage: running the paired ablation study ...\n');
        localRunMain(projectDir,'post-cohort',cohortFile);
        warning(warningState);
        results.main_completed = true;
    else
        fprintf('\nMain post-cohort stage skipped by user option.\n');
    end

    results.last_completed_stage = 'post_cohort_ablation';
    localSaveProgress(projectDir,results,opts);

    if opts.RunClusterStats
        clusterOpts = localSetDefault(opts.ClusterOptions,'OutputDir', ...
            fullfile(fileparts(cohortFile),'cluster_statistics_output'));
        fprintf('\nRunning environment-level cluster-aware statistics ...\n');
        results.cluster_statistics = runClusterAwareStatistics( ...
            cohortFile,clusterOpts);
    else
        fprintf('\nCluster-aware statistics skipped.\n');
    end


    results.last_completed_stage = 'cluster_statistics';
    localSaveProgress(projectDir,results,opts);

    if opts.RunWeights
        weightOpts = localSetDefault(opts.WeightOptions,'OutputDir', ...
            fullfile(fileparts(cohortFile),'fixed_path_weight_sensitivity_results'));
        fprintf('\nRunning fixed-path post hoc cost-weight sensitivity (no replanning) ...\n');
        results.weights = runFixedPathWeightSensitivityAnalysis(cohortFile,weightOpts);
    else
        fprintf('\nFixed-path weight-sensitivity analysis skipped.\n');
    end
    results.last_completed_stage = 'weight_analysis';
    localSaveProgress(projectDir,results,opts);

    if opts.RunResolution
        resolutionOpts = localSetDefault(opts.ResolutionOptions,'SpacingsM', ...
            [6 3 1.5 0.75 0.375]);
        resolutionOpts = localSetDefault(resolutionOpts,'PlanningSpacingM',0.75);
        resolutionOpts = localSetDefault(resolutionOpts,'OutputDir', ...
            fullfile(fileparts(cohortFile),'fixed_path_resolution_audit'));
        fprintf('\nRunning 6/3/1.5/0.75/0.375 m fixed-path numerical audit ...\n');
        results.resolution = runSpatialResolutionSensitivity( ...
            cohortFile,resolutionOpts);
    else
        fprintf('\nSpatial-resolution analysis skipped.\n');
    end

    results.last_completed_stage = 'resolution_analysis';
    localSaveProgress(projectDir,results,opts);

    if opts.RunSummary
        fprintf('\nSummarizing fixed-path outcomes ...\n');
        summarizeExperimentOutcomes(cohortFile, ...
            fullfile(fileparts(cohortFile),'experiment_outcome_summary'));
        results.summary_completed = true;
    end
    results.last_completed_stage = 'outcome_summary';
    localSaveProgress(projectDir,results,opts);

    if opts.RunBudget
        budgetOpts = localSetDefault(opts.BudgetOptions,'PlanningSpacingM',0.75);
        budgetOpts = localSetDefault(budgetOpts,'FinalSpacingM',0.75);
        budgetOpts = localSetDefault(budgetOpts,'CohortFiles',{cohortFile});
        fprintf('\nRunning computational-budget/permission analysis ...\n');
        results.budget = runComputationalBudgetAnalysis(budgetOpts);
    else
        fprintf('\nComputational-budget analysis skipped.\n');
    end

    results.last_completed_stage = 'finished_requested_stages';
    localSaveProgress(projectDir,results,opts);
    fprintf('\nValidation suite completed.\n');
    fprintf('Master results: validation_suite_results.mat\n');
    fprintf('Log: %s\n',logFile);
end

function localPublishFigureArtifacts(projectDir,figureResults)
    fields = {'figure7','figure8'};
    for i=1:numel(fields)
        sourcePng = figureResults.(fields{i});
        [sourceDir,stem,~] = fileparts(sourcePng);
        for extension = {'.png','.pdf'}
            sourceFile = fullfile(sourceDir,[stem extension{1}]);
            if ~isfile(sourceFile)
                error('ValidationSuite:MissingFigureArtifact', ...
                    'Expected Figure 7-8 artifact is missing: %s',sourceFile);
            end
            copyfile(sourceFile,fullfile(projectDir,[stem extension{1}]),'f');
        end
    end
    fprintf('Figure 7-8 PNG/PDF files synchronized to the project root.\n');
end

function localRunMain(projectDir,runMode,cohortFile)
    % Isolate clearvars and run-mode switches from the user's base workspace.
    RA_ALA_RUN_MODE = runMode; %#ok<NASGU>
    RA_ALA_DEFER_ANALYSES = true; %#ok<NASGU>
    RA_ALA_EXTERNAL_COHORT_FILE = cohortFile; %#ok<NASGU>
    run(fullfile(projectDir,'runMainExperiments.m'));
end

function localSaveProgress(projectDir,results,opts)
    save(fullfile(projectDir,'validation_suite_results.mat'),'results','opts');
end

function S = localSetDefault(S,name,value)
    if ~isfield(S,name) || isempty(S.(name))
        S.(name) = value;
    end
end
