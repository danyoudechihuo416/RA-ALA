function results = runReproducibilitySuite(userOpts)
%RUNREPRODUCIBILITYSUITE Run the complete reproducibility suite.
%   results = runReproducibilitySuite();
%
%   Execution order:
%     1. RA_ALA_demo: revised main experiments, including ST-EA* and the
%        existing weight-sensitivity analysis.
%     2. Environment-level cluster-aware inference for Section 5.5.
%     3. Fixed-path 12/6/3/1.5/0.75 m resolution sensitivity analysis,
%        using paths planned at the final 1.5 m main-experiment resolution.
%     4. Five-planner computational-budget and permission analysis.
%
%   To reuse an existing revised Section 5.5 cohort without rerunning the
%   long main experiment:
%     opts = struct('RunMain',false);
%     results = runReproducibilitySuite(opts);

    if nargin < 1, userOpts = struct(); end
    opts = struct('RunMain',true,'RunClusterStats',true, ...
        'RunResolution',true,'RunBudget',true,'ClusterOptions',struct(), ...
        'ResolutionOptions',struct(),'BudgetOptions',struct());
    names = fieldnames(userOpts);
    for i=1:numel(names), opts.(names{i})=userOpts.(names{i}); end

    projectDir = fileparts(mfilename('fullpath'));
    oldDir = pwd;
    cleanupDir = onCleanup(@()cd(oldDir)); %#ok<NASGU>
    cd(projectDir);
    logFile = fullfile(projectDir,'reproducibility_suite_run.log');
    diary(logFile);
    cleanupDiary = onCleanup(@()diary('off')); %#ok<NASGU>

    fprintf('\n============================================================\n');
    fprintf('Reproducibility suite started: %s\n', ...
        char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
    fprintf('Project: %s\n',projectDir);
    fprintf('============================================================\n');

    results = struct('main_completed',false,'cluster_statistics',[], ...
        'resolution',[],'budget',[],'options',opts,'log_file',logFile);

    if opts.RunMain
        fprintf('\n[1/4] Running revised main experiments (RA_ALA_demo.m) ...\n');
        evalin('base',sprintf('cd(''%s''); RA_ALA_demo;', ...
            strrep(projectDir,'''','''''')));
        results.main_completed = true;
    else
        fprintf('\n[1/4] Main experiments skipped by user option.\n');
        if ~exist('section55_same_cohort_data.mat','file')
            error('ReproducibilitySuite:MissingCohort', ...
                ['RunMain=false requires section55_same_cohort_data.mat ', ...
                'in the project folder.']);
        end
    end

    if opts.RunClusterStats
        fprintf('\n[2/4] Running environment-level cluster-aware statistics ...\n');
        results.cluster_statistics = runClusterAwareStatistics( ...
            'section55_same_cohort_data.mat',opts.ClusterOptions);
    else
        fprintf('\n[2/4] Cluster-aware statistics skipped.\n');
    end


    if opts.RunResolution
        fprintf('\n[3/4] Running 12/6/3/1.5/0.75 m spatial-resolution analysis ...\n');
        results.resolution = runSpatialResolutionSensitivity( ...
            'section55_same_cohort_data.mat',opts.ResolutionOptions);
    else
        fprintf('\n[3/4] Spatial-resolution analysis skipped.\n');
    end

    if opts.RunBudget
        fprintf('\n[4/4] Running computational-budget/permission analysis ...\n');
        results.budget = runComputationalBudgetAnalysis(opts.BudgetOptions);
    else
        fprintf('\n[4/4] Computational-budget analysis skipped.\n');
    end

    save(fullfile(projectDir,'reproducibility_suite_results.mat'), ...
        'results','opts');
    fprintf('\nReproducibility suite completed.\n');
    fprintf('Master results: reproducibility_suite_results.mat\n');
    fprintf('Log: %s\n',logFile);
end
