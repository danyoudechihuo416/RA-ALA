function results = runPlanningResolutionPilot(cohortFile,userOpts)
%RUNPLANNINGRESOLUTIONPILOT Compare 1.5 m and 0.75 m during RA-ALA search.
%   The archived 1.5 m path is paired with a fresh 0.75 m RA-ALA run
%   using the same environment and algorithm seed. Both paths are certified at 0.375 m.
%   This is distinct from runSpatialResolutionSensitivity, which keeps the
%   planned path fixed and changes only the evaluator resolution.
%
%   Default descriptive pilot (three environments, one run each):
%       R = runPlanningResolutionPilot('main_experiment_cohort.mat');
%
%   Full paired cohort (10 environments, three runs each):
%       R = runPlanningResolutionPilot('main_experiment_cohort.mat', ...
%           struct('N_ENV',10,'N_SEED',3));
%
%   A new output directory is required. Existing results are never
%   overwritten.

    if nargin < 1 || isempty(cohortFile)
        cohortFile = 'main_experiment_cohort.mat';
    end
    if nargin < 2, userOpts = struct(); end
    opts = localDefaults(userOpts);
    localPrepareOutputDirectory(opts.OutputDir);

    planningSpacings = opts.PlanningSpacingsM(:)';
    if numel(planningSpacings) ~= 2 || ...
            ~all(ismember([0.75 1.5],planningSpacings))
        error('PlanningResolutionPilot:RequiredPair', ...
            'PlanningSpacingsM must contain exactly 1.5 m and 0.75 m.');
    end

    fprintf('\nPaired planning-resolution pilot\n');
    fprintf('  Planning spacings: %s m\n',mat2str(planningSpacings));
    fprintf('  Common final certification: %.3g m\n',opts.FinalSpacingM);
    fprintf('  Environments: %d; runs/environment: %d\n\n', ...
        opts.N_ENV,opts.N_SEED);

    runs = cell(numel(planningSpacings),1);
    longTables = cell(numel(planningSpacings),1);
    for i = 1:numel(planningSpacings)
        spacing = planningSpacings(i);
        runDir = fullfile(opts.OutputDir,localSpacingFolder(spacing));
        fixedPathSpacings = unique([6 3 1.5 0.75 opts.FinalSpacingM spacing], ...
            'stable');
        childOpts = struct( ...
            'SpacingsM',fixedPathSpacings, ...
            'PlanningSpacingM',spacing, ...
            'ReuseSavedPaths',opts.ReuseArchived15mBaseline && abs(spacing-1.5)<1e-12, ...
            'MinSamples',opts.MinSamples, ...
            'N_ENV',opts.N_ENV, ...
            'N_SEED',opts.N_SEED, ...
            'TimeBudgetS',opts.TimeBudgetS, ...
            'NodeBudget',opts.NodeBudget, ...
            'RRTIterations',opts.RRTIterations, ...
            'Quiet',opts.Quiet, ...
            'OutputDir',runDir);
        runs{i} = runSpatialResolutionSensitivity(cohortFile,childOpts);

        T = runs{i}.raw;
        keep = abs(T.spacing_m-opts.FinalSpacingM) < 1e-12;
        T = T(keep,:);
        T.planning_spacing_m = repmat(spacing,height(T),1);
        T.planning_time_s = runs{i}.planning_time_s(:);
        T.planning_status = runs{i}.planning_status(:);
        longTables{i} = T;
    end

    caseResults = vertcat(longTables{:});
    caseResults = movevars(caseResults,'planning_spacing_m', ...
        'Before','spacing_m');
    caseResults.Properties.VariableNames{strcmp( ...
        caseResults.Properties.VariableNames,'spacing_m')} = ...
        'certification_spacing_m';
    pairedResults = localPair(caseResults,planningSpacings);
    summary = localSummary(caseResults,pairedResults,planningSpacings);

    writetable(caseResults,fullfile(opts.OutputDir, ...
        'planning_resolution_case_results.csv'));
    writetable(pairedResults,fullfile(opts.OutputDir, ...
        'planning_resolution_paired_results.csv'));
    writetable(summary,fullfile(opts.OutputDir, ...
        'planning_resolution_summary.csv'));
    localWriteReport(fullfile(opts.OutputDir, ...
        'planning_resolution_report.txt'),cohortFile,opts,summary);

    provenance = struct('created_at',char(datetime('now')), ...
        'cohort_file',char(cohortFile), ...
        'study_type','paired fresh planning at each resolution', ...
        'interpretation','descriptive pilot unless the full cohort is run');
    if exist('validationProvenance','file') == 2
        provenance.implementation = validationProvenance();
    end
    save(fullfile(opts.OutputDir,'planning_resolution_results.mat'), ...
        'caseResults','pairedResults','summary','runs','opts','provenance','-v7.3');

    results = struct('case_results',caseResults, ...
        'paired_results',pairedResults,'summary',summary, ...
        'runs',{runs},'options',opts,'output_dir',opts.OutputDir);
    fprintf('\nPaired pilot outputs written to: %s\n',opts.OutputDir);
end

function opts = localDefaults(u)
    stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
    opts = struct('PlanningSpacingsM',[1.5 0.75], ...
        'FinalSpacingM',0.375,'MinSamples',3, ...
        'N_ENV',3,'N_SEED',1,'TimeBudgetS',15, ...
        'NodeBudget',5000,'RRTIterations',2000,'Quiet',true, ...
        'ReuseArchived15mBaseline',true, ...
        'OutputDir',fullfile(pwd,['planning_resolution_pilot_' stamp]));
    names = fieldnames(u);
    for i = 1:numel(names), opts.(names{i}) = u.(names{i}); end
    validateattributes(opts.FinalSpacingM,{'numeric'}, ...
        {'scalar','positive','finite'});
    validateattributes(opts.MinSamples,{'numeric'}, ...
        {'scalar','integer','positive'});
    validateattributes(opts.N_ENV,{'numeric'}, ...
        {'scalar','integer','positive'});
    validateattributes(opts.N_SEED,{'numeric'}, ...
        {'scalar','integer','positive'});
end

function localPrepareOutputDirectory(outputDir)
    if exist(outputDir,'dir')
        error('PlanningResolutionPilot:OutputExists', ...
            'OutputDir already exists; choose a new directory: %s',outputDir);
    end
    [ok,msg] = mkdir(outputDir);
    if ~ok
        error('PlanningResolutionPilot:CreateOutputFailed', ...
            'Could not create OutputDir: %s',msg);
    end
end

function folder = localSpacingFolder(spacing)
    token = strrep(sprintf('%.6g',spacing),'.','p');
    folder = ['planning_' token 'm'];
end

function P = localPair(T,planningSpacings)
    low = min(planningSpacings);
    high = max(planningSpacings);
    A = T(abs(T.planning_spacing_m-high)<1e-12,:);
    B = T(abs(T.planning_spacing_m-low)<1e-12,:);
    keys = {'environment_seed','run_within_environment','algorithm_seed'};
    [~,ia,ib] = intersect(A(:,keys),B(:,keys),'rows','stable');
    A = A(ia,:); B = B(ib,:);

    environment_id = A.environment_id;
    environment_seed = A.environment_seed;
    run_within_environment = A.run_within_environment;
    algorithm_seed = A.algorithm_seed;
    feasible_1p5m = A.feasible;
    feasible_0p75m = B.feasible;
    J_1p5m = A.J; J_0p75m = B.J;
    energy_Wh_1p5m = A.energy_Wh; energy_Wh_0p75m = B.energy_Wh;
    arrival_time_s_1p5m = A.arrival_time_s;
    arrival_time_s_0p75m = B.arrival_time_s;
    penalty_1p5m = A.penalty_total; penalty_0p75m = B.penalty_total;
    planning_time_s_1p5m = A.planning_time_s;
    planning_time_s_0p75m = B.planning_time_s;
    status_1p5m = A.run_status; status_0p75m = B.run_status;
    delta_J_0p75_minus_1p5 = J_0p75m-J_1p5m;
    outcome = repmat({'same_feasibility'},height(A),1);
    outcome(~feasible_1p5m & feasible_0p75m) = {'improved_feasibility'};
    outcome(feasible_1p5m & ~feasible_0p75m) = {'worsened_feasibility'};
    jointlyFeasible = feasible_1p5m & feasible_0p75m;
    outcome(jointlyFeasible & delta_J_0p75_minus_1p5<0) = ...
        {'both_feasible_lower_J_at_0p75m'};
    outcome(jointlyFeasible & delta_J_0p75_minus_1p5>0) = ...
        {'both_feasible_higher_J_at_0p75m'};

    P = table(environment_id,environment_seed,run_within_environment, ...
        algorithm_seed,feasible_1p5m,feasible_0p75m,J_1p5m,J_0p75m, ...
        delta_J_0p75_minus_1p5,energy_Wh_1p5m,energy_Wh_0p75m, ...
        arrival_time_s_1p5m,arrival_time_s_0p75m,penalty_1p5m, ...
        penalty_0p75m,planning_time_s_1p5m,planning_time_s_0p75m, ...
        status_1p5m,status_0p75m,outcome);
end

function S = localSummary(T,P,planningSpacings)
    planning_spacing_m = planningSpacings(:);
    N = zeros(2,1); evaluated_N = zeros(2,1); numerical_failure_N = zeros(2,1);
    feasible_N = zeros(2,1); feasible_rate = nan(2,1);
    median_J_feasible = nan(2,1); median_energy_Wh_joint = nan(2,1);
    median_arrival_time_s_joint = nan(2,1); median_planning_time_s = nan(2,1);
    joint = P.feasible_1p5m & P.feasible_0p75m;
    for i = 1:2
        m = abs(T.planning_spacing_m-planningSpacings(i))<1e-12;
        U = T(m,:);
        ok = strcmp(U.run_status,'ok');
        N(i) = height(U); evaluated_N(i) = sum(ok);
        numerical_failure_N(i) = sum(~ok);
        feasible_N(i) = sum(U.feasible & ok);
        feasible_rate(i) = feasible_N(i)/N(i);
        median_J_feasible(i) = localMedian(U.J(U.feasible & ok));
        median_planning_time_s(i) = localMedian(U.planning_time_s);
        if abs(planningSpacings(i)-1.5)<1e-12
            median_energy_Wh_joint(i) = localMedian(P.energy_Wh_1p5m(joint));
            median_arrival_time_s_joint(i) = localMedian(P.arrival_time_s_1p5m(joint));
        else
            median_energy_Wh_joint(i) = localMedian(P.energy_Wh_0p75m(joint));
            median_arrival_time_s_joint(i) = localMedian(P.arrival_time_s_0p75m(joint));
        end
    end
    paired_N = repmat(height(P),2,1);
    improved_feasibility_N = repmat(sum(~P.feasible_1p5m & P.feasible_0p75m),2,1);
    worsened_feasibility_N = repmat(sum(P.feasible_1p5m & ~P.feasible_0p75m),2,1);
    jointly_feasible_N = repmat(sum(joint),2,1);
    median_paired_delta_J = repmat(localMedian( ...
        P.delta_J_0p75_minus_1p5(joint)),2,1);
    S = table(planning_spacing_m,N,evaluated_N,numerical_failure_N, ...
        feasible_N,feasible_rate,median_J_feasible, ...
        median_energy_Wh_joint,median_arrival_time_s_joint, ...
        median_planning_time_s,paired_N,improved_feasibility_N, ...
        worsened_feasibility_N,jointly_feasible_N,median_paired_delta_J);
end

function value = localMedian(x)
    x = x(isfinite(x));
    if isempty(x), value = NaN; else, value = median(x); end
end

function localWriteReport(file,cohortFile,opts,S)
    fid = fopen(file,'w');
    if fid < 0
        error('PlanningResolutionPilot:ReportOpenFailed', ...
            'Could not open report file: %s',file);
    end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid,'Paired planning-resolution pilot\n');
    fprintf(fid,'Cohort: %s\n',cohortFile);
    fprintf(fid,'Planning spacings: %s m\n',mat2str(opts.PlanningSpacingsM));
    fprintf(fid,'Common certification spacing: %.3g m\n',opts.FinalSpacingM);
    fprintf(fid,'Environments: %d; runs/environment: %d\n\n',opts.N_ENV,opts.N_SEED);
    for i = 1:height(S)
        fprintf(fid,['Planning %.3g m: feasible %d/%d (%.1f%%), ', ...
            'numerical failures %d, median feasible J %.6g, ', ...
            'median planning time %.3f s.\n'], ...
            S.planning_spacing_m(i),S.feasible_N(i),S.N(i), ...
            100*S.feasible_rate(i),S.numerical_failure_N(i), ...
            S.median_J_feasible(i),S.median_planning_time_s(i));
    end
    fprintf(fid,['\nPaired transitions from 1.5 m to 0.75 m: improved %d, ', ...
        'worsened %d, jointly feasible %d/%d.\n'], ...
        S.improved_feasibility_N(1),S.worsened_feasibility_N(1), ...
        S.jointly_feasible_N(1),S.paired_N(1));
    fprintf(fid,'Median paired delta J (0.75 m - 1.5 m) among jointly feasible cases: %.6g.\n', ...
        S.median_paired_delta_J(1));
    fprintf(fid,['\nInterpretation: the default three-environment run is a ', ...
        'descriptive pilot. It is not a replacement for the prespecified ', ...
        '10-environment, three-run cohort or cluster-aware inference.\n']);
end
