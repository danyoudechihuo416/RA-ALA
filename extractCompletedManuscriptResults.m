function outputs = extractCompletedManuscriptResults(cohortFile, outputDir)
%EXTRACTCOMPLETEDMANUSCRIPTRESULTS Export manuscript-ready summaries.
%   This function only reads the completed Figure 7-8 cohort. It does not
%   run planners, alter paths, or feed finer-resolution findings back into
%   optimization.

    if nargin < 1 || isempty(cohortFile)
        projectDir = fileparts(mfilename('fullpath'));
        cohortFile = fullfile(projectDir,'main_experiment_cohort.mat');
    end
    if nargin < 2 || isempty(outputDir)
        outputDir = fullfile(fileparts(mfilename('fullpath')), ...
            'manuscript_completed_results');
    end
    if ~isfile(cohortFile)
        error('CompletedResults:MissingCohort', ...
            'Completed cohort not found: %s',cohortFile);
    end
    if ~exist(outputDir,'dir'), mkdir(outputDir); end

    S = load(cohortFile,'algNames','env_seeds_used','stat_env', ...
        'stat_feasible','stat_planner_success','stat_failure_reason', ...
        'stat_is_unique_trial','stat_final_evaluation_details', ...
        'stat_runtime_s','stat_evaluator_calls');
    nAlg = numel(S.algNames);

    planner = strings(nAlg,1);
    environments = repmat(numel(S.env_seeds_used),nAlg,1);
    unique_runs = zeros(nAlg,1);
    generated_paths = zeros(nAlg,1);
    generation_failures = zeros(nAlg,1);
    feasible_paths = zeros(nAlg,1);
    height_violations = zeros(nAlg,1);
    static_violations = zeros(nAlg,1);
    scene_entry_violations = zeros(nAlg,1);
    nfz_violations = zeros(nAlg,1);
    battery_violations = zeros(nAlg,1);
    kinematic_violations = zeros(nAlg,1);
    multiple_positive_hard_penalty_components = zeros(nAlg,1);

    for a = 1:nAlg
        planner(a) = string(S.algNames{a});
        use = find(S.stat_is_unique_trial(a,:));
        unique_runs(a) = numel(use);
        generated_paths(a) = sum(S.stat_planner_success(a,use));
        generation_failures(a) = unique_runs(a) - generated_paths(a);
        feasible_paths(a) = sum(S.stat_feasible(a,use));

        for k = 1:numel(use)
            d = S.stat_final_evaluation_details{a,use(k)};
            if isempty(d), continue; end
            flags = [localPositive(d,'penalty_height'), ...
                localPositive(d,'penalty_static_collision'), ...
                localSceneEntryPositive(d), ...
                localPositive(d,'penalty_nfz'), ...
                localPositive(d,'penalty_battery'), ...
                localPositive(d,'penalty_kinematic')];
            height_violations(a) = height_violations(a) + flags(1);
            static_violations(a) = static_violations(a) + flags(2);
            scene_entry_violations(a) = scene_entry_violations(a) + flags(3);
            nfz_violations(a) = nfz_violations(a) + flags(4);
            battery_violations(a) = battery_violations(a) + flags(5);
            kinematic_violations(a) = kinematic_violations(a) + flags(6);
            multiple_positive_hard_penalty_components(a) = ...
                multiple_positive_hard_penalty_components(a) + (sum(flags) > 1);
        end
    end

    outcomeTable = table(planner,environments,unique_runs,generated_paths, ...
        generation_failures,feasible_paths,height_violations, ...
        static_violations,scene_entry_violations,nfz_violations, ...
        battery_violations,kinematic_violations, ...
        multiple_positive_hard_penalty_components);
    outcomeFile = fullfile(outputDir,'planner_outcome_counts.csv');
    writetable(outcomeTable,outcomeFile);

    failurePlanner = strings(0,1);
    failureReason = strings(0,1);
    failureCount = zeros(0,1);
    for a = 1:nAlg
        use = find(S.stat_is_unique_trial(a,:));
        reasons = string(S.stat_failure_reason(a,use));
        reasons(strlength(reasons)==0) = "unspecified";
        [groups,~,idx] = unique(reasons);
        for g = 1:numel(groups)
            failurePlanner(end+1,1) = string(S.algNames{a}); %#ok<AGROW>
            failureReason(end+1,1) = groups(g); %#ok<AGROW>
            failureCount(end+1,1) = sum(idx==g); %#ok<AGROW>
        end
    end
    failureTable = table(failurePlanner,failureReason,failureCount, ...
        'VariableNames',{'planner','failure_reason','count'});
    failureFile = fullfile(outputDir,'planner_failure_reasons.csv');
    writetable(failureTable,failureFile);

    n = zeros(nAlg,1);
    runtime_median_s = nan(nAlg,1);
    runtime_q1_s = nan(nAlg,1);
    runtime_q3_s = nan(nAlg,1);
    evaluator_calls_median = nan(nAlg,1);
    evaluator_calls_q1 = nan(nAlg,1);
    evaluator_calls_q3 = nan(nAlg,1);
    for a = 1:nAlg
        use = logical(S.stat_is_unique_trial(a,:));
        n(a) = sum(use);
        rt = S.stat_runtime_s(a,use);
        calls = S.stat_evaluator_calls(a,use);
        runtime_median_s(a) = median(rt,'omitnan');
        runtime_q1_s(a) = localQuantile(rt,0.25);
        runtime_q3_s(a) = localQuantile(rt,0.75);
        evaluator_calls_median(a) = median(calls,'omitnan');
        evaluator_calls_q1(a) = localQuantile(calls,0.25);
        evaluator_calls_q3(a) = localQuantile(calls,0.75);
    end
    runtimeTable = table(planner,n,runtime_median_s,runtime_q1_s, ...
        runtime_q3_s,evaluator_calls_median,evaluator_calls_q1, ...
        evaluator_calls_q3,feasible_paths);
    runtimeFile = fullfile(outputDir,'planner_runtime_summary.csv');
    writetable(runtimeTable,runtimeFile);

    outputs = struct('outcome_table',outcomeTable, ...
        'failure_table',failureTable,'runtime_table',runtimeTable, ...
        'outcome_file',outcomeFile,'failure_file',failureFile, ...
        'runtime_file',runtimeFile,'cohort_file',cohortFile);
end

function tf = localPositive(details, fieldName)
    tf = isfield(details,fieldName) && isfinite(details.(fieldName)) && ...
        details.(fieldName) > 0;
end

function tf = localSceneEntryPositive(details)
    if isfield(details,'penalty_scene_entry')
        tf = isfinite(details.penalty_scene_entry) && ...
            details.penalty_scene_entry > 0;
    else
        tf = localPositive(details,'penalty_dynamic_collision');
    end
end

function q = localQuantile(x,p)
    x = sort(x(isfinite(x)));
    if isempty(x), q = NaN; return; end
    if numel(x)==1, q = x; return; end
    idx = 1 + p*(numel(x)-1);
    lo = floor(idx); hi = ceil(idx); f = idx-lo;
    q = x(lo)*(1-f) + x(hi)*f;
end
