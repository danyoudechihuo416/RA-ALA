function results = runComputationalBudgetAnalysis(userOpts)
%RUNCOMPUTATIONALBUDGETANALYSIS Generate computational-budget and runtime evidence.
%   Full run:
%       results = runComputationalBudgetAnalysis();
%
%   Quick syntax/smoke run:
%       opts = struct('N_ENV',1,'N_SEED',1,'RRTIterations',20, ...
%           'TimeBudgetS',2,'NodeBudget',100, ...
%           'ALAConfig',struct('popSize',8,'maxIter',2, ...
%           'nWaypoints',4,'riskWeight',15,'windLookahead',3));
%       results = runComputationalBudgetAnalysis(opts);
%
%   The timer covers the complete planner call, including candidate
%   generation, unified re-evaluation, smoothing, and conditional recovery
%   when the method uses them. Environment generation, plotting, and file I/O
%   are outside the timed region.

    if nargin < 1, userOpts = struct(); end
    opts = defaultOptions(userOpts);
    if ~exist(opts.OutputDir, 'dir'), mkdir(opts.OutputDir); end

    global EVAL_COUNTER RESCUE_A_COUNT RESCUE_B_COUNT;
    EVAL_COUNTER = 0;
    RESCUE_A_COUNT = 0;
    RESCUE_B_COUNT = 0;

    hardware = collectHardwareInfo();
    writeHardwareOutputs(hardware, opts.OutputDir);

    algNames = {'RA-ALA','Energy-A*','Informed-RRT*','ST-EA*','Greedy'};
    budgetTable = buildBudgetPermissionsTable(opts, algNames);
    writetable(budgetTable, fullfile(opts.OutputDir, ...
        'algorithm_budget_permissions.csv'));
    if opts.TableOnly
        results = struct('budget_permissions',budgetTable,'hardware',hardware, ...
            'options',opts);
        fprintf('Budget/permission table written to: %s\n',opts.OutputDir);
        return;
    end

    envSeeds = resolveEnvironmentSeeds(opts);
    nRows = numel(envSeeds)*opts.N_SEED*numel(algNames);
    R = initializeRawResults(nRows);
    row = 0;

    fprintf('\nComputational-budget analysis\n');
    fprintf('  Environments: %d; repeated runs/environment: %d\n', ...
        numel(envSeeds), opts.N_SEED);
    fprintf('  Timer excludes environment generation, plotting, and file export.\n\n');

    for e = 1:numel(envSeeds)
        envSeed = envSeeds(e);
        [env, costModel, planner] = makeEnvironment(opts, envSeed);

        for s = 1:opts.N_SEED
            commonSeed = envSeed + s*53;
            fprintf('  env %2d/%d seed=%-6d run=%d/%d ', ...
                e, numel(envSeeds), envSeed, s, opts.N_SEED);

            for a = 1:numel(algNames)
                row = row + 1;
                algSeed = commonSeed + a*11;
                rng(algSeed, 'twister');
                EVAL_COUNTER = 0;
                RESCUE_A_COUNT = 0;
                RESCUE_B_COUNT = 0;

                R.environment_id(row) = e;
                R.env_seed(row) = envSeed;
                R.run_within_environment(row) = s;
                R.algorithm_seed(row) = algSeed;
                R.algorithm{row} = algNames{a};

                tRun = tic;
                try
                    [path, details, info, searchFitnessCalls] = runOnePlanner( ...
                        a, planner, costModel, env, opts, algSeed);
                    elapsed = toc(tRun);

                    R.wall_clock_s(row) = elapsed;
                    R.evaluatePath_calls(row) = EVAL_COUNTER;
                    R.search_fitness_calls(row) = searchFitnessCalls;
                    R.rescueA_adopted(row) = safeGet(details,'rescue_a_count',0);
                    R.rescueB_adopted(row) = safeGet(details,'rescue_b_count',0);
                    R.search_time_s(row) = nestedGet(details,{'timing','search_s'},NaN);
                    R.topk_time_s(row) = nestedGet(details,{'timing','topk_s'},NaN);
                    R.recovery_time_s(row) = nestedGet(details, ...
                        {'timing','rescue_total_s'},NaN);
                    R.actual_iterations_or_nodes(row) = actualWork(info,a,opts);
                    R.planner_success(row) = logical(safeGet(info,'success',false));
                    R.budget_reached(row) = logical(safeGet(info,'budgetExhausted',a==1));
                    R.timed_out(row) = determineTimeout(info,a,opts);
                    R.path_points(row) = size(path,1);
                    R.J(row) = safeGet(details,'J_final',NaN);
                    R.penalty_total(row) = safeGet(details,'penalty_total',NaN);
                    R.final_feasible(row) = logical(safeGet(details,'feasible',false));
                    R.failure_reason{row} = classifyFailure('',path,details,info, ...
                        R.timed_out(row));
                    fprintf('| %s %.2fs eval=%d ', shortName(algNames{a}), ...
                        elapsed, EVAL_COUNTER);
                catch ME
                    elapsed = toc(tRun);
                    R.wall_clock_s(row) = elapsed;
                    R.evaluatePath_calls(row) = EVAL_COUNTER;
                    R.exception(row) = true;
                    R.failure_reason{row} = ['exception:', nonemptyId(ME)];
                    fprintf('| %s FAILED ', shortName(algNames{a}));
                end
            end
            fprintf('\n');
        end
    end

    rawTable = rawStructToTable(R);
    summaryTable = summarizeRuntime(rawTable, algNames);
    writetable(rawTable, fullfile(opts.OutputDir,'runtime_case_results.csv'));
    writetable(summaryTable, fullfile(opts.OutputDir,'runtime_summary.csv'));

    writeHumanReadableReport(opts.OutputDir, hardware, budgetTable, ...
        summaryTable, envSeeds, opts);
    save(fullfile(opts.OutputDir,'computational_budget_results.mat'), ...
        'rawTable','summaryTable','budgetTable','hardware','envSeeds','opts');

    results = struct('raw',rawTable,'summary',summaryTable, ...
        'budget_permissions',budgetTable,'hardware',hardware, ...
        'environment_seeds',envSeeds,'options',opts);

    fprintf('\nOutputs written to: %s\n', opts.OutputDir);
    fprintf('  algorithm_budget_permissions.csv\n');
    fprintf('  runtime_case_results.csv\n');
    fprintf('  runtime_summary.csv\n');
    fprintf('  hardware_specifications.csv / hardware_specifications.txt\n');
    fprintf('  computational_budget_report.txt\n');
end

function opts = defaultOptions(u)
    opts = struct();
    opts.Seed = 42;
    opts.N_ENV = 10;
    opts.N_SEED = 3;
    opts.MapSize = 1000;
    opts.GridStep = 10;
    opts.WindLevel = 'medium';
    opts.RiskLevel = 'dense';
    opts.Start = [80 80 60];
    opts.Goal = [900 900 60];
    opts.TimeBudgetS = 15;
    opts.NodeBudget = 5000;
    opts.RRTIterations = 1500;
    opts.TimeStepS = 2;
    opts.TimeHorizonS = 300;
    opts.ScreenMode = 'fixed';
    opts.ScreenGap = 1.3;
    opts.OutputDir = fullfile(pwd,'computational_budget_output');
    opts.EnvironmentSeeds = [483, 638, 855, 948, 1041, 1103, 1227, 1475, 2312, 2560];
    opts.CohortFiles = {'section55_same_cohort_data.mat', ...
        'environment_level_statistics.mat','fig9_ablation_same_cohort_data.mat'};
    opts.QuietRAALA = true;
    opts.TableOnly = false;
    opts.ALAConfig = struct('popSize',40,'maxIter',80,'nWaypoints',8, ...
        'riskWeight',15.0,'windLookahead',3,'rescue_max_ins',12);

    names = fieldnames(u);
    for i = 1:numel(names), opts.(names{i}) = u.(names{i}); end
    requiredCfg = struct('popSize',40,'maxIter',80,'nWaypoints',8, ...
        'riskWeight',15.0,'windLookahead',3,'rescue_max_ins',12);
    cfgNames = fieldnames(requiredCfg);
    for i = 1:numel(cfgNames)
        if ~isfield(opts.ALAConfig,cfgNames{i})
            opts.ALAConfig.(cfgNames{i}) = requiredCfg.(cfgNames{i});
        end
    end
end

function T = buildBudgetPermissionsTable(opts, algNames)
    algorithm = algNames(:);
    comparison_level = repmat({'complete planner configuration'},5,1);
    temporal_reasoning = {
        'arrival-time-recursive unified evaluation inside search and final selection';
        'no; dynamic checks use departure-time approximation during graph search';
        'no explicit temporal state; collision checks do not propagate arrival time';
        'yes; state=(graph node,time bin), with exact propagated edge-arrival time';
        'no; dynamic checks use departure-time approximation'};
    initialization = {
        'structured population + Greedy projection + Energy-A* warm start';
        'nearest graph node initialization';
        'single start-node tree';
        'nearest space-time graph state at departure';
        'deterministic greedy construction'};
    population_or_sample_size = {
        sprintf('%d individuals',opts.ALAConfig.popSize);
        'N/A (spatial graph search)';
        'one sample per iteration';
        sprintf('N/A (space-time graph; %.1f s bins)',opts.TimeStepS);
        'N/A (deterministic)'};
    iteration_node_budget = {
        sprintf('%d iterations',opts.ALAConfig.maxIter);
        sprintf('%d expanded spatial nodes',opts.NodeBudget);
        sprintf('%d sampling iterations',opts.RRTIterations);
        sprintf('%d expanded space-time states',opts.NodeBudget);
        'one construction pass'};
    wall_clock_timeout = {
        'none; fixed iteration budget';
        sprintf('%.1f s',opts.TimeBudgetS);
        sprintf('%.1f s',opts.TimeBudgetS);
        sprintf('%.1f s',opts.TimeBudgetS);
        'none; deterministic pass'};
    stopping_criteria = {
        'fixed iteration budget, followed by Top-K selection and conditional recovery';
        'goal reached, node budget exhausted, open set empty, or timeout';
        'sampling budget exhausted or timeout';
        sprintf('goal reached, %d-state budget, %.0f-s horizon, open set empty, or timeout',opts.NodeBudget,opts.TimeHorizonS);
        'construction completed'};
    internal_search_fitness_calls = {
        sprintf('%d planned (= population x [initial + iterations])',opts.ALAConfig.popSize*(opts.ALAConfig.maxIter+1));
        'variable; one unified edge evaluation per relaxation attempt';
        'variable; edge-energy and collision checks';
        'variable; one unified edge evaluation per space-time transition attempt';
        'N/A'};
    candidate_policy = {
        'Top-K=5 parents; raw+smooth per parent; conditional mild and Rescue candidates';
        'one graph-derived output path';
        'one best tree-derived output path';
        'one best space-time graph output path';
        'one greedy output path'};
    smoothing_permitted = {
        'yes: spline smoothing; mild smoothing only in difficult environments';
        'no';'no';'no';'no'};
    local_recovery_permitted = {
        'yes: conditional RescueA dynamic detours and RescueB altitude/static recovery';
        'no';'no';'no';'no'};
    final_selection = {
        'unified re-evaluation; feasibility first, then lower J';
        'single output, followed by unified evaluation';
        'single output, followed by unified evaluation';
        'single output, followed by unified evaluation';
        'single output, followed by unified evaluation'};
    timeout_rule = {
        'not applicable; maxIter is enforced';
        'stop when elapsed planner time reaches timeBudget';
        'stop when elapsed planner time reaches timeBudget';
        'stop when elapsed planner time reaches timeBudget';
        'not applicable'};
    failure_rule = repmat({['exception, no path/goal, nonfinite output, or any ', ...
        'nonzero hard-constraint violation in the unified evaluator']},5,1);
    parameter_tuning = repmat({['fixed manuscript settings before this timing run; ', ...
        'parameter provenance must be documented in the manuscript']},5,1);

    T = table(algorithm,comparison_level,temporal_reasoning,initialization, ...
        population_or_sample_size,iteration_node_budget,wall_clock_timeout, ...
        stopping_criteria,internal_search_fitness_calls,candidate_policy, ...
        smoothing_permitted,local_recovery_permitted,final_selection, ...
        timeout_rule,failure_rule,parameter_tuning);
end
function envSeeds = resolveEnvironmentSeeds(opts)
    if ~isempty(opts.EnvironmentSeeds)
        envSeeds = opts.EnvironmentSeeds(:)';
        envSeeds = envSeeds(1:min(opts.N_ENV,numel(envSeeds)));
        return;
    end

    envSeeds = [];
    for i = 1:numel(opts.CohortFiles)
        f = opts.CohortFiles{i};
        if ~exist(f,'file'), continue; end
        L = load(f);
        if isfield(L,'env_seeds_used')
            envSeeds = L.env_seeds_used(:)';
        elseif isfield(L,'stat_env')
            envSeeds = unique(L.stat_env(isfinite(L.stat_env)),'stable');
        elseif isfield(L,'hier_stats') && isfield(L.hier_stats,'env_seeds')
            envSeeds = L.hier_stats.env_seeds(:)';
        end
        if ~isempty(envSeeds), break; end
    end

    if numel(envSeeds) >= opts.N_ENV
        envSeeds = envSeeds(1:opts.N_ENV);
        fprintf('Using %d previously recorded Section 5.5 environment seeds.\n', ...
            opts.N_ENV);
        return;
    end

    fprintf('No complete saved cohort found; reproducing Section 5.5 screening.\n');
    envSeeds = screenEnvironments(opts);
end

function seeds = screenEnvironments(opts)
    seeds = [];
    candidate = opts.Seed + 100;
    cfg = opts.ALAConfig;
    cfg.popSize = 30;
    cfg.maxIter = 60;
    cfg.rescue_max_ins = 8;

    while numel(seeds) < opts.N_ENV
        candidate = candidate + 31;
        [env,costModel,planner] = makeEnvironment(opts,candidate);
        fullCost = inf; fullFeasible = false;

        if any(strcmp(opts.ScreenMode,{'scheme_25','full_only'}))
            try
                rng(candidate+300,'twister');
                if opts.QuietRAALA
                    evalc('[~,fullCost,fullDet] = runRA_ALA(planner,costModel,env,opts.Start,opts.Goal,0,true,cfg);');
                else
                    [~,fullCost,fullDet] = runRA_ALA(planner,costModel,env, ...
                        opts.Start,opts.Goal,0,true,cfg);
                end
                fullFeasible = fullDet.feasible;
            catch
                fullFeasible = false;
            end
            if ~fullFeasible, continue; end
        end

        astarCost = inf; astarFeasible = false;
        try
            rng(candidate+200,'twister');
            [~,astarCost,astarInfo] = planner.energyAStar( ...
                opts.Start,opts.Goal,0,true);
            astarFeasible = safeGet(astarInfo.details,'feasible',false);
        catch
            astarFeasible = false;
        end

        include = false;
        switch opts.ScreenMode
            case 'astar_only'
                include = astarFeasible;
            case 'full_only'
                include = true;
            case 'scheme_25'
                include = ~astarFeasible || astarCost >= fullCost*opts.ScreenGap;
            otherwise
                error('Unknown ScreenMode: %s',opts.ScreenMode);
        end
        if include
            seeds(end+1) = candidate; %#ok<AGROW>
            fprintf('  accepted environment %d/%d: seed=%d\n', ...
                numel(seeds),opts.N_ENV,candidate);
        end
    end
end

function [env,costModel,planner] = makeEnvironment(opts,envSeed)
    rng(envSeed,'twister');
    env = CityEnvironment(opts.MapSize,opts.GridStep);
    env.generate('high',opts.WindLevel,opts.RiskLevel,envSeed);
    env.setTaskPoints(opts.Start,opts.Goal);
    costModel = UnifiedCostModel();
    costModel.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
    planner = PathPlanners(env,costModel);
    planner.setBudget(opts.TimeBudgetS,opts.NodeBudget,opts.RRTIterations);
end

function [path,details,info,searchCalls] = runOnePlanner( ...
        a,planner,costModel,env,opts,algSeed)
    rng(algSeed,'twister');
    searchCalls = 0;
    switch a
        case 1
            cfg = opts.ALAConfig;
            searchCalls = cfg.popSize*(cfg.maxIter+1);
            if opts.QuietRAALA
                evalc('[path,~,details] = runRA_ALA(planner,costModel,env,opts.Start,opts.Goal,0,true,cfg);');
            else
                [path,~,details] = runRA_ALA(planner,costModel,env, ...
                    opts.Start,opts.Goal,0,true,cfg);
            end
            info = struct('success',details.feasible,'reachedGoal',true, ...
                'iterations',cfg.maxIter,'iterationBudget',cfg.maxIter, ...
                'timeBudget',inf,'budgetExhausted',true);
        case 2
            [path,~,info] = planner.energyAStar(opts.Start,opts.Goal,0,true);
            details = safeGet(info,'details',struct());
        case 3
            [path,~,info] = planner.informedRRTStar( ...
                opts.Start,opts.Goal,0,true,opts.RRTIterations);
            details = safeGet(info,'details',struct());
        case 4
            [path,~,info] = planner.timeExpandedEnergyAStar( ...
                opts.Start,opts.Goal,0,true,opts.TimeStepS,opts.TimeHorizonS);
            details = safeGet(info,'details',struct());
        case 5
            [path,~,info] = planner.greedyPlanner(opts.Start,opts.Goal,0,true);
            details = safeGet(info,'details',struct());
    end
end
function R = initializeRawResults(n)
    R = struct();
    R.environment_id = nan(n,1);
    R.env_seed = nan(n,1);
    R.run_within_environment = nan(n,1);
    R.algorithm_seed = nan(n,1);
    R.algorithm = repmat({''},n,1);
    R.wall_clock_s = nan(n,1);
    R.evaluatePath_calls = nan(n,1);
    R.search_fitness_calls = nan(n,1);
    R.rescueA_adopted = nan(n,1);
    R.rescueB_adopted = nan(n,1);
    R.search_time_s = nan(n,1);
    R.topk_time_s = nan(n,1);
    R.recovery_time_s = nan(n,1);
    R.actual_iterations_or_nodes = nan(n,1);
    R.planner_success = false(n,1);
    R.budget_reached = false(n,1);
    R.timed_out = false(n,1);
    R.exception = false(n,1);
    R.path_points = nan(n,1);
    R.J = nan(n,1);
    R.penalty_total = nan(n,1);
    R.final_feasible = false(n,1);
    R.failure_reason = repmat({''},n,1);
end

function T = rawStructToTable(R)
    T = table(R.environment_id,R.env_seed,R.run_within_environment, ...
        R.algorithm_seed,R.algorithm,R.wall_clock_s,R.evaluatePath_calls, ...
        R.search_fitness_calls,R.rescueA_adopted,R.rescueB_adopted, ...
        R.search_time_s,R.topk_time_s,R.recovery_time_s, ...
        R.actual_iterations_or_nodes,R.planner_success,R.budget_reached, ...
        R.timed_out,R.exception,R.path_points,R.J,R.penalty_total, ...
        R.final_feasible,R.failure_reason, ...
        'VariableNames',{'environment_id','env_seed','run_within_environment', ...
        'algorithm_seed','algorithm','wall_clock_s','evaluatePath_calls', ...
        'search_fitness_calls','rescueA_adopted','rescueB_adopted', ...
        'search_time_s','topk_time_s','recovery_time_s', ...
        'actual_iterations_or_nodes','planner_success','budget_reached', ...
        'timed_out','exception','path_points','J','penalty_total', ...
        'final_feasible','failure_reason'});
end

function S = summarizeRuntime(T,algNames)
    n = numel(algNames);
    algorithm = algNames(:);
    N = zeros(n,1);
    runtime_median_s = nan(n,1); runtime_Q1_s = nan(n,1);
    runtime_Q3_s = nan(n,1); runtime_IQR_s = nan(n,1);
    runtime_min_s = nan(n,1); runtime_max_s = nan(n,1);
    evaluatePath_calls_median = nan(n,1);
    evaluatePath_calls_Q1 = nan(n,1); evaluatePath_calls_Q3 = nan(n,1);
    search_fitness_calls_median = nan(n,1);
    timeout_count = zeros(n,1); exception_count = zeros(n,1);
    failure_count = zeros(n,1); feasibility_rate = nan(n,1);

    for a = 1:n
        mask = strcmp(T.algorithm,algNames{a});
        N(a) = sum(mask);
        rt = T.wall_clock_s(mask & isfinite(T.wall_clock_s));
        ev = T.evaluatePath_calls(mask & isfinite(T.evaluatePath_calls));
        sf = T.search_fitness_calls(mask & isfinite(T.search_fitness_calls));
        if ~isempty(rt)
            runtime_median_s(a)=median(rt); runtime_Q1_s(a)=qtl(rt,.25);
            runtime_Q3_s(a)=qtl(rt,.75);
            runtime_IQR_s(a)=runtime_Q3_s(a)-runtime_Q1_s(a);
            runtime_min_s(a)=min(rt); runtime_max_s(a)=max(rt);
        end
        if ~isempty(ev)
            evaluatePath_calls_median(a)=median(ev);
            evaluatePath_calls_Q1(a)=qtl(ev,.25);
            evaluatePath_calls_Q3(a)=qtl(ev,.75);
        end
        if ~isempty(sf), search_fitness_calls_median(a)=median(sf); end
        timeout_count(a)=sum(T.timed_out(mask));
        exception_count(a)=sum(T.exception(mask));
        failure_count(a)=sum(~T.final_feasible(mask));
        feasibility_rate(a)=mean(T.final_feasible(mask));
    end

    S = table(algorithm,N,runtime_median_s,runtime_Q1_s,runtime_Q3_s, ...
        runtime_IQR_s,runtime_min_s,runtime_max_s, ...
        evaluatePath_calls_median,evaluatePath_calls_Q1, ...
        evaluatePath_calls_Q3,search_fitness_calls_median,timeout_count, ...
        exception_count,failure_count,feasibility_rate);
end

function hardware = collectHardwareInfo()
    hardware = struct();
    hardware.run_datetime = datestr(now,31);
    hardware.matlab_version = version;
    try, hardware.matlab_release = version('-release'); catch, hardware.matlab_release='unknown'; end
    hardware.architecture = computer('arch');
    try
        osName = char(java.lang.System.getProperty('os.name'));
        osVersion = char(java.lang.System.getProperty('os.version'));
        osArch = char(java.lang.System.getProperty('os.arch'));
        if ispc
            hardware.operating_system = sprintf('Microsoft Windows (kernel %s, %s)', ...
                osVersion,osArch);
        else
            hardware.operating_system = sprintf('%s %s (%s)',osName,osVersion,osArch);
        end
    catch
        hardware.operating_system = computer;
    end
    hardware.cpu_model = strtrim(getenv('PROCESSOR_IDENTIFIER'));
    hardware.logical_processors = str2double(getenv('NUMBER_OF_PROCESSORS'));
    hardware.ram_gb = NaN;

    try
        bean = java.lang.management.ManagementFactory.getOperatingSystemMXBean();
        try
            ramBytes = double(bean.getTotalMemorySize());
        catch
            ramBytes = double(bean.getTotalPhysicalMemorySize());
        end
        if isfinite(ramBytes) && ramBytes > 0
            hardware.ram_gb = ramBytes / 1024^3;
        end
    catch
    end

    if ispc
        [status,osRaw] = system(['powershell -NoProfile -NonInteractive -Command ', ...
            '"$o=Get-CimInstance Win32_OperatingSystem; ', ...
            '[Console]::Write($o.Version+''|''+$o.BuildNumber)"']);
        if status==0
            osParts = strsplit(strtrim(osRaw),'|');
            if numel(osParts)==2
                buildNumber = str2double(osParts{2});
                if isfinite(buildNumber) && buildNumber >= 22000
                    windowsName = 'Microsoft Windows 11';
                else
                    windowsName = 'Microsoft Windows 10';
                end
                hardware.operating_system = sprintf('%s (version %s, build %s, %s)', ...
                    windowsName,osParts{1},osParts{2},hardware.architecture);
            end
        end
        [status,cpu] = system(['powershell -NoProfile -NonInteractive -Command ', ...
            '"(Get-CimInstance Win32_Processor | Select-Object -First 1 ', ...
            '-ExpandProperty Name)"']);
        if status==0 && ~isempty(strtrim(cpu)), hardware.cpu_model=strtrim(cpu); end
        if ~isfinite(hardware.ram_gb)
            [status,ram] = system(['powershell -NoProfile -NonInteractive -Command ', ...
                '"$m=(Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory; ', ...
                '[Console]::Write(($m/1GB).ToString(''F2'',', ...
                '[Globalization.CultureInfo]::InvariantCulture))"']);
            if status==0, hardware.ram_gb=str2double(strtrim(ram)); end
        end
    end

    hardware.parallel_pool_active = false;
    if exist('gcp','file')==2
        try, hardware.parallel_pool_active = ~isempty(gcp('nocreate')); catch, end
    end
    hardware.timer_scope = ['complete planner call only; excludes environment ', ...
        'generation, plotting, and file export'];
end

function writeHardwareOutputs(H,outDir)
    names = fieldnames(H);
    values = cell(numel(names),1);
    for i=1:numel(names)
        v=H.(names{i});
        if isnumeric(v), values{i}=num2str(v); elseif islogical(v), values{i}=mat2str(v); else, values{i}=v; end
    end
    T=table(names,values,'VariableNames',{'item','value'});
    writetable(T,fullfile(outDir,'hardware_specifications.csv'));
    fid=fopen(fullfile(outDir,'hardware_specifications.txt'),'w');
    for i=1:numel(names), fprintf(fid,'%s: %s\n',names{i},values{i}); end
    fclose(fid);
end

function writeHumanReadableReport(outDir,H,B,S,seeds,opts)
    fid=fopen(fullfile(outDir,'computational_budget_report.txt'),'w');
    fprintf(fid,'COMPUTATIONAL-BUDGET REPORT\n\n');
    fprintf(fid,'Independent environments: %d\n',numel(seeds));
    fprintf(fid,'Runs per environment: %d\n',opts.N_SEED);
    fprintf(fid,'Environment seeds: %s\n\n',mat2str(seeds));
    fprintf(fid,'HARDWARE\nCPU: %s\nRAM: %.2f GB\nOS: %s\nMATLAB: %s\nParallel pool: %d\n\n', ...
        H.cpu_model,H.ram_gb,H.operating_system,H.matlab_version,H.parallel_pool_active);
    fprintf(fid,'RUNTIME AND EVALUATOR CALLS (median [Q1, Q3])\n');
    for i=1:height(S)
        fprintf(fid,'%-16s runtime %.4f [%.4f, %.4f] s; evaluatePath %.1f [%.1f, %.1f]; feasibility %.1f%%; failures %d/%d\n', ...
            S.algorithm{i},S.runtime_median_s(i),S.runtime_Q1_s(i),S.runtime_Q3_s(i), ...
            S.evaluatePath_calls_median(i),S.evaluatePath_calls_Q1(i), ...
            S.evaluatePath_calls_Q3(i),100*S.feasibility_rate(i), ...
            S.failure_count(i),S.N(i));
    end
    fprintf(fid,'\nBUDGET, CANDIDATE, SMOOTHING, AND RECOVERY RULES\n');
    for i=1:height(B)
        fprintf(fid,'\n%s\n',B.algorithm{i});
        fprintf(fid,'  Initialization: %s\n',B.initialization{i});
        fprintf(fid,'  Population/sample: %s\n',B.population_or_sample_size{i});
        fprintf(fid,'  Budget: %s; timeout: %s\n',B.iteration_node_budget{i},B.wall_clock_timeout{i});
        fprintf(fid,'  Candidate policy: %s\n',B.candidate_policy{i});
        fprintf(fid,'  Smoothing: %s\n',B.smoothing_permitted{i});
        fprintf(fid,'  Recovery: %s\n',B.local_recovery_permitted{i});
        fprintf(fid,'  Failure rule: %s\n',B.failure_rule{i});
    end
    fprintf(fid,'\nInterpretation boundary: these measurements compare complete planner configurations, not isolated update operators.\n');
    fclose(fid);
end

function value = actualWork(info,a,opts)
    switch a
        case 1, value=opts.ALAConfig.maxIter;
        case 2, value=safeGet(info,'nodesExpanded',NaN);
        case 3, value=safeGet(info,'iterations',NaN);
        case 4, value=safeGet(info,'statesExpanded',NaN);
        case 5, value=1;
    end
end
function tf = determineTimeout(info,a,opts)
    if ~ismember(a,[2 3 4]), tf=false; return; end
    elapsed=safeGet(info,'time',0);
    tf=isfinite(elapsed) && elapsed>=opts.TimeBudgetS;
end

function reason = classifyFailure(exceptionId,path,details,info,timedOut)
    if ~isempty(exceptionId), reason=['exception:',exceptionId]; return; end
    if logical(safeGet(details,'feasible',false)), reason='none'; return; end
    if timedOut, reason='timeout_without_feasible_output'; return; end
    if isempty(path) || size(path,1)<2, reason='no_path'; return; end
    if ~logical(safeGet(info,'reachedGoal',true)), reason='goal_not_reached'; return; end
    if ~isfinite(safeGet(details,'J_final',NaN)), reason='nonfinite_output'; return; end
    reason='final_hard_constraint_violation';
end

function v = safeGet(s,name,default)
    if isstruct(s) && isfield(s,name) && ~isempty(s.(name)), v=s.(name); else, v=default; end
end

function v = nestedGet(s,names,default)
    v=s;
    for i=1:numel(names)
        if ~isstruct(v) || ~isfield(v,names{i}), v=default; return; end
        v=v.(names{i});
    end
    if isempty(v), v=default; end
end

function q = qtl(x,p)
    x=sort(x(isfinite(x)));
    if isempty(x), q=NaN; return; end
    if numel(x)==1, q=x; return; end
    pos=1+(numel(x)-1)*p; lo=floor(pos); hi=ceil(pos);
    q=x(lo)+(pos-lo)*(x(hi)-x(lo));
end

function s = shortName(name)
    switch name
        case 'RA-ALA', s='RA';
        case 'Energy-A*', s='EA';
        case 'Informed-RRT*', s='RRT';
        case 'ST-EA*', s='ST';
        otherwise, s='GR';
    end
end

function id = nonemptyId(ME)
    id=ME.identifier;
    if isempty(id), id='unidentified_error'; end
end
