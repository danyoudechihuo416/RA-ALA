function results = runFigure7And8Experiment(userOpts)
%RUNFIGURE7AND8EXPERIMENT Run the primary statistical cohort.
%   The current manuscript presents these outputs as Figures 11-12. The
%   function and output stems retain their legacy fig7/fig8 names.
%   All planners use 0.75 m collision sampling during planning/search and
%   final evaluation. The fixed 10-environment design and method-specific
%   budgets match the statistical cohort in runMainExperiments.
%
%   Default use:
%       R = runFigure7And8Experiment();
%
%   The function checkpoints after every unique planner run. Re-running the
%   same command resumes an incomplete cohort and regenerates the figures
%   after all cases are complete. Other manuscript experiments are not run.

    if nargin < 1, userOpts = struct(); end
    projectDir = fileparts(mfilename('fullpath'));
    opts = localDefaults(projectDir,userOpts);
    if ~exist(opts.OutputDir,'dir'), mkdir(opts.OutputDir); end

    mapSize = 1000;
    gridStep = 10;
    windLevel = 'medium';
    riskLevel = 'dense';
    startPt = [80 80 60];
    goalPt = [900 900 60];
    algNames = {'RA-ALA','Energy-A*','Informed-RRT*','ST-EA*','Greedy'};
    nAlg = numel(algNames);
    st_time_step = 2;
    st_time_horizon = 300;
    env_seeds_used = [483 638 855 948 1041 1103 1227 1475 2312 2560];
    N_ENV = numel(env_seeds_used);
    N_SEED = 3;
    N_STAT = N_ENV*N_SEED;
    planning_collision_sample_spacing_m = 0.75;
    final_verification_spacing_m = 0.75;
    main_collision_sample_spacing_m = final_verification_spacing_m;
    main_min_collision_samples = 3;
    budget_interpretation = ['system-level comparison under the prespecified ', ...
        'method-specific algorithm budgets'];
    validation_source = validationProvenance();

    ala_cfg_stat = struct('popSize',40,'maxIter',80,'nWaypoints',8, ...
        'riskWeight',15.0,'windLookahead',3,'rescue_max_ins',12);
    metadata = struct('schema_version',2,'planning_spacing_m',0.75, ...
        'final_spacing_m',0.75,'min_samples',3, ...
        'environment_seeds',env_seeds_used,'runs_per_environment',N_SEED, ...
        'algorithm_names',{algNames},'ala_config',ala_cfg_stat, ...
        'created_at',char(datetime('now')));
    checkpointFile = fullfile(opts.OutputDir, ...
        'figure7_8_0p75m_checkpoint.mat');

    if opts.Resume && isfile(checkpointFile)
        saved = load(checkpointFile,'state','metadata');
        localValidateCheckpoint(saved.metadata,metadata);
        state = saved.state;
        fprintf('Resuming primary cohort (manuscript Figures 11-12) from: %s\n',checkpointFile);
    else
        state = localEmptyState(nAlg,N_STAT);
        fprintf('Starting a new primary cohort (manuscript Figures 11-12).\n');
    end

    fprintf('\nPrimary standalone statistical experiment (manuscript Figures 11-12)\n');
    fprintf('  Planning/search spacing: %.2f m\n', ...
        planning_collision_sample_spacing_m);
    fprintf('  Final evaluation spacing: %.2f m\n', ...
        final_verification_spacing_m);
    fprintf('  Environments: %d; RA-ALA/RRT* runs per environment: %d\n', ...
        N_ENV,N_SEED);
    fprintf('  Deterministic planners: one unique run per environment\n');
    fprintf('  Output: %s\n\n',opts.OutputDir);

    global EVAL_COUNTER;
    for env_count = 1:N_ENV
        envSeed = env_seeds_used(env_count);
        rng(envSeed,'twister');
        env = CityEnvironment(mapSize,gridStep);
        env.generate('high',windLevel,riskLevel,envSeed);
        env.setTaskPoints(startPt,goalPt);

        cmPlan = UnifiedCostModel();
        cmPlan.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
        cmPlan.setCollisionSampling(planning_collision_sample_spacing_m,3);
        cmFinal = UnifiedCostModel();
        cmFinal.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
        cmFinal.setCollisionSampling(final_verification_spacing_m,3);
        planner = PathPlanners(env,cmPlan);
        planner.setBudget(15,5000,2000);

        fprintf('  [environment %d/%d] seed=%d\n',env_count,N_ENV,envSeed);
        for s = 1:N_SEED
            col = (env_count-1)*N_SEED+s;
            algSeed = envSeed+s*53;
            state.stat_env(col) = envSeed;
            state.stat_ra_seed(col) = algSeed+11;

            for a = 1:nAlg
                if state.completed(a,col), continue; end
                stochastic = a==1 || a==3;
                if ~stochastic && s>1
                    src = col-(s-1);
                    if ~state.completed(a,src)
                        error('Figure78:MissingDeterministicSource', ...
                            'The unique deterministic trial is not complete.');
                    end
                    state = localCopyTrial(state,a,src,col);
                    state.completed(a,col) = true;
                    localSaveCheckpoint(checkpointFile,state,metadata);
                    continue;
                end

                state.stat_is_unique_trial(a,col) = true;
                actualSeed = algSeed+a*11;
                if stochastic
                    state.stat_algorithm_seed(a,col) = actualSeed;
                end
                rng(actualSeed,'twister');
                EVAL_COUNTER = 0;
                timer = tic;
                waitSchedule = [];
                try
                    switch a
                        case 1
                            [path,~,searchDetails] = runRA_ALA( ...
                                planner,cmPlan,env,startPt,goalPt,0,true,ala_cfg_stat);
                            plannerInfo = struct('reachedGoal',~isempty(path), ...
                                'stopReason','completed');
                        case 2
                            [path,~,plannerInfo] = planner.energyAStar( ...
                                startPt,goalPt,0,true);
                        case 3
                            [path,~,plannerInfo] = planner.informedRRTStar( ...
                                startPt,goalPt,0,true,1500);
                        case 4
                            [path,~,plannerInfo] = planner.timeExpandedEnergyAStar( ...
                                startPt,goalPt,0,true,st_time_step,st_time_horizon);
                            if isfield(plannerInfo,'waitBeforeSegmentS')
                                waitSchedule = plannerInfo.waitBeforeSegmentS;
                            end
                        case 5
                            [path,~,plannerInfo] = planner.greedyPlanner( ...
                                startPt,goalPt,0,true);
                    end

                    reached = isfield(plannerInfo,'reachedGoal') && ...
                        logical(plannerInfo.reachedGoal) && ~isempty(path) && ...
                        size(path,2)==3 && all(isfinite(path(:)));
                    state.stat_reached_goal(a,col) = reached;
                    state.stat_planner_success(a,col) = reached;
                    if ~reached
                        if isfield(plannerInfo,'stopReason')
                            state.stat_failure_reason{a,col} = ...
                                char(plannerInfo.stopReason);
                        else
                            state.stat_failure_reason{a,col} = 'goal_not_reached';
                        end
                    else
                        if a~=1
                            [~,searchDetails] = cmPlan.evaluatePath( ...
                                path,0,true,waitSchedule);
                        end
                        [~,finalDetails] = cmFinal.evaluatePath( ...
                            path,0,true,waitSchedule);
                        state.stat_search_evaluation_details{a,col} = searchDetails;
                        state.stat_final_evaluation_details{a,col} = finalDetails;
                        state.stat_search_J(a,col) = searchDetails.J_final;
                        state.stat_search_E(a,col) = searchDetails.E_total;
                        state.stat_search_P(a,col) = searchDetails.penalty_total;
                        state.stat_J(a,col) = finalDetails.J_final;
                        state.stat_E(a,col) = finalDetails.E_total;
                        state.stat_P(a,col) = finalDetails.penalty_total;
                        state.stat_feasible(a,col) = logical(finalDetails.feasible);
                        state.stat_paths{a,col} = path;
                        state.stat_wait_schedules{a,col} = waitSchedule;
                        if ~finalDetails.numerically_valid
                            state.stat_failure_reason{a,col} = ...
                                finalDetails.evaluation_status;
                            state.stat_J(a,col) = NaN;
                            state.stat_E(a,col) = NaN;
                            state.stat_P(a,col) = NaN;
                            state.stat_feasible(a,col) = false;
                        elseif finalDetails.feasible
                            state.stat_failure_reason{a,col} = 'none';
                        else
                            state.stat_failure_reason{a,col} = ...
                                'execution_constraint_violation';
                        end
                    end
                catch ME
                    state.stat_failure_reason{a,col} = sprintf('%s: %s', ...
                        ME.identifier,ME.message);
                    state.stat_feasible(a,col) = false;
                end
                state.stat_runtime_s(a,col) = toc(timer);
                state.stat_evaluator_calls(a,col) = EVAL_COUNTER;
                EVAL_COUNTER = [];
                state.completed(a,col) = true;
                localSaveCheckpoint(checkpointFile,state,metadata);
                fprintf('    run %d/%d, %-15s: feasible=%d, %.1f s\n', ...
                    s,N_SEED,algNames{a},state.stat_feasible(a,col), ...
                    state.stat_runtime_s(a,col));
            end
        end
    end
    EVAL_COUNTER = [];

    stat_J = state.stat_J;
    stat_E = state.stat_E;
    stat_P = state.stat_P;
    stat_search_J = state.stat_search_J;
    stat_search_E = state.stat_search_E;
    stat_search_P = state.stat_search_P;
    stat_feasible = state.stat_feasible;
    stat_env = state.stat_env;
    stat_ra_seed = state.stat_ra_seed;
    stat_algorithm_seed = state.stat_algorithm_seed;
    stat_planner_success = state.stat_planner_success;
    stat_reached_goal = state.stat_reached_goal;
    stat_failure_reason = state.stat_failure_reason;
    stat_paths = state.stat_paths;
    stat_wait_schedules = state.stat_wait_schedules;
    stat_search_evaluation_details = state.stat_search_evaluation_details;
    stat_final_evaluation_details = state.stat_final_evaluation_details;
    stat_is_unique_trial = state.stat_is_unique_trial;
    stat_runtime_s = state.stat_runtime_s;
    stat_evaluator_calls = state.stat_evaluator_calls;

    cohortFile = fullfile(opts.OutputDir,'main_experiment_cohort_0p75m.mat');
    save(cohortFile,'env_seeds_used','stat_env','stat_ra_seed', ...
        'stat_algorithm_seed','stat_J','stat_E','stat_P','stat_feasible', ...
        'stat_paths','stat_search_J','stat_search_E','stat_search_P', ...
        'stat_search_evaluation_details','stat_final_evaluation_details', ...
        'stat_planner_success','stat_reached_goal','stat_failure_reason', ...
        'stat_wait_schedules','stat_is_unique_trial','stat_runtime_s', ...
        'stat_evaluator_calls','ala_cfg_stat','N_ENV','N_SEED','N_STAT', ...
        'mapSize','gridStep','windLevel','riskLevel','startPt','goalPt', ...
        'algNames','st_time_step','st_time_horizon', ...
        'planning_collision_sample_spacing_m','final_verification_spacing_m', ...
        'main_collision_sample_spacing_m','main_min_collision_samples', ...
        'budget_interpretation','validation_source','metadata','-v7.3');

    fig7File = fullfile(opts.OutputDir,'fig7_distributional_robustness.png');
    fig7 = plotDistributionalRobustness(stat_J,stat_E,stat_feasible, ...
        stat_env,env_seeds_used,algNames,fig7File,stat_is_unique_trial);
    if ~opts.KeepFiguresOpen && isgraphics(fig7), close(fig7); end

    clusterDir = fullfile(opts.OutputDir,'cluster_statistics_output');
    clusterStats = runClusterAwareStatistics(cohortFile, ...
        struct('OutputDir',clusterDir));
    fig8File = fullfile(opts.OutputDir,'fig8_statistical_significance.png');
    fig8 = plotClusterAwareStatistics(clusterStats,fig8File);
    if ~opts.KeepFiguresOpen && isgraphics(fig8), close(fig8); end

    publishedCohort = cohortFile;
    if opts.PublishToProjectRoot
        publishedCohort = fullfile(projectDir,'main_experiment_cohort.mat');
        copyfile(cohortFile,publishedCohort,'f');
        for figureFile = {fig7File,fig8File}
            [sourceDir,stem,~] = fileparts(figureFile{1});
            for extension = {'.png','.pdf'}
                sourceFile = fullfile(sourceDir,[stem extension{1}]);
                copyfile(sourceFile,fullfile(projectDir,[stem extension{1}]),'f');
            end
        end
        fprintf('  Release cohort and manuscript Figures 11-12 synchronized to the project root.\n');
    end

    results = struct('cohort_file',publishedCohort, ...
        'archive_cohort_file',cohortFile,'figure7',fig7File, ...
        'figure8',fig8File,'cluster_statistics',clusterStats, ...
        'output_dir',opts.OutputDir,'state',state);
    fprintf('\nPrimary cohort experiment complete.\n');
    fprintf('  Cohort: %s\n',cohortFile);
    fprintf('  Manuscript Figure 11 (legacy fig7 file): %s\n',fig7File);
    fprintf('  Manuscript Figure 12 (legacy fig8 file): %s\n',fig8File);
end

function opts = localDefaults(projectDir,u)
    opts = struct('OutputDir',fullfile(projectDir, ...
        'figure7_8_0p75m_output'),'Resume',true,'KeepFiguresOpen',true, ...
        'PublishToProjectRoot',true);
    names = fieldnames(u);
    for i=1:numel(names), opts.(names{i}) = u.(names{i}); end
    validateattributes(opts.Resume,{'logical','numeric'},{'scalar'});
    opts.Resume = logical(opts.Resume);
    validateattributes(opts.KeepFiguresOpen,{'logical','numeric'},{'scalar'});
    opts.KeepFiguresOpen = logical(opts.KeepFiguresOpen);
    validateattributes(opts.PublishToProjectRoot,{'logical','numeric'},{'scalar'});
    opts.PublishToProjectRoot = logical(opts.PublishToProjectRoot);
end

function state = localEmptyState(nAlg,nStat)
    state = struct();
    state.stat_J = nan(nAlg,nStat);
    state.stat_E = nan(nAlg,nStat);
    state.stat_P = nan(nAlg,nStat);
    state.stat_search_J = nan(nAlg,nStat);
    state.stat_search_E = nan(nAlg,nStat);
    state.stat_search_P = nan(nAlg,nStat);
    state.stat_feasible = false(nAlg,nStat);
    state.stat_env = zeros(1,nStat);
    state.stat_ra_seed = zeros(1,nStat);
    state.stat_algorithm_seed = nan(nAlg,nStat);
    state.stat_planner_success = false(nAlg,nStat);
    state.stat_reached_goal = false(nAlg,nStat);
    state.stat_failure_reason = repmat({''},nAlg,nStat);
    state.stat_paths = cell(nAlg,nStat);
    state.stat_wait_schedules = cell(nAlg,nStat);
    state.stat_search_evaluation_details = cell(nAlg,nStat);
    state.stat_final_evaluation_details = cell(nAlg,nStat);
    state.stat_is_unique_trial = false(nAlg,nStat);
    state.stat_runtime_s = nan(nAlg,nStat);
    state.stat_evaluator_calls = nan(nAlg,nStat);
    state.completed = false(nAlg,nStat);
end

function state = localCopyTrial(state,a,src,dst)
    numericFields = {'stat_J','stat_E','stat_P','stat_search_J', ...
        'stat_search_E','stat_search_P','stat_feasible', ...
        'stat_algorithm_seed','stat_planner_success','stat_reached_goal', ...
        'stat_is_unique_trial','stat_runtime_s','stat_evaluator_calls'};
    cellFields = {'stat_failure_reason','stat_paths','stat_wait_schedules', ...
        'stat_search_evaluation_details','stat_final_evaluation_details'};
    for i=1:numel(numericFields)
        f=numericFields{i}; state.(f)(a,dst)=state.(f)(a,src);
    end
    for i=1:numel(cellFields)
        f=cellFields{i}; state.(f){a,dst}=state.(f){a,src};
    end
    state.stat_is_unique_trial(a,dst)=false;
end

function localValidateCheckpoint(saved,current)
    required = {'schema_version','planning_spacing_m','final_spacing_m', ...
        'environment_seeds','runs_per_environment','algorithm_names', ...
        'ala_config','validation_source'};
    for i=1:numel(required)
        if ~isfield(saved,required{i})
            error('Figure78:InvalidCheckpoint', ...
                'Checkpoint metadata is missing %s.',required{i});
        end
    end
    same = saved.schema_version==current.schema_version && ...
        saved.planning_spacing_m==current.planning_spacing_m && ...
        saved.final_spacing_m==current.final_spacing_m && ...
        saved.runs_per_environment==current.runs_per_environment && ...
        isequal(saved.environment_seeds,current.environment_seeds) && ...
        isequal(saved.algorithm_names,current.algorithm_names) && ...
        isequaln(saved.ala_config,current.ala_config) && ...
        isequaln(saved.validation_source.sha256,current.validation_source.sha256);
    if ~same
        error('Figure78:CheckpointMismatch', ...
            ['The existing checkpoint belongs to a different experiment. ', ...
            'Choose a new OutputDir.']);
    end
end

function localSaveCheckpoint(file,state,metadata)
    save(file,'state','metadata','-v7.3');
end
