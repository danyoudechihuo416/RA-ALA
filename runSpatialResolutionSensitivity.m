function results = runSpatialResolutionSensitivity(cohortFile,userOpts)
%RUNSPATIALRESOLUTIONSENSITIVITY Audit 12/6/3/1.5 m collision sampling.
%   The RA-ALA path is planned once at PlanningSpacingM and then held fixed
%   while the unified evaluator is rerun at each requested spacing. This
%   isolates numerical collision-detection resolution from optimizer
%   stochasticity and avoids tripling the planning burden.
%
%   Full Section 5.5 cohort:
%       R = runSpatialResolutionSensitivity('section55_same_cohort_data.mat');
%
%   A saved cohort is strongly preferred. For a standalone run, explicitly
%   provide EnvironmentSeeds in userOpts.

    if nargin < 1 || isempty(cohortFile)
        cohortFile = 'section55_same_cohort_data.mat';
    end
    if nargin < 2, userOpts = struct(); end
    opts = localDefaults(userOpts);
    if ~exist(opts.OutputDir,'dir'), mkdir(opts.OutputDir); end

    [envSeeds,algorithmSeeds,settings,cfg,cohortSource,savedPaths] = ...
        localResolveCohort(cohortFile,opts);
    nEnv = numel(envSeeds);
    nSeed = size(algorithmSeeds,2);
    spacings = opts.SpacingsM(:)';
    nSpacing = numel(spacings);
    nCases = nEnv*nSeed;
    nRows = nCases*nSpacing;

    case_id = nan(nRows,1); environment_id = nan(nRows,1);
    environment_seed = nan(nRows,1); run_within_environment = nan(nRows,1);
    algorithm_seed = nan(nRows,1); spacing_m = nan(nRows,1);
    J = nan(nRows,1); energy_Wh = nan(nRows,1); arrival_time_s = nan(nRows,1);
    dynamic_risk = nan(nRows,1); penalty_total = nan(nRows,1);
    penalty_static = nan(nRows,1); penalty_dynamic = nan(nRows,1);
    penalty_nfz = nan(nRows,1); feasible = false(nRows,1);
    static_violation = false(nRows,1); dynamic_violation = false(nRows,1);
    nfz_violation = false(nRows,1); evaluation_time_s = nan(nRows,1);
    total_collision_subsamples = nan(nRows,1);
    total_nfz_subsamples = nan(nRows,1);
    path_points = nan(nRows,1); run_status = repmat({''},nRows,1);
    paths = cell(nCases,1);
    planning_time_s = nan(nCases,1);
    planning_status = repmat({''},nCases,1);

    fprintf('\nSpatial-resolution sensitivity analysis\n');
    fprintf('  Cohort: %s\n',cohortSource);
    fprintf('  Environments: %d; runs/environment: %d\n',nEnv,nSeed);
    fprintf('  Planning spacing: %.3g m\n',opts.PlanningSpacingM);
    fprintf('  Fixed-path evaluator spacings: %s m\n\n',mat2str(spacings));

    row = 0; caseCounter = 0;
    for ei = 1:nEnv
        envSeed = envSeeds(ei);
        rng(envSeed,'twister');
        env = CityEnvironment(settings.MapSize,settings.GridStep);
        env.generate('high',settings.WindLevel,settings.RiskLevel,envSeed);
        env.setTaskPoints(settings.Start,settings.Goal);

        for si = 1:nSeed
            caseCounter = caseCounter+1;
            algSeed = algorithmSeeds(ei,si);
            rng(algSeed,'twister');
            cmPlan = localCostModel(env,opts.PlanningSpacingM,opts.MinSamples);
            planner = PathPlanners(env,cmPlan);
            planner.setBudget(opts.TimeBudgetS,opts.NodeBudget,opts.RRTIterations);
            timer = tic;
            try
                if ~isempty(savedPaths{ei,si})
                    path = savedPaths{ei,si};
                    planning_time_s(caseCounter) = 0;
                    planning_status{caseCounter} = 'reused_saved_path';
                else
                    if opts.Quiet
                        evalc('[path,~,detPlan] = runRA_ALA(planner,cmPlan,env,settings.Start,settings.Goal,0,true,cfg);');
                    else
                        [path,~,detPlan] = runRA_ALA(planner,cmPlan,env, ...
                            settings.Start,settings.Goal,0,true,cfg);
                    end
                    planning_time_s(caseCounter) = toc(timer);
                    if isempty(path) || size(path,1)<2 || ~isfinite(detPlan.J_final)
                        error('ResolutionSensitivity:InvalidPath', ...
                            'Planner returned an empty or nonfinite path.');
                    end
                    planning_status{caseCounter} = 'ok';
                end
                if isempty(path) || size(path,1)<2
                    error('ResolutionSensitivity:InvalidPath', ...
                        'Saved or generated path is invalid.');
                end
                paths{caseCounter} = path;
            catch ME
                planning_time_s(caseCounter) = toc(timer);
                planning_status{caseCounter} = ['failed:',localExceptionId(ME)];
                path = [];
            end

            for ri = 1:nSpacing
                row = row+1;
                case_id(row) = caseCounter;
                environment_id(row) = ei;
                environment_seed(row) = envSeed;
                run_within_environment(row) = si;
                algorithm_seed(row) = algSeed;
                spacing_m(row) = spacings(ri);
                path_points(row) = size(path,1);
                if isempty(path)
                    run_status{row} = planning_status{caseCounter};
                    continue;
                end

                try
                    cm = localCostModel(env,spacings(ri),opts.MinSamples);
                    timerEval = tic;
                    [~,det] = cm.evaluatePath(path,0,true);
                    evaluation_time_s(row) = toc(timerEval);
                    J(row) = det.J_final;
                    energy_Wh(row) = det.E_total;
                    arrival_time_s(row) = det.T_total;
                    dynamic_risk(row) = det.R_dynamic;
                    penalty_total(row) = det.penalty_total;
                    penalty_static(row) = det.penalty_static_collision;
                    penalty_dynamic(row) = det.penalty_dynamic_collision;
                    penalty_nfz(row) = det.penalty_nfz;
                    feasible(row) = logical(det.feasible);
                    static_violation(row) = det.penalty_static_collision>0;
                    dynamic_violation(row) = det.penalty_dynamic_collision>0;
                    nfz_violation(row) = det.penalty_nfz>0;
                    total_collision_subsamples(row) = det.total_collision_subsamples;
                    total_nfz_subsamples(row) = det.total_nfz_subsamples;
                    run_status{row} = 'ok';
                catch ME
                    run_status{row} = ['failed:',localExceptionId(ME)];
                end
            end
            fprintf('  env %2d/%d, run %d/%d: %s\n',ei,nEnv,si,nSeed, ...
                planning_status{caseCounter});
        end
    end

    raw = table(case_id,environment_id,environment_seed, ...
        run_within_environment,algorithm_seed,spacing_m,J,energy_Wh, ...
        arrival_time_s,dynamic_risk,penalty_total,penalty_static, ...
        penalty_dynamic,penalty_nfz,feasible,static_violation, ...
        dynamic_violation,nfz_violation,evaluation_time_s, ...
        total_collision_subsamples,total_nfz_subsamples,path_points,run_status);
    summary = localSummary(raw,spacings);
    stability = localStability(raw,spacings);

    writetable(raw,fullfile(opts.OutputDir,'spatial_resolution_case_results.csv'));
    writetable(summary,fullfile(opts.OutputDir,'spatial_resolution_summary.csv'));
    writetable(stability,fullfile(opts.OutputDir,'spatial_resolution_stability.csv'));
    localWriteReport(fullfile(opts.OutputDir,'spatial_resolution_report.txt'), ...
        cohortSource,opts,summary,stability,nCases);
    save(fullfile(opts.OutputDir,'spatial_resolution_results.mat'), ...
        'raw','summary','stability','paths','planning_time_s', ...
        'planning_status','envSeeds','algorithmSeeds','settings','cfg','opts');

    results = struct('raw',raw,'summary',summary,'stability',stability, ...
        'paths',{paths},'planning_time_s',planning_time_s, ...
        'planning_status',{planning_status},'options',opts);
    fprintf('\nOutputs written to: %s\n',opts.OutputDir);
end

function opts = localDefaults(u)
    opts = struct('SpacingsM',[12 6 3 1.5 0.75],'PlanningSpacingM',1.5, ...
        'ReuseSavedPaths',true, ...
        'MinSamples',3,'N_ENV',10,'N_SEED',3,'EnvironmentSeeds',[], ...
        'AlgorithmSeeds',[],'MapSize',1000,'GridStep',10, ...
        'WindLevel','medium','RiskLevel','dense','Start',[80 80 60], ...
        'Goal',[900 900 60],'TimeBudgetS',15,'NodeBudget',5000, ...
        'RRTIterations',2000,'Quiet',true, ...
        'OutputDir',fullfile(pwd,'spatial_resolution_output'), ...
        'ALAConfig',struct('popSize',40,'maxIter',80,'nWaypoints',8, ...
        'riskWeight',15,'windLookahead',3,'rescue_max_ins',12));
    names = fieldnames(u);
    for i=1:numel(names), opts.(names{i})=u.(names{i}); end
    validateattributes(opts.SpacingsM,{'numeric'},{'vector','positive','finite'});
    if ~any(abs(opts.SpacingsM-3)<eps) || ~any(abs(opts.SpacingsM-6)<eps)
        error('ResolutionSensitivity:RequiredSpacings', ...
            'SpacingsM must include both 6 m and 3 m.');
    end
    if ~any(abs(opts.SpacingsM-opts.PlanningSpacingM)<eps)
        error('ResolutionSensitivity:MissingPlanningSpacing', ...
            'SpacingsM must include PlanningSpacingM.');
    end
end

function [envSeeds,algSeeds,S,cfg,source,savedPaths] = localResolveCohort(file,opts)
    S = struct('MapSize',opts.MapSize,'GridStep',opts.GridStep, ...
        'WindLevel',opts.WindLevel,'RiskLevel',opts.RiskLevel, ...
        'Start',opts.Start,'Goal',opts.Goal);
    cfg = opts.ALAConfig;
    if exist(file,'file')
        L = load(file);
        if ~isfield(L,'env_seeds_used')
            error('ResolutionSensitivity:MissingSeeds', ...
                '%s does not contain env_seeds_used.',file);
        end
        envSeeds = L.env_seeds_used(:)';
        envSeeds = envSeeds(1:min(opts.N_ENV,numel(envSeeds)));
        if isfield(L,'mapSize'), S.MapSize=L.mapSize; end
        if isfield(L,'gridStep'), S.GridStep=L.gridStep; end
        if isfield(L,'windLevel'), S.WindLevel=L.windLevel; end
        if isfield(L,'riskLevel'), S.RiskLevel=L.riskLevel; end
        if isfield(L,'startPt'), S.Start=L.startPt; end
        if isfield(L,'goalPt'), S.Goal=L.goalPt; end
        if isfield(L,'ala_cfg_stat'), cfg=L.ala_cfg_stat; end
        algSeeds = nan(numel(envSeeds),opts.N_SEED);
        savedPaths = cell(numel(envSeeds),opts.N_SEED);
        savedSpacingMatches = opts.ReuseSavedPaths && ...
            isfield(L,'main_collision_sample_spacing_m') && ...
            isscalar(L.main_collision_sample_spacing_m) && ...
            abs(double(L.main_collision_sample_spacing_m)-opts.PlanningSpacingM)<eps;
        if opts.ReuseSavedPaths && ~savedSpacingMatches
            warning('ResolutionSensitivity:SavedPathResolutionMismatch', ...
                ['Saved paths will not be reused because their planning ', ...
                'resolution is missing or differs from %.3g m. Paths will ', ...
                'be replanned at the requested resolution.'], ...
                opts.PlanningSpacingM);
        end
        for e=1:numel(envSeeds)
            if isfield(L,'stat_env') && isfield(L,'stat_ra_seed')
                candidates = L.stat_ra_seed(L.stat_env==envSeeds(e));
                candidates = unique(candidates(isfinite(candidates)),'stable');
            else
                candidates = [];
            end
            for s=1:opts.N_SEED
                if s<=numel(candidates), algSeeds(e,s)=candidates(s);
                else, algSeeds(e,s)=envSeeds(e)+s*53;
                end
                if savedSpacingMatches && isfield(L,'stat_paths') && ...
                        isfield(L,'stat_env') && ...
                        isfield(L,'stat_ra_seed')
                    idx=find(L.stat_env==envSeeds(e) & ...
                        L.stat_ra_seed==algSeeds(e,s),1);
                    if ~isempty(idx) && size(L.stat_paths,1)>=1
                        savedPaths{e,s}=L.stat_paths{1,idx};
                    end
                end
            end
        end
        source = char(java.io.File(file).getCanonicalPath());
    else
        if isempty(opts.EnvironmentSeeds)
            error('ResolutionSensitivity:CohortRequired', ...
                ['Cohort file not found. Run RA_ALA_demo first or provide ', ...
                'userOpts.EnvironmentSeeds explicitly.']);
        end
        envSeeds = opts.EnvironmentSeeds(:)';
        envSeeds = envSeeds(1:min(opts.N_ENV,numel(envSeeds)));
        if isempty(opts.AlgorithmSeeds)
            algSeeds = envSeeds(:)+(1:opts.N_SEED)*53;
        else
            algSeeds = opts.AlgorithmSeeds;
            algSeeds = algSeeds(1:numel(envSeeds),1:opts.N_SEED);
        end
        savedPaths = cell(numel(envSeeds),opts.N_SEED);
        source = 'explicit userOpts.EnvironmentSeeds';
    end
end

function cm = localCostModel(env,spacing,minSamples)
    cm = UnifiedCostModel();
    cm.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
    cm.setCollisionSampling(spacing,minSamples);
end

function S = localSummary(T,spacings)
    n=numel(spacings);
    spacing_m=spacings(:); N=zeros(n,1); median_J=nan(n,1);
    median_energy_Wh=nan(n,1); median_arrival_time_s=nan(n,1);
    median_dynamic_risk=nan(n,1); median_penalty=nan(n,1);
    feasible_rate=nan(n,1); static_violation_count=zeros(n,1);
    dynamic_violation_count=zeros(n,1); nfz_violation_count=zeros(n,1);
    median_evaluation_time_s=nan(n,1); median_subsamples=nan(n,1);
    for i=1:n
        m=T.spacing_m==spacings(i) & strcmp(T.run_status,'ok');
        N(i)=sum(m); median_J(i)=localMedian(T.J(m));
        median_energy_Wh(i)=localMedian(T.energy_Wh(m));
        median_arrival_time_s(i)=localMedian(T.arrival_time_s(m));
        median_dynamic_risk(i)=localMedian(T.dynamic_risk(m));
        median_penalty(i)=localMedian(T.penalty_total(m));
        feasible_rate(i)=mean(T.feasible(m));
        static_violation_count(i)=sum(T.static_violation(m));
        dynamic_violation_count(i)=sum(T.dynamic_violation(m));
        nfz_violation_count(i)=sum(T.nfz_violation(m));
        median_evaluation_time_s(i)=localMedian(T.evaluation_time_s(m));
        median_subsamples(i)=localMedian(T.total_collision_subsamples(m));
    end
    S=table(spacing_m,N,median_J,median_energy_Wh,median_arrival_time_s, ...
        median_dynamic_risk,median_penalty,feasible_rate, ...
        static_violation_count,dynamic_violation_count,nfz_violation_count, ...
        median_evaluation_time_s,median_subsamples);
end

function S = localStability(T,spacings)
    ref=min(spacings); compare=spacings(spacings~=ref); n=numel(compare);
    spacing_m=compare(:); reference_spacing_m=repmat(ref,n,1);
    paired_N=zeros(n,1); median_relative_J_change=nan(n,1);
    max_relative_J_change=nan(n,1); median_relative_energy_change=nan(n,1);
    max_relative_energy_change=nan(n,1); median_relative_arrival_change=nan(n,1);
    max_relative_arrival_change=nan(n,1); feasibility_agreement=nan(n,1);
    static_status_agreement=nan(n,1); dynamic_status_agreement=nan(n,1);
    nfz_status_agreement=nan(n,1);
    for i=1:n
        A=T(T.spacing_m==compare(i),:); B=T(T.spacing_m==ref,:);
        [ids,ia,ib]=intersect(A.case_id,B.case_id,'stable'); %#ok<ASGLU>
        ok=strcmp(A.run_status(ia),'ok') & strcmp(B.run_status(ib),'ok');
        ia=ia(ok); ib=ib(ok); paired_N(i)=numel(ia);
        d=localRelative(A.J(ia),B.J(ib));
        median_relative_J_change(i)=localMedian(d); max_relative_J_change(i)=localMax(d);
        d=localRelative(A.energy_Wh(ia),B.energy_Wh(ib));
        median_relative_energy_change(i)=localMedian(d); max_relative_energy_change(i)=localMax(d);
        d=localRelative(A.arrival_time_s(ia),B.arrival_time_s(ib));
        median_relative_arrival_change(i)=localMedian(d); max_relative_arrival_change(i)=localMax(d);
        feasibility_agreement(i)=mean(A.feasible(ia)==B.feasible(ib));
        static_status_agreement(i)=mean(A.static_violation(ia)==B.static_violation(ib));
        dynamic_status_agreement(i)=mean(A.dynamic_violation(ia)==B.dynamic_violation(ib));
        nfz_status_agreement(i)=mean(A.nfz_violation(ia)==B.nfz_violation(ib));
    end
    S=table(spacing_m,reference_spacing_m,paired_N, ...
        median_relative_J_change,max_relative_J_change, ...
        median_relative_energy_change,max_relative_energy_change, ...
        median_relative_arrival_change,max_relative_arrival_change, ...
        feasibility_agreement,static_status_agreement, ...
        dynamic_status_agreement,nfz_status_agreement);
end

function localWriteReport(file,source,opts,S,C,nCases)
    fid=fopen(file,'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'SPATIAL COLLISION-SAMPLING RESOLUTION STUDY\n\n');
    fprintf(fid,'Cohort: %s\nCases planned once: %d\n',source,nCases);
    fprintf(fid,'Planning/evaluation baseline: %.3g m\n',opts.PlanningSpacingM);
    fprintf(fid,'Design: fixed selected paths re-evaluated at %s m.\n\n',mat2str(opts.SpacingsM));
    fprintf(fid,'SUMMARY\n');
    for i=1:height(S)
        fprintf(fid,'%.3g m: N=%d, median J=%.6g, median T=%.6g s, feasible=%.1f%%, static/dynamic/NFZ=%d/%d/%d\n', ...
            S.spacing_m(i),S.N(i),S.median_J(i),S.median_arrival_time_s(i), ...
            100*S.feasible_rate(i),S.static_violation_count(i), ...
            S.dynamic_violation_count(i),S.nfz_violation_count(i));
    end
    fprintf(fid,'\nSTABILITY RELATIVE TO THE FINEST RESOLUTION\n');
    for i=1:height(C)
        fprintf(fid,'%.3g vs %.3g m: N=%d, median/max rel J=%.4g/%.4g, median/max rel T=%.4g/%.4g, feasibility agreement=%.1f%%, static/dynamic/NFZ agreement=%.1f/%.1f/%.1f%%\n', ...
            C.spacing_m(i),C.reference_spacing_m(i),C.paired_N(i), ...
            C.median_relative_J_change(i),C.max_relative_J_change(i), ...
            C.median_relative_arrival_change(i),C.max_relative_arrival_change(i), ...
            100*C.feasibility_agreement(i),100*C.static_status_agreement(i), ...
            100*C.dynamic_status_agreement(i),100*C.nfz_status_agreement(i));
    end
    fprintf(fid,'\nInterpretation boundary: this is a numerical-resolution study, not a claim of analytic continuous collision detection.\n');
end

function d=localRelative(a,b)
    d=abs(a-b)./max(abs(b),eps);
    d=d(isfinite(d));
end
function v=localMedian(x), x=x(isfinite(x)); if isempty(x), v=NaN; else, v=median(x); end, end
function v=localMax(x), x=x(isfinite(x)); if isempty(x), v=NaN; else, v=max(x); end, end
function id=localExceptionId(ME), id=ME.identifier; if isempty(id), id='unidentified_error'; end, end
