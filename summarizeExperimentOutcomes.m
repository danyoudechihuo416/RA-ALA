function outputs = summarizeExperimentOutcomes(cohortFile,outDir)
% Re-evaluate the fixed Section 5.5 paths without rerunning any planner.

projectDir = fileparts(mfilename('fullpath'));
if nargin < 1, cohortFile = fullfile(projectDir,'main_experiment_cohort.mat'); end
if nargin < 2, outDir = fullfile(projectDir,'experiment_outcome_summary'); end
S = load(cohortFile);
sampling = resolveCohortEvaluationSettings(S);
if ~exist(outDir,'dir'), mkdir(outDir); end

nAlg = numel(S.algNames);
nCase = numel(S.stat_env);
metricNames = {'J','E','T','R'};
J = nan(nAlg,nCase); E = J; T = J; R = J;
Pheight = J; Pstatic = J; Pdyn = J; Pnfz = J; Pbatt = J; Pkin = J;
feasible = false(nAlg,nCase);
evaluationStatus = repmat({'no_path'},nAlg,nCase);
numericalFailure = false(nAlg,nCase);
validTrial = true(nAlg,nCase);
if isfield(S,'stat_is_unique_trial'), validTrial=logical(S.stat_is_unique_trial); end

for c = 1:nCase
    envSeed = S.stat_env(c);
    rng(envSeed,'twister');
    env = CityEnvironment(S.mapSize,S.gridStep);
    env.generate('high',S.windLevel,S.riskLevel,envSeed);
    env.setTaskPoints(S.startPt,S.goalPt);

    for a = 1:nAlg
        if ~validTrial(a,c), continue; end
        path = S.stat_paths{a,c};
        if isempty(path) || size(path,1)<2, continue; end
        cm = UnifiedCostModel();
        cm.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
        cm.setCollisionSampling(sampling.FinalSpacingM,sampling.MinSamples);
        waitSchedule=[];
        if isfield(S,'stat_wait_schedules')
            waitSchedule=S.stat_wait_schedules{a,c};
        end
        [~,d] = cm.evaluatePath(path,0,true,waitSchedule);
        evaluationStatus{a,c}=d.evaluation_status;
        numericalFailure(a,c)=strcmp(d.evaluation_status,'time_solver_failure');
        if ~d.numerically_valid, continue; end
        J(a,c)=d.J_final; E(a,c)=d.E_total; T(a,c)=d.T_total;
        R(a,c)=d.R_dynamic; Pheight(a,c)=d.penalty_height;
        Pstatic(a,c)=d.penalty_static_collision;
        if isfield(d,'penalty_scene_entry')
            Pdyn(a,c)=d.penalty_scene_entry;
        else
            Pdyn(a,c)=d.penalty_dynamic_collision;
        end
        Pnfz(a,c)=d.penalty_nfz; Pbatt(a,c)=d.penalty_battery;
        Pkin(a,c)=d.penalty_kinematic;
        feasible(a,c)=d.feasible;
    end
end

tol = 1e-12;
alg = string(S.algNames(:));
height_n=sum((Pheight>tol)&validTrial,2); static_n=sum((Pstatic>tol)&validTrial,2);
dynamic_n=sum((Pdyn>tol)&validTrial,2); nfz_n=sum((Pnfz>tol)&validTrial,2);
battery_n=sum((Pbatt>tol)&validTrial,2); kinematic_n=sum((Pkin>tol)&validTrial,2);
trial_n=sum(validTrial,2);
any_n=sum((~feasible)&validTrial,2);
multi_n=sum(((Pheight>tol)+(Pstatic>tol)+(Pdyn>tol)+(Pnfz>tol)+(Pbatt>tol)+(Pkin>tol)>1)&validTrial,2);
counts = table(alg,trial_n,height_n,static_n,dynamic_n,nfz_n,battery_n,kinematic_n,multi_n,any_n, ...
    'VariableNames',{'Algorithm','N','Height','Static','SceneEntry','ActiveNFZ','Battery','Kinematic','Multiple','Any'});
counts.NumericalFailure=sum(numericalFailure&validTrial,2);
counts.NoPath=sum(strcmp(evaluationStatus,'no_path')&validTrial,2);
counts.AnyPhysical=sum(((Pheight>tol)|(Pstatic>tol)|(Pdyn>tol)| ...
    (Pnfz>tol)|(Pbatt>tol)|(Pkin>tol))&validTrial,2);
% Any retains its historical meaning: all unsuccessful trials, including failures.
writetable(counts,fullfile(outDir,'hard_violation_counts.csv'));

baseNames = alg(2:end); nBase = nAlg-1;
joint_n=zeros(nBase,1); joint_env_n=zeros(nBase,1); med_J_RA=nan(nBase,1); med_J_base=nan(nBase,1);
med_E_RA=med_J_RA; med_E_base=med_J_RA; med_T_RA=med_J_RA; med_T_base=med_J_RA;
med_R_RA=med_J_RA; med_R_base=med_J_RA;
for b=1:nBase
    a=b+1;
    baseIdx=1:nCase;
    % Reuse a deterministic output only for descriptive pairing with RA trials.
    for c=1:nCase
        if ~validTrial(a,c)
            idx=find(validTrial(a,:) & S.stat_env==S.stat_env(c),1);
            if ~isempty(idx), baseIdx(c)=idx; end
        end
    end
    m=validTrial(1,:) & feasible(1,:) & feasible(a,baseIdx);
    joint_n(b)=sum(m);
    joint_env_n(b)=numel(unique(S.stat_env(m)));
    valsRA={J(1,m),E(1,m),T(1,m),R(1,m)};
    valsB ={J(a,baseIdx(m)),E(a,baseIdx(m)),T(a,baseIdx(m)),R(a,baseIdx(m))};
    for q=1:numel(metricNames)
        eval(sprintf('med_%s_RA(b)=median(valsRA{q},''omitnan'');',metricNames{q}));
        eval(sprintf('med_%s_base(b)=median(valsB{q},''omitnan'');',metricNames{q}));
    end
end
joint = table(baseNames,joint_n,joint_env_n,med_J_RA,med_J_base,med_E_RA,med_E_base, ...
    med_T_RA,med_T_base,med_R_RA,med_R_base, ...
    'VariableNames',{'Baseline','JointlyFeasibleN','JointlyFeasibleEnvironments','RA_MedianJ','Baseline_MedianJ', ...
    'RA_MedianE_Wh','Baseline_MedianE_Wh','RA_MedianT_s','Baseline_MedianT_s', ...
    'RA_MedianRisk','Baseline_MedianRisk'});
writetable(joint,fullfile(outDir,'jointly_feasible_pairwise_summary.csv'));

caseTable = table();
for a=1:nAlg
    if isfield(S,'stat_algorithm_seed')
        algSeed=S.stat_algorithm_seed(a,:)';
    else
        algSeed=nan(nCase,1); if a==1, algSeed=S.stat_ra_seed(:); end
    end
    reached=false(nCase,1); plannerSuccess=false(nCase,1);
    failureReason=repmat({''},nCase,1);
    if isfield(S,'stat_reached_goal'), reached=S.stat_reached_goal(a,:)'; end
    if isfield(S,'stat_planner_success'), plannerSuccess=S.stat_planner_success(a,:)'; end
    if isfield(S,'stat_failure_reason'), failureReason=S.stat_failure_reason(a,:)'; end
    part=table(repmat(alg(a),nCase,1),S.stat_env(:),algSeed, ...
        validTrial(a,:)',reached,plannerSuccess,failureReason,J(a,:)',E(a,:)',T(a,:)',R(a,:)', ...
        Pheight(a,:)',Pstatic(a,:)',Pdyn(a,:)',Pnfz(a,:)',Pbatt(a,:)',Pkin(a,:)',feasible(a,:)', ...
        'VariableNames',{'Algorithm','EnvironmentSeed','AlgorithmSeed','UniqueTrial','ReachedGoal', ...
        'PlannerSuccess','FailureReason','J','EnergyWh','TimeS','DynamicRisk', ...
        'Pheight','Pstatic','PsceneEntry','PNFZ','Pbattery','Pkinematic','Feasible'});
    part.EvaluationStatus=evaluationStatus(a,:)';
    part.NumericalFailure=numericalFailure(a,:)';
    caseTable=[caseTable;part]; %#ok<AGROW>
end
writetable(caseTable,fullfile(outDir,'case_level_evaluator_outputs.csv'));
pairing_rule = ['Descriptive jointly feasible RA-trial pairs; a deterministic baseline ', ...
    'is reused within its environment, not treated as a new independent observation.'];
save(fullfile(outDir,'experiment_outcome_summary.mat'),'counts','joint','caseTable','sampling','pairing_rule');
outputs=struct('counts',counts,'joint',joint,'caseTable',caseTable,'pairing_rule',pairing_rule);
disp(counts); disp(joint);
end
