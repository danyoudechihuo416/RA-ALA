function summarizeEvaluationOutcomes()
% Re-evaluate the fixed Section 5.5 paths without rerunning any planner.

projectDir = fileparts(mfilename('fullpath'));
S = load(fullfile(projectDir,'section55_same_cohort_data.mat'));
outDir = fullfile(projectDir,'evaluation_outcomes');
if ~exist(outDir,'dir'), mkdir(outDir); end

nAlg = numel(S.algNames);
nCase = numel(S.stat_env);
metricNames = {'J','E','T','R'};
J = nan(nAlg,nCase); E = J; T = J; R = J;
Pheight = J; Pstatic = J; Pdyn = J; Pnfz = J; Pbatt = J;
feasible = false(nAlg,nCase);

for c = 1:nCase
    envSeed = S.stat_env(c);
    rng(envSeed,'twister');
    env = CityEnvironment(S.mapSize,S.gridStep);
    env.generate('high',S.windLevel,S.riskLevel,envSeed);
    env.setTaskPoints(S.startPt,S.goalPt);

    for a = 1:nAlg
        path = S.stat_paths{a,c};
        if isempty(path) || size(path,1)<2, continue; end
        cm = UnifiedCostModel();
        cm.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
        cm.setCollisionSampling(1.5,3);
        [~,d] = cm.evaluatePath(path,0,true);
        J(a,c)=d.J_final; E(a,c)=d.E_total; T(a,c)=d.T_total;
        R(a,c)=d.R_dynamic; Pheight(a,c)=d.penalty_height;
        Pstatic(a,c)=d.penalty_static_collision;
        Pdyn(a,c)=d.penalty_dynamic_collision;
        Pnfz(a,c)=d.penalty_nfz; Pbatt(a,c)=d.penalty_battery;
        feasible(a,c)=d.feasible;
    end
end

tol = 1e-12;
alg = string(S.algNames(:));
height_n = sum(Pheight>tol,2); static_n = sum(Pstatic>tol,2);
dynamic_n = sum(Pdyn>tol,2); nfz_n = sum(Pnfz>tol,2);
battery_n = sum(Pbatt>tol,2);
any_n = sum(~feasible,2);
multi_n = sum((Pheight>tol)+(Pstatic>tol)+(Pdyn>tol)+(Pnfz>tol)+(Pbatt>tol)>1,2);
counts = table(alg,height_n,static_n,dynamic_n,nfz_n,battery_n,multi_n,any_n, ...
    'VariableNames',{'Algorithm','Height','Static','Dynamic','ActiveNFZ','Battery','Multiple','Any'});
writetable(counts,fullfile(outDir,'hard_violation_counts.csv'));

baseNames = alg(2:end); nBase = nAlg-1;
joint_n=zeros(nBase,1); med_J_RA=nan(nBase,1); med_J_base=nan(nBase,1);
med_E_RA=med_J_RA; med_E_base=med_J_RA; med_T_RA=med_J_RA; med_T_base=med_J_RA;
med_R_RA=med_J_RA; med_R_base=med_J_RA;
for b=1:nBase
    a=b+1; m=feasible(1,:) & feasible(a,:);
    joint_n(b)=sum(m);
    valsRA={J(1,m),E(1,m),T(1,m),R(1,m)};
    valsB ={J(a,m),E(a,m),T(a,m),R(a,m)};
    for q=1:numel(metricNames)
        eval(sprintf('med_%s_RA(b)=median(valsRA{q},''omitnan'');',metricNames{q}));
        eval(sprintf('med_%s_base(b)=median(valsB{q},''omitnan'');',metricNames{q}));
    end
end
joint = table(baseNames,joint_n,med_J_RA,med_J_base,med_E_RA,med_E_base, ...
    med_T_RA,med_T_base,med_R_RA,med_R_base, ...
    'VariableNames',{'Baseline','JointlyFeasibleN','RA_MedianJ','Baseline_MedianJ', ...
    'RA_MedianE_Wh','Baseline_MedianE_Wh','RA_MedianT_s','Baseline_MedianT_s', ...
    'RA_MedianRisk','Baseline_MedianRisk'});
writetable(joint,fullfile(outDir,'jointly_feasible_pairwise_summary.csv'));

caseTable = table();
for a=1:nAlg
    part=table(repmat(alg(a),nCase,1),S.stat_env(:),S.stat_ra_seed(:), ...
        J(a,:)',E(a,:)',T(a,:)',R(a,:)',Pheight(a,:)',Pstatic(a,:)', ...
        Pdyn(a,:)',Pnfz(a,:)',Pbatt(a,:)',feasible(a,:)', ...
        'VariableNames',{'Algorithm','EnvironmentSeed','AlgorithmSeed','J','EnergyWh', ...
        'TimeS','DynamicRisk','Pheight','Pstatic','Pdynamic','PNFZ','Pbattery','Feasible'});
    caseTable=[caseTable;part]; %#ok<AGROW>
end
writetable(caseTable,fullfile(outDir,'case_level_evaluator_outputs.csv'));
save(fullfile(outDir,'evaluation_outcomes.mat'),'counts','joint','caseTable');
disp(counts); disp(joint);
end
