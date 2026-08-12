%% Incrementally add the w/o Headwind Guidance ablation.
% Reuses the exact Full, w/o Smooth, and w/o Unified Eval results already
% stored for Section 5.5. Only the 30 windLookahead=0 searches are run.

clearvars;
clc;

old_file = 'fig9_ablation_same_cohort_data.mat';
cohort_file = 'section55_same_cohort_data.mat';
checkpoint_file = 'headwind_ablation_checkpoint.mat';

if ~isfile(old_file) || ~isfile(cohort_file)
    error('Required saved data are missing. Expected %s and %s.', old_file, cohort_file);
end

old = load(old_file);
cohort = load(cohort_file);

required_old = {'abl_J','abl_E','abl_T','abl_R','abl_Pen','abl_feasible', ...
    'ablationNames','abl_env_seeds','abl_run_seeds','cfg_abl_base'};
for k = 1:numel(required_old)
    if ~isfield(old, required_old{k})
        error('Missing variable %s in %s.', required_old{k}, old_file);
    end
end

required_cohort = {'N_ENV','N_SEED','N_STAT','mapSize','gridStep','windLevel', ...
    'riskLevel','startPt','goalPt','env_seeds_used','stat_ra_seed', ...
    'stat_J','stat_E','stat_P','stat_feasible'};
for k = 1:numel(required_cohort)
    if ~isfield(cohort, required_cohort{k})
        error('Missing variable %s in %s.', required_cohort{k}, cohort_file);
    end
end

old_names = string(old.ablationNames);
idx_full = find(old_names == "Full RA-ALA", 1);
idx_smooth = find(old_names == "w/o Smooth", 1);
idx_unified = find(old_names == "w/o Unified Eval", 1);
if isempty(idx_full) || isempty(idx_smooth) || isempty(idx_unified)
    error('Could not locate the three reusable valid variants in %s.', old_file);
end

N_ABL_ENV = cohort.N_ENV;
N_ABL_REP = cohort.N_SEED;
N_ABL = cohort.N_STAT;
abl_env_seeds = cohort.env_seeds_used;
abl_run_seeds = cohort.stat_ra_seed;
cfg_abl_base = old.cfg_abl_base;

if ~isequal(abl_env_seeds, old.abl_env_seeds) || ...
        ~isequal(abl_run_seeds, old.abl_run_seeds)
    error('Saved ablation cohort does not match the Section 5.5 cohort.');
end

head_J = nan(1,N_ABL);
head_E = nan(1,N_ABL);
head_T = nan(1,N_ABL);
head_R = nan(1,N_ABL);
head_Pen = nan(1,N_ABL);
head_feasible = false(1,N_ABL);
completed = false(1,N_ABL);

if isfile(checkpoint_file)
    cp = load(checkpoint_file);
    cp_required = {'head_J','head_E','head_T','head_R','head_Pen', ...
        'head_feasible','completed','abl_env_seeds','abl_run_seeds'};
    if all(isfield(cp,cp_required)) && isequal(cp.abl_env_seeds,abl_env_seeds) && ...
            isequal(cp.abl_run_seeds,abl_run_seeds)
        head_J = cp.head_J; head_E = cp.head_E; head_T = cp.head_T;
        head_R = cp.head_R; head_Pen = cp.head_Pen;
        head_feasible = cp.head_feasible; completed = cp.completed;
        fprintf('Resuming checkpoint: %d/%d cases already completed.\n', sum(completed), N_ABL);
    end
end

cfg_headwind = cfg_abl_base;
cfg_headwind.windLookahead = 0;

fprintf('\nIncremental ablation: w/o Headwind Guidance\n');
fprintf('Cohort: %d environments x %d seeds = %d paired cases\n', ...
    N_ABL_ENV, N_ABL_REP, N_ABL);

col = 0;
for ei = 1:N_ABL_ENV
    rng(abl_env_seeds(ei));
    env_run = CityEnvironment(cohort.mapSize, cohort.gridStep);
    env_run.generate('high', cohort.windLevel, cohort.riskLevel, abl_env_seeds(ei));
    env_run.setTaskPoints(cohort.startPt, cohort.goalPt);
    cm_run = UnifiedCostModel();
    cm_run.setEnvironment(env_run.windField, env_run.dynObstacles, env_run.heightMap);
    pl_run = PathPlanners(env_run, cm_run);
    pl_run.setBudget(15, 5000, 2000);

    for ri = 1:N_ABL_REP
        col = col + 1;
        if completed(col)
            continue;
        end
        rng(abl_run_seeds(col));
        try
            [~,~,det] = runRA_ALA(pl_run, cm_run, env_run, ...
                cohort.startPt, cohort.goalPt, 0, true, cfg_headwind);
            head_J(col) = det.J_final;
            head_E(col) = det.E_total;
            head_T(col) = det.T_total;
            head_R(col) = det.R_dynamic;
            head_Pen(col) = det.penalty_total;
            head_feasible(col) = logical(det.feasible);
        catch ME
            warning('Headwind ablation failed at environment %d run %d: %s', ei, ri, ME.message);
        end
        completed(col) = true;
        save(checkpoint_file, 'head_J','head_E','head_T','head_R','head_Pen', ...
            'head_feasible','completed','abl_env_seeds','abl_run_seeds','cfg_headwind');
        fprintf('  env %2d/%d run %d/%d: %2d/%d completed\n', ...
            ei,N_ABL_ENV,ri,N_ABL_REP,sum(completed),N_ABL);
    end
end

ablationNames = {'Full RA-ALA','w/o Smooth', ...
    'w/o Headwind Guidance','w/o Unified Eval'};
abl_J = [old.abl_J(idx_full,:); old.abl_J(idx_smooth,:); head_J; old.abl_J(idx_unified,:)];
abl_E = [old.abl_E(idx_full,:); old.abl_E(idx_smooth,:); head_E; old.abl_E(idx_unified,:)];
abl_T = [old.abl_T(idx_full,:); old.abl_T(idx_smooth,:); head_T; old.abl_T(idx_unified,:)];
abl_R = [old.abl_R(idx_full,:); old.abl_R(idx_smooth,:); head_R; old.abl_R(idx_unified,:)];
abl_Pen = [old.abl_Pen(idx_full,:); old.abl_Pen(idx_smooth,:); head_Pen; old.abl_Pen(idx_unified,:)];
abl_feasible = [old.abl_feasible(idx_full,:); old.abl_feasible(idx_smooth,:); ...
    head_feasible; old.abl_feasible(idx_unified,:)];

stat_J = cohort.stat_J;
stat_E = cohort.stat_E;
stat_P = cohort.stat_P;
stat_feasible = cohort.stat_feasible;
stat_env = cohort.stat_env;
stat_ra_seed = cohort.stat_ra_seed;

max_diff_J = max(abs(abl_J(1,:) - stat_J(1,:)),[],'omitnan');
if max_diff_J > 1e-9 || ~isequal(abl_feasible(1,:),stat_feasible(1,:))
    error('Full RA-ALA no longer matches Section 5.5.');
end

save(old_file, 'abl_J','abl_E','abl_T','abl_R','abl_Pen','abl_feasible', ...
    'ablationNames','abl_env_seeds','abl_run_seeds','cfg_abl_base', ...
    'stat_J','stat_E','stat_P','stat_feasible','stat_env','stat_ra_seed', ...
    'N_ABL_ENV','N_ABL_REP','N_ABL');

mean_J = mean(abl_J,2,'omitnan');
mean_E = mean(abl_E,2,'omitnan');
mean_T = mean(abl_T,2,'omitnan');
mean_R = mean(abl_R,2,'omitnan');
mean_Pen = mean(abl_Pen,2,'omitnan');
infeasible_rate = mean(~abl_feasible,2);
relative_J_pct = 100*(mean_J-mean_J(1))/mean_J(1);
summary_table = table(string(ablationNames(:)),mean_J,mean_E,mean_T,mean_R, ...
    mean_Pen,infeasible_rate,relative_J_pct, ...
    'VariableNames',{'Variant','MeanJ','MeanEnergyWh','MeanFlightTimeS', ...
    'MeanDynamicRisk','MeanPenalty','InfeasibleRate','RelativeJPercent'});
writetable(summary_table,'fig9_ablation_summary.csv');
disp(summary_table);

regen_fig9;
fprintf('Incremental headwind ablation and Figure 9 regeneration completed.\n');

