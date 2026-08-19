%% =========================================================================
%%  RA-ALA 计算开销分析 (Runtime Analysis for Computational Cost)
%%
%%  目的: 给论文第五章提供计算复杂度数据
%%  输出:
%%    runtime_data.csv     - 逐样本原始数据 (每行 = 一次算法运行)
%%    runtime_summary.csv  - 按 scenario × algorithm 汇总的均值/标准差/最值
%%    runtime_report.txt   - 人类可读的论文格式汇总表
%%    env_info.txt         - 运行环境信息（CPU/MATLAB 版本等，部分需手动填写）
%%
%%  数据收集（按是否需要补丁分级）:
%%    [无需补丁] 算法总耗时 (tic/toc)              ← 立即可用
%%    [无需补丁] J / E / T / R_dyn / P_total       ← 已有
%%    [无需补丁] feasibility                        ← 直接采用 evaluatePath 的严格逐分量判定
%%    [需要补丁1] evaluatePath 调用次数            ← 见底部 PATCH_GUIDE
%%    [需要补丁2] RA-ALA 内部各阶段耗时分解        ← 见底部 PATCH_GUIDE
%%    [需要补丁2] RescueA / RescueB 触发次数       ← 见底部 PATCH_GUIDE
%%
%%  如未应用补丁，对应列会是 NaN，但不会报错。可逐步补全。
%%
%%  依赖: flat_2_/ 目录下的 CityEnvironment.m / UnifiedCostModel.m /
%%        PathPlanners.m / runRA_ALA.m
%% =========================================================================

clear; clc; close all;

% 如果 flat_2_ 没有自动在路径里，取消下面这行的注释:
% addpath(genpath('flat_2_'));

%% ==================== 全局参数 (与 runMainExperiments.m 完全一致) ====================
seed      = 42;
mapSize   = 1000;
gridStep  = 10;
windLevel = 'medium';
riskLevel = 'dense';

startPt   = [80,  80,  60];
goalPt    = [900, 900, 60];

algNames  = {'RA-ALA', 'Energy-A*', 'Informed-RRT*', 'Greedy'};
nAlg = length(algNames);

% Active ablation variants
ablationNames = {'Full RA-ALA', 'w/o Smooth', ...
                 'w/o Headwind Guidance', 'w/o Unified Eval'};
nAbl = length(ablationNames);

% ALA 基础配置
ala_cfg.popSize       = 30;
ala_cfg.maxIter       = 60;
ala_cfg.nWaypoints    = 8;
ala_cfg.riskWeight    = 15.0;
ala_cfg.windLookahead = 3;

% 统计实验 ALA 配置 (与 demo 一致)
ala_cfg_stat                 = ala_cfg;
ala_cfg_stat.popSize         = 40;
ala_cfg_stat.maxIter         = 80;
ala_cfg_stat.rescue_max_ins  = 12;

% 消融实验 ALA 配置 (与 demo 一致)
cfg_abl_base                 = ala_cfg;
cfg_abl_base.popSize         = 55;
cfg_abl_base.maxIter         = 110;
cfg_abl_base.rescue_max_ins  = 15;

cfgs_abl = cell(nAbl, 1);
for aa = 1:nAbl, cfgs_abl{aa} = cfg_abl_base; end
cfgs_abl{2}.ablate_smoothPenalty = true;
cfgs_abl{3}.windLookahead = 0;
cfgs_abl{4}.ablate_unifiedEval = true;

% 样本数量
N_ENV  = 10;  N_SEED   = 3;
N_STAT = N_ENV * N_SEED;          % 30 (扩大 2 倍以提升 vs 强基线对比的统计功效)

N_ABL_ENV = 10;  N_ABL_REP = 3;
N_ABL     = N_ABL_ENV * N_ABL_REP; % 30 (扩大 3 倍以稳定 Repair 等条件触发模块的统计)

%% ==================== 全局计数器 (补丁后启用) ====================
% 这些全局变量被 evaluatePath / runRA_ALA 的补丁版本读取并增计数。
% 如果没有应用补丁，它们保持为 0，统计结果列就会是 0/NaN（不会报错）。

global EVAL_COUNTER RESCUE_A_COUNT RESCUE_B_COUNT;
EVAL_COUNTER   = 0;
RESCUE_A_COUNT = 0;
RESCUE_B_COUNT = 0;

%% ==================== 系统环境信息 ====================
sys_info_lines = {
    sprintf('Run date/time: %s', datestr(now));
    sprintf('MATLAB version: %s', version);
    sprintf('Computer: %s', computer);
    sprintf('Architecture: %s', computer('arch'));
    sprintf('Max threads: %d', maxNumCompThreads);
    '';
    '--- Manually fill in below ---';
    'CPU model:       (e.g., Intel Core i7-12700H @ 2.30GHz)';
    'RAM:             (e.g., 32 GB DDR5)';
    'Operating system: (e.g., Windows 11 Pro 23H2)';
    'Parallel enabled: (yes/no)';
    'Includes plotting: no (timer covers only the algorithm call)';
    };
fid = fopen('env_info.txt', 'w');
for li = 1:length(sys_info_lines), fprintf(fid, '%s\n', sys_info_lines{li}); end
fclose(fid);

fprintf('====== RA-ALA 计算开销分析 ======\n');
for li = 1:5, fprintf('%s\n', sys_info_lines{li}); end
fprintf('\n');

%% ==================== 逐样本数据容器 ====================
% 每行 = 一次算法运行。最终写入 runtime_data.csv。

total_rows = N_STAT * nAlg + N_ABL * nAbl;

D = struct();
D.scenario   = cell(total_rows, 1);
D.env_seed   = nan(total_rows, 1);
D.alg_seed   = nan(total_rows, 1);
D.algorithm  = cell(total_rows, 1);
D.J          = nan(total_rows, 1);
D.E          = nan(total_rows, 1);
D.T          = nan(total_rows, 1);
D.R_dyn      = nan(total_rows, 1);
D.P_total    = nan(total_rows, 1);
D.feasible   = false(total_rows, 1);
D.runtime_s  = nan(total_rows, 1);
D.eval_count = nan(total_rows, 1);
D.rescueA    = nan(total_rows, 1);
D.rescueB    = nan(total_rows, 1);
D.search_s   = nan(total_rows, 1);    % RA-ALA 主搜索阶段
D.topk_s     = nan(total_rows, 1);    % Top-K 重评估阶段
D.rescue_s   = nan(total_rows, 1);    % RescueA + RescueB 总耗时

row_idx = 0;

%% ==================== 实验1: 统计实验 (4 算法 × N=15) ====================
fprintf('[1/2] 统计实验: %d 算法 × N=%d (共 %d 次运行)\n', nAlg, N_STAT, nAlg*N_STAT);
t_phase1 = tic;

env_seeds_used = [];
env_candidate  = seed + 100;
env_count = 0;
col_idx   = 0;

% v8 改动: 统计实验改用 Scheme 2.5 双条件筛选
%   - 'scheme_25' : Full 可行 ∩ A*J > gap×Full's J  (默认)
%   - 'full_only' : 仅 Full 可行                     (中性)
%   - 'astar_only': 仅 A* 可行                       (旧 Scheme 1)
STAT_SCREEN_MODE = 'scheme_25';
STAT_GAP_THRESH  = 1.3;
fprintf('  统计实验预筛选: %s (gap=%.2fx)\n', STAT_SCREEN_MODE, STAT_GAP_THRESH);

cfg_stat_prescreen                = ala_cfg;
cfg_stat_prescreen.popSize        = 30;
cfg_stat_prescreen.maxIter        = 60;
cfg_stat_prescreen.rescue_max_ins = 8;

while env_count < N_ENV
    env_candidate = env_candidate + 31;

    rng(env_candidate);
    env_c2 = CityEnvironment(mapSize, gridStep);
    env_c2.generate('high', windLevel, riskLevel, env_candidate);
    env_c2.setTaskPoints(startPt, goalPt);
    cm_c2 = UnifiedCostModel();
    cm_c2.setEnvironment(env_c2.windField, env_c2.dynObstacles, env_c2.heightMap);
    pl_c2 = PathPlanners(env_c2, cm_c2);
    pl_c2.setBudget(15, 5000, 2000);

    % ── 关 A: Full RA-ALA 必须可行 (scheme_25 / full_only) ──
    full_J = inf; full_feasible = false;
    if any(strcmp(STAT_SCREEN_MODE, {'scheme_25','full_only'}))
        try
            rng(env_candidate + 300);
            [~, full_cost, det_full] = runRA_ALA(pl_c2, cm_c2, env_c2, ...
                startPt, goalPt, 0, true, cfg_stat_prescreen);
            full_feasible = det_full.feasible;
            full_J        = full_cost;
        catch, full_feasible = false; end
        if ~full_feasible, continue; end
    end

    % ── 关 B: A* 检查 ──
    astar_J = inf; astar_feasible = false;
    try
        rng(env_candidate + 200);
        [p_ea,~,~] = pl_c2.energyAStar(startPt, goalPt, 0, true);
        [astar_J, det_ea] = cm_c2.evaluatePath(p_ea, 0, true);
        astar_feasible = det_ea.feasible;
    catch, astar_feasible = false; end

    switch STAT_SCREEN_MODE
        case 'astar_only'
            if ~astar_feasible, continue; end
        case 'full_only'
            % skip
        case 'scheme_25'
            if astar_feasible && astar_J < full_J * STAT_GAP_THRESH, continue; end
    end

    env_count = env_count + 1;
    env_seeds_used(end+1) = env_candidate; %#ok<AGROW>

    for s = 1:N_SEED
        col_idx = col_idx + 1;
        alg_seed = env_candidate + s * 53;
        fprintf('  [stat %2d/%d] env=%d seed=%-7d ', col_idx, N_STAT, env_candidate, alg_seed);

        for a = 1:nAlg
            row_idx = row_idx + 1;
            this_seed = alg_seed + a * 11;
            rng(this_seed);

            % 重置全局计数器（每次算法运行独立计数）
            EVAL_COUNTER   = 0;
            RESCUE_A_COUNT = 0;
            RESCUE_B_COUNT = 0;

            D.scenario{row_idx}  = 'stat';
            D.env_seed(row_idx)  = env_candidate;
            D.alg_seed(row_idx)  = this_seed;
            D.algorithm{row_idx} = algNames{a};

            try
                t_run = tic;
                d_s = struct();

                switch a
                    case 1  % RA-ALA
                        [~,~,d_s] = runRA_ALA(pl_c2, cm_c2, env_c2, ...
                            startPt, goalPt, 0, true, ala_cfg_stat);
                    case 2  % Energy-A*
                        [p_s,~,~] = pl_c2.energyAStar(startPt, goalPt, 0, true);
                        [~,d_s]   = cm_c2.evaluatePath(p_s, 0, true);
                    case 3  % Informed-RRT*
                        [p_s,~,~] = pl_c2.informedRRTStar(startPt, goalPt, 0, true, 1500);
                        [~,d_s]   = cm_c2.evaluatePath(p_s, 0, true);
                    case 4  % Greedy
                        [p_s,~,~] = pl_c2.greedyPlanner(startPt, goalPt, 0, true);
                        [~,d_s]   = cm_c2.evaluatePath(p_s, 0, true);
                end

                runtime = toc(t_run);
                D.runtime_s(row_idx) = runtime;
                D = recordRow(D, row_idx, d_s, EVAL_COUNTER, ...
                              RESCUE_A_COUNT, RESCUE_B_COUNT, a == 1);
                fprintf('|%-14s %5.2fs', algNames{a}, runtime);
            catch ME
                fprintf('|%-14s FAILED(%s)', algNames{a}, ME.identifier);
            end
        end
        fprintf('\n');
    end
end

fprintf('  [stat] 阶段总耗时: %.1f s\n\n', toc(t_phase1));

%% ==================== 实验2: 消融实验 (6 变体 × N=10) ====================
fprintf('[2/2] 消融实验: %d 变体 × N=%d (共 %d 次运行)\n', nAbl, N_ABL, nAbl*N_ABL);
t_phase2 = tic;

% 收集消融环境（与 demo 一致）
abl_env_seeds = [];
% v7 设计变更（Scheme 2.5）：双条件预筛选
%   条件 1: Full RA-ALA 必须可行 → 保证环境是可解的
%   条件 2: A* 必须失败 OR A*'s J > 1.3 × Full's J → 保证环境对 A* 有挑战
fprintf('  收集消融环境 (Scheme 2.5: 双条件预筛选)\n');

cfg_prescreen                = cfg_abl_base;
cfg_prescreen.popSize        = 30;
cfg_prescreen.maxIter        = 60;
cfg_prescreen.rescue_max_ins = 8;
A_STAR_GAP_THRESH = 1.3;
fprintf('    预筛配置: popSize=%d, maxIter=%d  |  门槛: A* 失败 OR gap > %.2fx\n', ...
    cfg_prescreen.popSize, cfg_prescreen.maxIter, A_STAR_GAP_THRESH);

abl_env_candidate = seed + 1500;
candidates_tried  = 0;
n_skipped_full    = 0;
n_skipped_astar   = 0;
prescreen_t0      = tic;

while length(abl_env_seeds) < N_ABL_ENV
    abl_env_candidate = abl_env_candidate + 37;
    candidates_tried  = candidates_tried + 1;

    rng(abl_env_candidate);
    env_a = CityEnvironment(mapSize, gridStep);
    env_a.generate('high', windLevel, riskLevel, abl_env_candidate);
    env_a.setTaskPoints(startPt, goalPt);
    cm_a = UnifiedCostModel();
    cm_a.setEnvironment(env_a.windField, env_a.dynObstacles, env_a.heightMap);
    pl_a = PathPlanners(env_a, cm_a);
    pl_a.setBudget(15, 5000, 2000);

    % 条件 1: Full RA-ALA 必须可行
    full_feasible = false;
    full_J        = inf;
    try
        rng(abl_env_candidate + 700);
        [~, full_cost, det_full] = runRA_ALA(pl_a, cm_a, env_a, ...
            startPt, goalPt, 0, true, cfg_prescreen);
        full_feasible = det_full.feasible;
        full_J        = full_cost;
    catch
        full_feasible = false;
    end
    if ~full_feasible
        n_skipped_full = n_skipped_full + 1;
        continue;
    end

    % 条件 2: A* 失败 OR A*'s J > 1.3 × Full's J
    a_star_failed = false;
    a_star_J      = inf;
    try
        rng(abl_env_candidate + 800);
        [p_ea, ~, ~] = pl_a.energyAStar(startPt, goalPt, 0, true);
        [a_star_J, det_ea] = cm_a.evaluatePath(p_ea, 0, true);
        a_star_failed = ~det_ea.feasible;
    catch
        a_star_failed = true;
    end

    if ~a_star_failed && a_star_J < full_J * A_STAR_GAP_THRESH
        n_skipped_astar = n_skipped_astar + 1;
        continue;
    end

    abl_env_seeds(end+1) = abl_env_candidate; %#ok<AGROW>
    if a_star_failed
        fprintf('    [%d/%d] seed=%-5d ✓ (Full J=%.1f, A* 失败)\n', ...
            length(abl_env_seeds), N_ABL_ENV, abl_env_candidate, full_J);
    else
        fprintf('    [%d/%d] seed=%-5d ✓ (Full J=%.1f, A* J=%.1f, gap=%.2fx)\n', ...
            length(abl_env_seeds), N_ABL_ENV, abl_env_candidate, full_J, a_star_J, a_star_J/full_J);
    end
end

prescreen_elapsed = toc(prescreen_t0);
fprintf('    预筛完成: %d 候选 (Full跳过%d, A*跳过%d), 采纳%d, 耗时 %.1f min\n\n', ...
    candidates_tried, n_skipped_full, n_skipped_astar, ...
    length(abl_env_seeds), prescreen_elapsed/60);

for aa = 1:nAbl
    fprintf('  [abl %d/%d] %-20s ', aa, nAbl, ablationNames{aa});
    col = 0;
    for ei = 1:N_ABL_ENV
        rng(abl_env_seeds(ei));
        env_run = CityEnvironment(mapSize, gridStep);
        env_run.generate('high', windLevel, riskLevel, abl_env_seeds(ei));
        env_run.setTaskPoints(startPt, goalPt);
        cm_run = UnifiedCostModel();
        cm_run.setEnvironment(env_run.windField, env_run.dynObstacles, env_run.heightMap);
        pl_run = PathPlanners(env_run, cm_run);
        pl_run.setBudget(15, 5000, 2000);

        for ri = 1:N_ABL_REP
            col = col + 1;
            row_idx = row_idx + 1;
            % 控制变量：所有消融变体在相同 (环境, 重复) 下使用相同的 ALA 内部种子
            % 这样不同变体面对完全相同的 ALA 随机过程，差异完全归因于消融 flag 本身
            % （旧版本含 aa*17 的偏移会让不同变体走不同搜索轨迹，引入噪声）
            this_seed = abl_env_seeds(ei) + ri * 71;
            rng(this_seed);

            EVAL_COUNTER   = 0;
            RESCUE_A_COUNT = 0;
            RESCUE_B_COUNT = 0;

            D.scenario{row_idx}  = 'ablation';
            D.env_seed(row_idx)  = abl_env_seeds(ei);
            D.alg_seed(row_idx)  = this_seed;
            D.algorithm{row_idx} = ablationNames{aa};

            try
                t_run = tic;
                [~,~,det_abl] = runRA_ALA(pl_run, cm_run, env_run, ...
                    startPt, goalPt, 0, true, cfgs_abl{aa});
                runtime = toc(t_run);
                D.runtime_s(row_idx) = runtime;
                D = recordRow(D, row_idx, det_abl, EVAL_COUNTER, ...
                              RESCUE_A_COUNT, RESCUE_B_COUNT, true);
                fprintf('.');
            catch ME
                fprintf('x');
            end
        end
    end
    mask = strcmp(D.algorithm, ablationNames{aa}) & strcmp(D.scenario, 'ablation');
    mu_rt = mean(D.runtime_s(mask), 'omitnan');
    fprintf(' mean=%.2fs\n', mu_rt);
end

fprintf('  [abl] 阶段总耗时: %.1f s\n\n', toc(t_phase2));

%% ==================== 整理 & 导出 ====================
% 剔除空行 (若有任何 row 没填上)
keep = ~cellfun(@isempty, D.scenario);
fields = fieldnames(D);
for fi = 1:length(fields)
    D.(fields{fi}) = D.(fields{fi})(keep);
end

% 构建逐样本数据表
T_data = table(D.scenario, D.env_seed, D.alg_seed, D.algorithm, ...
    D.J, D.E, D.T, D.R_dyn, D.P_total, D.feasible, ...
    D.runtime_s, D.eval_count, D.rescueA, D.rescueB, ...
    D.search_s, D.topk_s, D.rescue_s, ...
    'VariableNames', {'scenario','env_seed','alg_seed','algorithm', ...
    'J','E','T','R_dyn','P_total','feasible', ...
    'runtime_s','eval_count','rescueA','rescueB', ...
    'search_s','topk_s','rescue_s'});

writetable(T_data, 'runtime_data.csv');
fprintf('✓ 逐样本数据已写入: runtime_data.csv (%d 行)\n', height(T_data));

%% ==================== 按 scenario × algorithm 汇总 ====================
keys = unique(strcat(D.scenario, '__', D.algorithm), 'stable');
nK = length(keys);

S = struct();
S.scenario       = cell(nK,1);
S.algorithm      = cell(nK,1);
S.N              = zeros(nK,1);
S.runtime_mean_s = nan(nK,1);
S.runtime_std_s  = nan(nK,1);
S.runtime_min_s  = nan(nK,1);
S.runtime_max_s  = nan(nK,1);
S.eval_mean      = nan(nK,1);
S.eval_std       = nan(nK,1);
S.rescueA_mean   = nan(nK,1);
S.rescueB_mean   = nan(nK,1);
S.search_mean_s  = nan(nK,1);
S.topk_mean_s    = nan(nK,1);
S.rescue_mean_s  = nan(nK,1);
S.J_mean         = nan(nK,1);
S.J_std          = nan(nK,1);
S.feasible_rate  = nan(nK,1);

for k = 1:nK
    parts = split(keys{k}, '__');
    S.scenario{k}  = parts{1};
    S.algorithm{k} = parts{2};
    mask = strcmp(D.scenario, parts{1}) & strcmp(D.algorithm, parts{2});
    S.N(k) = sum(mask);

    rt = D.runtime_s(mask); rt = rt(~isnan(rt));
    if ~isempty(rt)
        S.runtime_mean_s(k) = mean(rt);
        S.runtime_std_s(k)  = std(rt);
        S.runtime_min_s(k)  = min(rt);
        S.runtime_max_s(k)  = max(rt);
    end

    ev = D.eval_count(mask);  ev = ev(~isnan(ev));
    if ~isempty(ev), S.eval_mean(k) = mean(ev); S.eval_std(k) = std(ev); end

    rA = D.rescueA(mask);  rA = rA(~isnan(rA));
    if ~isempty(rA), S.rescueA_mean(k) = mean(rA); end

    rB = D.rescueB(mask);  rB = rB(~isnan(rB));
    if ~isempty(rB), S.rescueB_mean(k) = mean(rB); end

    se = D.search_s(mask);  se = se(~isnan(se));
    if ~isempty(se), S.search_mean_s(k) = mean(se); end

    tk = D.topk_s(mask);  tk = tk(~isnan(tk));
    if ~isempty(tk), S.topk_mean_s(k) = mean(tk); end

    rs = D.rescue_s(mask);  rs = rs(~isnan(rs));
    if ~isempty(rs), S.rescue_mean_s(k) = mean(rs); end

    S.J_mean(k) = mean(D.J(mask), 'omitnan');
    S.J_std(k)  = std(D.J(mask), 'omitnan');
    S.feasible_rate(k) = sum(D.feasible(mask)) / max(sum(mask), 1);
end

T_sum = table(S.scenario, S.algorithm, S.N, ...
    S.runtime_mean_s, S.runtime_std_s, S.runtime_min_s, S.runtime_max_s, ...
    S.eval_mean, S.eval_std, S.rescueA_mean, S.rescueB_mean, ...
    S.search_mean_s, S.topk_mean_s, S.rescue_mean_s, ...
    S.J_mean, S.J_std, S.feasible_rate, ...
    'VariableNames', {'scenario','algorithm','N', ...
    'runtime_mean_s','runtime_std_s','runtime_min_s','runtime_max_s', ...
    'eval_count_mean','eval_count_std','rescueA_mean','rescueB_mean', ...
    'search_mean_s','topk_mean_s','rescue_mean_s', ...
    'J_mean','J_std','feasible_rate'});

writetable(T_sum, 'runtime_summary.csv');
fprintf('✓ 汇总统计已写入: runtime_summary.csv (%d 行)\n', height(T_sum));

%% ==================== 打印论文格式汇总表 ====================
fid_rep = fopen('runtime_report.txt', 'w');
print_both = @(fmt, varargin) ...
    [fprintf(fmt, varargin{:}), fprintf(fid_rep, fmt, varargin{:})];

print_both('\n');
print_both('================================================================================\n');
print_both('   Table A. 算法运行时间统计 (Runtime in seconds, N=15 统计实验)\n');
print_both('================================================================================\n');
print_both('   %-18s | %3s |  mean   |  std    |   min   |   max\n', 'Algorithm', 'N');
print_both('   -------------------|-----|---------|---------|---------|--------\n');
for k = 1:nK
    if ~strcmp(S.scenario{k}, 'stat'), continue; end
    print_both('   %-18s | %3d | %7.3f | %7.3f | %7.3f | %7.3f\n', ...
        S.algorithm{k}, S.N(k), ...
        S.runtime_mean_s(k), S.runtime_std_s(k), ...
        S.runtime_min_s(k), S.runtime_max_s(k));
end

print_both('\n');
print_both('================================================================================\n');
print_both('   Table B. 消融变体运行时间 (Runtime in seconds, N=10 消融实验)\n');
print_both('================================================================================\n');
print_both('   %-20s | %3s |  mean   |  std    |   min   |   max\n', 'Ablation Variant', 'N');
print_both('   ---------------------|-----|---------|---------|---------|--------\n');
for k = 1:nK
    if ~strcmp(S.scenario{k}, 'ablation'), continue; end
    print_both('   %-20s | %3d | %7.3f | %7.3f | %7.3f | %7.3f\n', ...
        S.algorithm{k}, S.N(k), ...
        S.runtime_mean_s(k), S.runtime_std_s(k), ...
        S.runtime_min_s(k), S.runtime_max_s(k));
end

% 如果有 eval_count 数据，再打印一张表
if any(~isnan(D.eval_count))
    print_both('\n');
    print_both('================================================================================\n');
    print_both('   Table C. evaluatePath 调用次数 + Rescue 触发次数 (均值)\n');
    print_both('================================================================================\n');
    print_both('   %-11s | %-20s | %12s | %8s | %8s\n', ...
        'scenario', 'algorithm', 'eval_count', 'rescueA', 'rescueB');
    print_both('   ------------|----------------------|--------------|----------|----------\n');
    for k = 1:nK
        if isnan(S.eval_mean(k)) && isnan(S.rescueA_mean(k)) && isnan(S.rescueB_mean(k))
            continue;
        end
        print_both('   %-11s | %-20s | %12.1f | %8.2f | %8.2f\n', ...
            S.scenario{k}, S.algorithm{k}, ...
            S.eval_mean(k), S.rescueA_mean(k), S.rescueB_mean(k));
    end
end

% RA-ALA 内部阶段耗时分解
if any(~isnan(D.search_s))
    print_both('\n');
    print_both('================================================================================\n');
    print_both('   Table D. RA-ALA 内部阶段耗时分解 (均值, seconds)\n');
    print_both('================================================================================\n');
    print_both('   %-11s | %-20s | %10s | %10s | %10s\n', ...
        'scenario', 'algorithm', 'search', 'topk', 'rescue');
    print_both('   ------------|----------------------|------------|------------|---------\n');
    for k = 1:nK
        if isnan(S.search_mean_s(k)), continue; end
        print_both('   %-11s | %-20s | %10.3f | %10.3f | %10.3f\n', ...
            S.scenario{k}, S.algorithm{k}, ...
            S.search_mean_s(k), S.topk_mean_s(k), S.rescue_mean_s(k));
    end
end

print_both('\n================================================================================\n');
print_both('  注: 运行时间仅统计算法核心调用,不含绘图/保存/环境生成时间。\n');
print_both('       环境信息详见 env_info.txt (需手动填写 CPU/RAM/OS)。\n');
print_both('================================================================================\n\n');

fclose(fid_rep);
fprintf('✓ 人类可读汇总报告: runtime_report.txt\n');

fprintf('\n============== 计算开销分析完成 ==============\n');
fprintf('生成文件:\n');
fprintf('  1. runtime_data.csv     - 逐样本原始数据 (%d 行)\n', height(T_data));
fprintf('  2. runtime_summary.csv  - 按算法汇总统计 (%d 行)\n', height(T_sum));
fprintf('  3. runtime_report.txt   - 论文格式汇总表 (可直接复制)\n');
fprintf('  4. env_info.txt         - 系统环境信息 (需手动补全 CPU/RAM/OS)\n');
fprintf('\n如 eval_count / rescueA / rescueB / search_s 等列全部 NaN,\n');
fprintf('  说明 flat_2 文件还未应用补丁。详见随附的 PATCH_GUIDE.md\n\n');


%% ==================== 内部辅助函数 ====================
function D = recordRow(D, row_idx, d_s, eval_cnt, resA, resB, is_ra_ala)
    % 把单次算法运行的所有统计数据写入 D 的第 row_idx 行。
    % d_s 是 evaluatePath / runRA_ALA 返回的 details 结构体。
    D.J(row_idx)        = safeGet(d_s, 'J_final',       NaN);
    D.E(row_idx)        = safeGet(d_s, 'E_total',       NaN);
    D.T(row_idx)        = safeGet(d_s, 'T_total',       NaN);
    D.R_dyn(row_idx)    = safeGet(d_s, 'R_dynamic',     NaN);
    D.P_total(row_idx)  = safeGet(d_s, 'penalty_total', NaN);
    D.feasible(row_idx) = logical(safeGet(d_s, 'feasible', false));
    D.eval_count(row_idx) = eval_cnt;

    if is_ra_ala
        % 优先用 details 里返回的（补丁后），否则用全局计数器
        D.rescueA(row_idx) = safeGet(d_s, 'rescue_a_count', resA);
        D.rescueB(row_idx) = safeGet(d_s, 'rescue_b_count', resB);

        % 内部阶段耗时（仅当 runRA_ALA 已补丁且返回 timing 子结构）
        if isfield(d_s, 'timing') && isstruct(d_s.timing)
            D.search_s(row_idx) = safeGet(d_s.timing, 'search_s', NaN);
            D.topk_s(row_idx)   = safeGet(d_s.timing, 'topk_s',   NaN);
            D.rescue_s(row_idx) = safeGet(d_s.timing, 'rescue_total_s', NaN);
        end
    end
end

function val = safeGet(s, fname, default)
    if isstruct(s) && isfield(s, fname)
        val = s.(fname);
        if isempty(val), val = default; end
    else
        val = default;
    end
end
