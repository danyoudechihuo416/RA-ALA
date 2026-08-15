%% =========================================================================
%%  RA-ALA 论文实验脚本
%%  面向时变城市低空环境的风险感知能耗优化 ALA 三维路径规划方法
%% =========================================================================
%%
%%  实验1 — 消融实验 (验证各组件贡献)
%%    A1: 去掉平滑导向项          → 检验路径结构引导的贡献
%%    A2: 去掉逆风前瞻导向项      → 检验搜索期逆风引导的贡献
%%
%%  实验2 — 参数敏感性 (确定最优超参数)
%%    nWaypoints: 4 / 6 / 8 / 10 / 14
%%    popSize:    10 / 20 / 30 / 50
%%    maxIter:    20 / 40 / 60 / 80 / 100
%%
%%  实验3 — 时变实验 (证明时变感知的必要性)
%%    所有算法在 t=0,60,120,180,240s 五个出发时刻对比
%%
%%  实验4 — 多随机种子统计 (保证结果可靠性)
%%    每个(算法,场景)组合重复 20 次, 报告均值±标准差
%%
%%  依赖: CityEnvironment.m, UnifiedCostModel.m, PathPlanners.m
%%        (与 run_RA_ALA.m 放同一目录)
%% =========================================================================

clear; clc; close all;
warning off;

fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║  RA-ALA 论文完整实验 (4类实验 + 统计分析)            ║\n');

%% ====================================================================
%%  实时显示控制开关
%% ====================================================================
SHOW_LIVE     = true;   % 总开关
SHOW_ITER_BAR = true;   % 每轮迭代命令行进度条
REFRESH_EVERY = 5;      % 图形刷新间隔（迭代次数）
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

%% ==================== 全局配置 ====================

baseSeed  = 42;
mapSize   = 1000;
gridStep  = 10;
windLevel = 'medium';
riskLevel = 'dense';
cityLevel = 'high';        % 消融/参数/时变实验默认用 high 复杂度

startPt   = [80,  80,  60];
goalPt    = [900, 900, 60];

% 基准 cfg (完整 RA-ALA)
cfg_base.popSize       = 30;
cfg_base.maxIter       = 60;
cfg_base.nWaypoints    = 8;
cfg_base.riskWeight    = 15.0;
cfg_base.windLookahead = 3;

% 输出目录
outDir = './exp_results';
if ~exist(outDir, 'dir'), mkdir(outDir); end

%% ==================== 构建环境 (共用) ====================

fprintf('[准备] 生成城市环境...\n');
rng(baseSeed);
env = CityEnvironment(mapSize, gridStep);
env.generate(cityLevel, windLevel, riskLevel, baseSeed);
env.setTaskPoints(startPt, goalPt);

costModel = UnifiedCostModel();
costModel.setEnvironment(env.windField, env.dynObstacles, env.heightMap);

planner = PathPlanners(env, costModel);
planner.setBudget(15, 5000, 2000);

fprintf('  环境: %s城市, %d建筑, %d障碍, %d禁飞区\n\n', ...
    cityLevel, size(env.buildings,1), ...
    length(env.dynObstacles.movingObs), length(env.dynObstacles.tempNFZ));

%% ====================================================================
%%  实验1: 消融实验 — 验证各组件不可或缺
%% ====================================================================
%%  设计原理:
%%    每次只关闭一个组件, 与完整版对比.
%%    如果关闭后代价显著上升 → 该组件有贡献.
%%    使用同一种子, 差异完全来自组件的有无.
%% ====================================================================

fprintf('━━━━━━ 实验1: 消融实验 ━━━━━━\n');

% 消融变体定义
ablationNames = {'Full RA-ALA', 'w/o Smooth', 'w/o Headwind Guidance'};
nAbl = length(ablationNames);

% 为每个变体构建 cfg
cfgs_abl = cell(nAbl, 1);
for a = 1:nAbl
    cfgs_abl{a} = cfg_base;  % 从基准复制
end
cfgs_abl{2}.ablate_smoothPenalty = true;   % A1: 去平滑导向
cfgs_abl{3}.windLookahead = 0;              % A2: 去逆风前瞻导向

% 多种子运行
nSeedsAbl = 10;
abl_costs          = zeros(nAbl, nSeedsAbl);  % final_unified_cost
abl_internal_costs = zeros(nAbl, nSeedsAbl);  % ★ internal_search_cost (RA-ALA 专用)
abl_energy  = zeros(nAbl, nSeedsAbl);
abl_time    = zeros(nAbl, nSeedsAbl);
abl_risk    = zeros(nAbl, nSeedsAbl);
abl_penalty = zeros(nAbl, nSeedsAbl);

for a = 1:nAbl
    fprintf('  [%d/%d] %-22s: ', a, nAbl, ablationNames{a});
    for s = 1:nSeedsAbl
        rng(baseSeed + s * 100);
        [~, cost, det] = runRA_ALA_exp(planner, costModel, env, ...
            startPt, goalPt, 0, true, cfgs_abl{a});
        abl_costs(a, s)          = cost;           % final_unified_cost
        abl_internal_costs(a, s) = det.internal_search_cost;  % ★
        abl_energy(a, s)  = det.E_total;
        abl_time(a, s)    = det.T_total;
        abl_risk(a, s)    = det.R_dynamic;
        abl_penalty(a, s) = det.penalty_total;
    end
    fprintf('final_J=%.1f±%.1f  intern_J=%.1f±%.1f  E=%.1f  Risk=%.4f  Pen=%.2f\n', ...
        mean(abl_costs(a,:)),         std(abl_costs(a,:)), ...
        mean(abl_internal_costs(a,:)),std(abl_internal_costs(a,:)), ...
        mean(abl_energy(a,:)), ...
        mean(abl_risk(a,:)), ...
        mean(abl_penalty(a,:)));
end

% ---- 消融图: 分组柱状图 ----
fig_abl = figure('Units','centimeters','Position',[1 1 28 12],'Color','w');
ablMetrics = [mean(abl_costs,2), mean(abl_energy,2), mean(abl_risk,2)*100, mean(abl_penalty,2)];
ablStds    = [std(abl_costs,0,2), std(abl_energy,0,2), std(abl_risk,0,2)*100, std(abl_penalty,0,2)];
metricLabels = {'Cost J', 'Energy (Wh)', 'Risk (×100)', 'Penalty'};

for m = 1:4
    subplot(1, 4, m);
    b = bar(ablMetrics(:, m), 0.65); hold on;
    errorbar(1:nAbl, ablMetrics(:,m), ablStds(:,m), 'k.', 'LineWidth', 1);
    b.FaceColor = 'flat';
    colors_abl = [0.85 0.1 0.1; 0.5 0.5 0.8; 0.5 0.8 0.5];
    for i = 1:nAbl, b.CData(i,:) = colors_abl(i,:); end
    set(gca, 'XTick', 1:nAbl, 'XTickLabel', ...
        {'Full','-Smooth','-Headwind'}, ...
        'XTickLabelRotation', 25, 'FontSize', 8);
    ylabel(metricLabels{m}); grid on;
    % 标注: Full 上面画一条基准线
    yline(ablMetrics(1,m), 'r--', 'LineWidth', 0.8);
end
sgtitle('Ablation Study: Component Contribution', 'FontSize', 13, 'FontWeight', 'bold');
exportPublicationFigure(fig_abl, fullfile(outDir, 'exp1_ablation.png'));
fprintf('  → 图已保存: exp1_ablation.png\n\n');

%% ====================================================================
%%  实验2: 参数敏感性 — 确定最优超参数
%% ====================================================================
%%  设计原理:
%%    每次只变一个参数, 其余固定在基准值.
%%    找到代价和计算时间的平衡点.
%%    每个参数值重复 5 次取均值.
%% ====================================================================

fprintf('━━━━━━ 实验2: 参数敏感性 ━━━━━━\n');

nSeedsSens = 5;

% ---- 2a: nWaypoints ----
wpValues = [4, 6, 8, 10, 14];
nWP_costs = zeros(length(wpValues), nSeedsSens);
nWP_times = zeros(length(wpValues), nSeedsSens);

fprintf('  nWaypoints: ');
for wi = 1:length(wpValues)
    cfg_wp = cfg_base;
    cfg_wp.nWaypoints = wpValues(wi);
    for s = 1:nSeedsSens
        rng(baseSeed + s);
        tic;
        [~, cost, ~] = runRA_ALA_exp(planner, costModel, env, ...
            startPt, goalPt, 0, true, cfg_wp);
        nWP_costs(wi, s) = cost;
        nWP_times(wi, s) = toc;
    end
    fprintf('%d(%.1f) ', wpValues(wi), mean(nWP_costs(wi,:)));
end
fprintf('\n');

% ---- 2b: popSize ----
popValues = [10, 20, 30, 50];
pop_costs = zeros(length(popValues), nSeedsSens);
pop_times = zeros(length(popValues), nSeedsSens);

fprintf('  popSize:    ');
for pi = 1:length(popValues)
    cfg_pop = cfg_base;
    cfg_pop.popSize = popValues(pi);
    for s = 1:nSeedsSens
        rng(baseSeed + s);
        tic;
        [~, cost, ~] = runRA_ALA_exp(planner, costModel, env, ...
            startPt, goalPt, 0, true, cfg_pop);
        pop_costs(pi, s) = cost;
        pop_times(pi, s) = toc;
    end
    fprintf('%d(%.1f) ', popValues(pi), mean(pop_costs(pi,:)));
end
fprintf('\n');

% ---- 2c: maxIter ----
iterValues = [20, 40, 60, 80, 100];
iter_costs = zeros(length(iterValues), nSeedsSens);
iter_times = zeros(length(iterValues), nSeedsSens);

fprintf('  maxIter:    ');
for ii = 1:length(iterValues)
    cfg_it = cfg_base;
    cfg_it.maxIter = iterValues(ii);
    for s = 1:nSeedsSens
        rng(baseSeed + s);
        tic;
        [~, cost, ~] = runRA_ALA_exp(planner, costModel, env, ...
            startPt, goalPt, 0, true, cfg_it);
        iter_costs(ii, s) = cost;
        iter_times(ii, s) = toc;
    end
    fprintf('%d(%.1f) ', iterValues(ii), mean(iter_costs(ii,:)));
end
fprintf('\n');

% ---- 参数敏感性图: 三子图 (代价+计算时间 双Y轴) ----
fig_sens = figure('Units','centimeters','Position',[1 1 32 10],'Color','w');

subplot(1,3,1);
yyaxis left;
errorbar(wpValues, mean(nWP_costs,2), std(nWP_costs,0,2), 'o-', 'LineWidth', 1.5);
ylabel('Cost J');
yyaxis right;
plot(wpValues, mean(nWP_times,2), 's--', 'LineWidth', 1.2);
ylabel('Time (s)');
xlabel('nWaypoints'); title('(a) Waypoint Count'); grid on;
xline(8, 'k:', 'default', 'FontSize', 8);

subplot(1,3,2);
yyaxis left;
errorbar(popValues, mean(pop_costs,2), std(pop_costs,0,2), 'o-', 'LineWidth', 1.5);
ylabel('Cost J');
yyaxis right;
plot(popValues, mean(pop_times,2), 's--', 'LineWidth', 1.2);
ylabel('Time (s)');
xlabel('Population Size'); title('(b) Population Size'); grid on;
xline(30, 'k:', 'default', 'FontSize', 8);

subplot(1,3,3);
yyaxis left;
errorbar(iterValues, mean(iter_costs,2), std(iter_costs,0,2), 'o-', 'LineWidth', 1.5);
ylabel('Cost J');
yyaxis right;
plot(iterValues, mean(iter_times,2), 's--', 'LineWidth', 1.2);
ylabel('Time (s)');
xlabel('Max Iterations'); title('(c) Iteration Count'); grid on;
xline(60, 'k:', 'default', 'FontSize', 8);

sgtitle('Parameter Sensitivity Analysis', 'FontSize', 13, 'FontWeight', 'bold');
exportPublicationFigure(fig_sens, fullfile(outDir, 'exp2_sensitivity.png'));
fprintf('  → 图已保存: exp2_sensitivity.png\n\n');

%% ====================================================================
%%  实验3: 时变实验 — 证明时变感知必要性
%% ====================================================================
%%  设计原理:
%%    所有算法在相同 5 个出发时刻上运行并对比.
%%    RA-ALA 应当在不同出发时刻产生不同路径 (时变适应),
%%    基线算法在不同时刻的代价波动更大 (不适应时变).
%%    每个(算法,时刻)组合重复 5 次.
%% ====================================================================

fprintf('━━━━━━ 实验3: 时变实验 ━━━━━━\n');

departureTimes = [0, 60, 120, 180, 240];
nDepart = length(departureTimes);
algNames = {'RA-ALA', 'Energy-A*', 'Informed-RRT*', 'Greedy'};
nAlg = length(algNames);
nSeedsTV = 5;

% 结果矩阵: [nAlg x nDepart x nSeeds]
% tv_costs         = final_unified_cost (所有算法统一 evaluatePath 口径)
% tv_internal_costs= internal_search_cost (仅 RA-ALA 有意义; 基线填 NaN)
tv_costs          = zeros(nAlg, nDepart, nSeedsTV);
tv_internal_costs = NaN(nAlg, nDepart, nSeedsTV);  % ★
tv_energy = zeros(nAlg, nDepart, nSeedsTV);
tv_risk   = zeros(nAlg, nDepart, nSeedsTV);

for d = 1:nDepart
    t_dep = departureTimes(d);
    fprintf('  t=%3ds: ', t_dep);
    for a = 1:nAlg
        for s = 1:nSeedsTV
            rng(baseSeed + s + a*10);
            switch a
                case 1  % RA-ALA
                    [~, c, det, ~] = runRA_ALA_exp(planner, costModel, env, ...
                        startPt, goalPt, t_dep, true, cfg_base);
                    % ★ 同时记录两种代价
                    tv_costs(a, d, s)          = det.J_final;   % final_unified
                    tv_internal_costs(a, d, s) = det.internal_search_cost;
                case 2  % Energy-A*
                    [p, ~, ~] = planner.energyAStar(startPt, goalPt, t_dep, true);
                    [c, det] = costModel.evaluatePath(p, t_dep, true);
                    tv_costs(a, d, s) = c;
                case 3  % Informed-RRT*
                    [p, ~, ~] = planner.informedRRTStar(startPt, goalPt, t_dep, true, 2000);
                    [c, det] = costModel.evaluatePath(p, t_dep, true);
                    tv_costs(a, d, s) = c;
                case 4  % Greedy
                    [p, ~, ~] = planner.greedyPlanner(startPt, goalPt, t_dep, true);
                    [c, det] = costModel.evaluatePath(p, t_dep, true);
                    tv_costs(a, d, s) = c;
            end
            tv_energy(a, d, s) = det.E_total;
            tv_risk(a, d, s)   = det.R_dynamic;
        end
        if a == 1
            fprintf('%s=%.0f(i:%.0f)  ', algNames{a}, ...
                mean(tv_costs(a,d,:)), mean(tv_internal_costs(a,d,:),'omitnan'));
        else
            fprintf('%s=%.0f  ', algNames{a}, mean(tv_costs(a,d,:)));
        end
    end
    fprintf('\n');
end

% ---- 时变图: 折线图 (4算法, X=出发时刻, Y=代价) + 误差带 ----
algColors = [0.85 0.1 0.1; 0.0 0.45 0.74; 0.47 0.67 0.19; 0.7 0.7 0.7];
algMarkers = {'o', 's', 'd', '^'};

fig_tv = figure('Units','centimeters','Position',[1 1 28 12],'Color','w');

metricTV = {tv_costs, tv_energy, tv_risk * 100};
metricTVNames = {'Cost J', 'Energy (Wh)', 'Risk (×100)'};

for m = 1:3
    subplot(1, 3, m);
    hold on;
    for a = 1:nAlg
        mu  = mean(metricTV{m}(a, :, :), 3);
        sd  = std(metricTV{m}(a, :, :), 0, 3);
        errorbar(departureTimes, mu, sd, ['-' algMarkers{a}], ...
            'Color', algColors(a,:), 'LineWidth', 1.5, 'MarkerFaceColor', algColors(a,:));
    end
    xlabel('Departure Time (s)');
    ylabel(metricTVNames{m});
    title(metricTVNames{m});
    grid on;
    if m == 1
        legend(algNames, 'Location', 'best', 'FontSize', 7);
    end
end
sgtitle('Time-Varying Performance: All Algorithms × 5 Departure Times', ...
    'FontSize', 13, 'FontWeight', 'bold');
exportPublicationFigure(fig_tv, fullfile(outDir, 'exp3_timevarying.png'));
fprintf('  → 图已保存: exp3_timevarying.png\n\n');

%% ====================================================================
%%  实验4: 多随机种子统计 — 保证结论可靠
%% ====================================================================
%%  设计原理:
%%    每个算法在 3 种城市复杂度 × 20 个随机种子下运行.
%%    报告 均值±标准差, 中位数, 最优/最差.
%%    绘制箱线图, 供论文附表使用.
%% ====================================================================

fprintf('━━━━━━ 实验4: 多种子统计 (3复杂度×4算法×20种子) ━━━━━━\n');

cityLevels = {'low', 'medium', 'high'};
nCity = length(cityLevels);
nSeedsStat = 20;

% 结果: [nCity x nAlg x nSeeds] × 4 指标
% stat_cost          = final_unified_cost (统一 evaluatePath 口径)
% stat_internal_cost = internal_search_cost (仅 RA-ALA 有意义; 基线 NaN)
stat_cost          = zeros(nCity, nAlg, nSeedsStat);
stat_internal_cost = NaN(nCity, nAlg, nSeedsStat);   % ★
stat_energy = zeros(nCity, nAlg, nSeedsStat);
stat_time   = zeros(nCity, nAlg, nSeedsStat);
stat_risk   = zeros(nCity, nAlg, nSeedsStat);

for c = 1:nCity
    fprintf('  [%s] ', cityLevels{c});

    % 为每种复杂度生成环境
    rng(baseSeed);
    env_c = CityEnvironment(mapSize, gridStep);
    env_c.generate(cityLevels{c}, windLevel, riskLevel, baseSeed);
    env_c.setTaskPoints(startPt, goalPt);

    cm_c = UnifiedCostModel();
    cm_c.setEnvironment(env_c.windField, env_c.dynObstacles, env_c.heightMap);
    pl_c = PathPlanners(env_c, cm_c);
    pl_c.setBudget(15, 5000, 2000);

    for a = 1:nAlg
        for s = 1:nSeedsStat
            rng(baseSeed + s * 7 + a * 13);
            if exist('SHOW_LIVE','var') && SHOW_LIVE && SHOW_ITER_BAR
                pct=s/nSeedsStat; bar_len=20;
                filled=round(pct*bar_len);
                fprintf('\r    [%s%s] 种子 %2d/%d  算法: %-14s', ...
                    repmat('█',1,filled),repmat('░',1,bar_len-filled), ...
                    s,nSeedsStat,algNames{a});
            end
            switch a
                case 1
                    [~, cost, det] = runRA_ALA_exp(pl_c, cm_c, env_c, ...
                        startPt, goalPt, 0, true, cfg_base);
                    stat_cost(c, a, s)          = det.J_final;   % final_unified
                    stat_internal_cost(c, a, s) = det.internal_search_cost;  % ★
                case 2
                    [p,~,~] = pl_c.energyAStar(startPt, goalPt, 0, true);
                    [cost, det] = cm_c.evaluatePath(p, 0, true);
                    stat_cost(c, a, s) = cost;
                case 3
                    [p,~,~] = pl_c.informedRRTStar(startPt, goalPt, 0, true, 2000);
                    [cost, det] = cm_c.evaluatePath(p, 0, true);
                    stat_cost(c, a, s) = cost;
                case 4
                    [p,~,~] = pl_c.greedyPlanner(startPt, goalPt, 0, true);
                    [cost, det] = cm_c.evaluatePath(p, 0, true);
                    stat_cost(c, a, s) = cost;
            end
            stat_energy(c, a, s) = det.E_total;
            stat_time(c, a, s)   = det.T_total;
            stat_risk(c, a, s)   = det.R_dynamic;
        end
        if exist('SHOW_LIVE','var') && SHOW_LIVE && SHOW_ITER_BAR
            fprintf('\n'); % 换行，结束进度条
        end
        if a == 1
            fprintf('%s=%.0f±%.0f(i:%.0f)  ', algNames{a}, ...
                mean(stat_cost(c,a,:)), std(stat_cost(c,a,:)), ...
                mean(stat_internal_cost(c,a,:),'omitnan'));
        else
            fprintf('%s=%.0f±%.0f  ', algNames{a}, ...
                mean(stat_cost(c,a,:)), std(stat_cost(c,a,:)));
        end
    end
    fprintf('\n');
end

% ---- 箱线图: 每种复杂度一行, 4指标各一列 ----
fig_box = figure('Units','centimeters','Position',[1 1 32 24],'Color','w');
metricStatNames = {'Cost J', 'Energy (Wh)', 'Time (s)', 'Risk'};

for c = 1:nCity
    for m = 1:4
        subplot(nCity, 4, (c-1)*4 + m);

        % 选指标
        switch m
            case 1, D = squeeze(stat_cost(c,:,:))';
            case 2, D = squeeze(stat_energy(c,:,:))';
            case 3, D = squeeze(stat_time(c,:,:))';
            case 4, D = squeeze(stat_risk(c,:,:))';
        end

        boxplot(D, 'Labels', {'RA','E-A*','RRT*','Grd'}, ...
            'Colors', algColors, 'Widths', 0.6);
        grid on;
        set(gca, 'FontSize', 7);

        if c == 1, title(metricStatNames{m}, 'FontSize', 10); end
        if m == 1, ylabel(cityLevels{c}, 'FontSize', 10, 'FontWeight', 'bold'); end
    end
end
sgtitle(sprintf('Multi-Seed Statistics (%d seeds per config)', nSeedsStat), ...
    'FontSize', 13, 'FontWeight', 'bold');
exportPublicationFigure(fig_box, fullfile(outDir, 'exp4_boxplot.png'));
fprintf('  → 图已保存: exp4_boxplot.png\n\n');

% ---- 汇总表 (论文用) ----
fprintf('╔════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  实验4 汇总: 均值 ± 标准差  (20 seeds)                          ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════╝\n');
fprintf('%-8s %-15s %12s %12s %12s %12s\n', '复杂度', '算法', 'Cost J', 'Energy Wh', 'Time s', 'Risk');
fprintf('%s\n', repmat('─', 1, 72));
for c = 1:nCity
    for a = 1:nAlg
        fprintf('%-8s %-15s %5.1f±%-5.1f %5.1f±%-5.1f %5.1f±%-5.1f %6.4f±%.4f\n', ...
            cityLevels{c}, algNames{a}, ...
            mean(stat_cost(c,a,:)),   std(stat_cost(c,a,:)), ...
            mean(stat_energy(c,a,:)), std(stat_energy(c,a,:)), ...
            mean(stat_time(c,a,:)),   std(stat_time(c,a,:)), ...
            mean(stat_risk(c,a,:)),   std(stat_risk(c,a,:)));
    end
    if c < nCity, fprintf('%s\n', repmat('·', 1, 72)); end
end

%% ====================================================================
%%  实验5: 基线对比深度分析
%% ====================================================================
%%  基于实验4的多种子数据, 进行:
%%    (1) 逐指标改进率表: RA-ALA 比每个基线好多少%
%%    (2) Wilcoxon signed-rank 配对检验 (同种子 → 配对)
%%    (3) Cliff's delta 效应量
%%    (4) 基线弱点归因: 逐维度分析每个基线为什么差
%%    (5) 消融 vs 基线交叉对照: 消融退化到哪个基线的水平
%%    (6) 综合对比雷达图
%% ====================================================================

fprintf('\n━━━━━━ 实验5: 基线对比深度分析 ━━━━━━\n');

metricStatNames = {'Cost J', 'Energy (Wh)', 'Time (s)', 'Risk'};
nMetrics = 4;

% 将四个指标整合到一个 4-D 数组: [nCity x nAlg x nSeeds x nMetrics]
allStat = cat(4, stat_cost, stat_energy, stat_time, stat_risk);

% ---- (1) 改进率表 ----
fprintf('\n╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  RA-ALA vs 基线: 改进率 (%%)  (正值 = RA-ALA 更优)                    ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════════════╝\n');
fprintf('%-8s %-16s %12s %12s %12s %12s\n', ...
    '复杂度', '基线', 'Cost', 'Energy', 'Time', 'Risk');
fprintf('%s\n', repmat('─', 1, 72));

improv_all = zeros(nCity, nAlg-1, nMetrics);  % [nCity x 3基线 x 4指标]

for c = 1:nCity
    for b = 2:nAlg   % b=2,3,4 对应三个基线
        for m = 1:nMetrics
            ra_vals   = squeeze(allStat(c, 1, :, m));   % RA-ALA
            base_vals = squeeze(allStat(c, b, :, m));   % 基线
            % 改进率 = (基线 - RA-ALA) / 基线 × 100%
            % 逐种子计算再取均值, 更稳健
            pairwise_improv = (base_vals - ra_vals) ./ max(abs(base_vals), 1e-6) * 100;
            improv_all(c, b-1, m) = mean(pairwise_improv);
        end
        fprintf('%-8s %-16s %+10.1f%%  %+10.1f%%  %+10.1f%%  %+10.1f%%\n', ...
            cityLevels{c}, algNames{b}, ...
            improv_all(c, b-1, 1), improv_all(c, b-1, 2), ...
            improv_all(c, b-1, 3), improv_all(c, b-1, 4));
    end
    if c < nCity, fprintf('%s\n', repmat('·', 1, 72)); end
end

% 跨城市平均改进率
fprintf('%s\n', repmat('═', 1, 72));
fprintf('%-8s %-16s %12s %12s %12s %12s\n', ...
    '平均', '', 'Cost', 'Energy', 'Time', 'Risk');
for b = 2:nAlg
    avg_imp = squeeze(mean(improv_all(:, b-1, :), 1));
    fprintf('%-8s %-16s %+10.1f%%  %+10.1f%%  %+10.1f%%  %+10.1f%%\n', ...
        '平均', algNames{b}, avg_imp(1), avg_imp(2), avg_imp(3), avg_imp(4));
end

% ---- (2) Wilcoxon signed-rank 配对检验 + (3) Cliff's delta ----
fprintf('\n╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  统计显著性检验: Wilcoxon signed-rank + Cliff''s delta               ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════════════╝\n');
fprintf('%-8s %-16s %-8s %12s %10s %10s\n', ...
    '复杂度', '基线', '指标', 'p-value', 'Cliff δ', '效应');
fprintf('%s\n', repmat('─', 1, 72));

pval_all   = zeros(nCity, nAlg-1, nMetrics);
delta_all  = zeros(nCity, nAlg-1, nMetrics);

for c = 1:nCity
    for b = 2:nAlg
        for m = 1:nMetrics
            ra_vals   = squeeze(allStat(c, 1, :, m));
            base_vals = squeeze(allStat(c, b, :, m));

            % Wilcoxon signed-rank (手动实现, 不依赖 toolbox)
            pval_all(c, b-1, m) = signrankTest_local(ra_vals, base_vals);

            % Cliff's delta
            delta_all(c, b-1, m) = cliffsDelta_local(ra_vals, base_vals);

            fprintf('%-8s %-16s %-8s %12s %+9.3f  %s\n', ...
                cityLevels{c}, algNames{b}, metricStatNames{m}, ...
                formatPval_local(pval_all(c, b-1, m)), ...
                delta_all(c, b-1, m), ...
                interpretDelta_local(delta_all(c, b-1, m)));
        end
        if b < nAlg || c < nCity
            fprintf('%s\n', repmat('·', 1, 72));
        end
    end
end

% ---- (4) 基线弱点归因分析 ----
fprintf('\n╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  基线弱点归因分析 (基于 high 复杂度的逐维度差距)                     ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════════════╝\n');

weaknessReasons = {
    'Energy-A*',      {'使用栅格图搜索, 受限于离散节点分辨率'; ...
                       '不做风场时变感知, 无法利用顺风/避逆风'; ...
                       '无显式动态风险检测, 依赖代价模型的间接惩罚'};
    'Informed-RRT*',  {'随机采样本质导致路径随机性大, 标准差高'; ...
                       '每次运行结果不同, 可靠性差于确定性优化'; ...
                       '碰撞检测使用固定时刻t, 不随飞行进度推进'};
    'Greedy',         {'纯距离贪心, 完全不考虑能耗/风场/风险'; ...
                       '路径沿栅格节点贪心前进, 易陷入局部陷阱'; ...
                       '无任何优化过程, 解质量最差但速度最快'}
};

for b = 1:size(weaknessReasons, 1)
    algName = weaknessReasons{b, 1};
    reasons = weaknessReasons{b, 2};
    bIdx = find(strcmp(algNames, algName));
    if isempty(bIdx), continue; end

    % 取 high 复杂度 (c=3) 的数据
    ra_mu = squeeze(mean(allStat(3, 1, :, :), 3));     % [1 x 4]
    bl_mu = squeeze(mean(allStat(3, bIdx, :, :), 3));   % [1 x 4]

    % 找该基线最弱的指标 (改进率最大的那个)
    pct = (bl_mu - ra_mu) ./ max(abs(bl_mu), 1e-6) * 100;
    [worstPct, worstIdx] = max(pct);

    fprintf('\n  ★ %s (最大劣势: %s, 差 %.1f%%)\n', algName, metricStatNames{worstIdx}, worstPct);
    for ri = 1:length(reasons)
        fprintf('    - %s\n', reasons{ri});
    end
end

% ---- (5) 消融 vs 基线交叉对照 ----
fprintf('\n╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  消融退化 vs 基线水平对照 (Cost J)                                   ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════════════╝\n');
fprintf('  目的: 验证每个消融变体退化后是否接近某个基线的性能\n\n');

% 消融数据: abl_costs [nAbl x nSeedsAbl], ablationNames
% 基线数据: stat_cost [nCity x nAlg x nSeedsStat], 取 high (c=3)
abl_mu = mean(abl_costs, 2);          % [nAbl x 1]
base_mu_high = squeeze(mean(stat_cost(3, :, :), 3))';  % [nAlg x 1]

fprintf('  %-24s  %10s   最接近的基线\n', '配置', 'Cost J');
fprintf('  %s\n', repmat('─', 1, 60));
fprintf('  %-24s  %10.1f   (基准)\n', 'Full RA-ALA', abl_mu(1));

for a = 2:length(ablationNames)
    % 找最接近的基线
    diffs_to_base = abs(abl_mu(a) - base_mu_high(2:end));
    [minDiff, closestIdx] = min(diffs_to_base);
    closestName = algNames{closestIdx + 1};
    pctDeg = (abl_mu(a) - abl_mu(1)) / abl_mu(1) * 100;
    fprintf('  %-24s  %10.1f   ≈ %s (差%.1f, 退化%.1f%%)\n', ...
        ablationNames{a}, abl_mu(a), closestName, minDiff, pctDeg);
end

fprintf('\n  基线参考值 (high 复杂度):\n');
for a = 1:nAlg
    fprintf('    %-16s = %.1f\n', algNames{a}, base_mu_high(a));
end

% ---- (6) 综合对比图: 分组柱状图 + 改进率标注 ----
fig_comp = figure('Units','centimeters','Position',[1 1 32 20],'Color','w');
algColors_comp = [0.85 0.1 0.1; 0.0 0.45 0.74; 0.47 0.67 0.19; 0.7 0.7 0.7];

for m = 1:nMetrics
    subplot(2, 2, m);
    hold on;
    set(gca, 'FontSize', 9, 'FontName', 'Times New Roman');

    % 数据: [nCity x nAlg] 均值
    barMu  = zeros(nCity, nAlg);
    barStd = zeros(nCity, nAlg);
    for c = 1:nCity
        for a = 1:nAlg
            barMu(c, a)  = mean(allStat(c, a, :, m));
            barStd(c, a) = std(allStat(c, a, :, m));
        end
    end

    hb = bar(barMu, 'grouped');
    for a = 1:nAlg
        hb(a).FaceColor = algColors_comp(a, :);
    end

    % 误差棒
    nG = nCity; nB = nAlg;
    gW = min(0.8, nB/(nB+1.5));
    for a = 1:nB
        xPos = (1:nG) - gW/2 + (2*a-1)*gW/(2*nB);
        errorbar(xPos, barMu(:,a), barStd(:,a), 'k.', 'LineWidth', 0.8);

        % 在基线柱子上标注 vs RA-ALA 的改进率
        if a > 1
            for c = 1:nCity
                imp = improv_all(c, a-1, m);
                if abs(imp) > 0.5
                    text(xPos(c), barMu(c,a) + barStd(c,a) + max(barMu(:))*0.02, ...
                        sprintf('%+.0f%%', imp), ...
                        'HorizontalAlignment','center', 'FontSize', 6, ...
                        'Color', [0.1 0.5 0.1], 'FontWeight', 'bold');
                end
            end
        end
    end

    set(gca, 'XTick', 1:nCity, 'XTickLabel', {'Low','Medium','High'});
    ylabel(metricStatNames{m}, 'FontSize', 10);
    grid on;

    if m == 1
        legend(algNames, 'Location', 'northwest', 'FontSize', 7);
    end
    title(metricStatNames{m}, 'FontSize', 11, 'FontWeight', 'bold');
end

sgtitle({'RA-ALA vs Baselines: Comprehensive Comparison', ...
         sprintf('(%d seeds per config, %% = improvement over baseline)', nSeedsStat)}, ...
    'FontSize', 13, 'FontWeight', 'bold');
exportPublicationFigure(fig_comp, fullfile(outDir, 'exp5_baseline_comparison.png'));
fprintf('\n  → 图已保存: exp5_baseline_comparison.png\n');

% ---- (7) Cliff's delta 热力图 ----
fig_delta = figure('Units','centimeters','Position',[1 1 28 14],'Color','w');

for c = 1:nCity
    subplot(1, 3, c);
    % deltaMatrix: [3基线 x 4指标]
    deltaMatrix = squeeze(delta_all(c, :, :));  % [nAlg-1 x nMetrics]

    imagesc(deltaMatrix);
    colorbar;
    % 蓝→白→红 色图: 蓝色=RA-ALA更好(负delta), 红色=基线更好
    colormap(gca, makeRBColormap());
    caxis([-1, 1]);

    set(gca, 'XTick', 1:nMetrics, 'XTickLabel', {'Cost','Energy','Time','Risk'}, ...
        'YTick', 1:nAlg-1, 'YTickLabel', algNames(2:end), 'FontSize', 8);

    % 标注数值 + 显著性星号
    for bi = 1:nAlg-1
        for mi = 1:nMetrics
            dv = deltaMatrix(bi, mi);
            pv = pval_all(c, bi, mi);
            starStr = '';
            if pv < 0.001,     starStr = '***';
            elseif pv < 0.01,  starStr = '**';
            elseif pv < 0.05,  starStr = '*';
            end
            txtColor = 'k';
            if abs(dv) > 0.6, txtColor = 'w'; end
            text(mi, bi, sprintf('%.2f%s', dv, starStr), ...
                'HorizontalAlignment','center', 'FontSize', 8, ...
                'Color', txtColor, 'FontWeight', 'bold');
        end
    end

    title(cityLevels{c}, 'FontSize', 11, 'FontWeight', 'bold');
    if c == 1, ylabel('Baseline', 'FontSize', 10); end
end

sgtitle({'Cliff''s δ Effect Size: RA-ALA vs Each Baseline', ...
         '(blue = RA-ALA better, * p<.05, ** p<.01, *** p<.001)'}, ...
    'FontSize', 12, 'FontWeight', 'bold');
exportPublicationFigure(fig_delta, fullfile(outDir, 'exp5_cliff_delta_heatmap.png'));
fprintf('  → 图已保存: exp5_cliff_delta_heatmap.png\n');

% ---- (8) 雷达图: high 复杂度下四算法四维度 ----
fig_radar = figure('Units','centimeters','Position',[1 1 18 16],'Color','w');
hold on;

% 取 high 复杂度 (c=3), 归一化到 [0,1]: 0=最好, 1=最差
radarMu = zeros(nAlg, nMetrics);
for a = 1:nAlg
    for m = 1:nMetrics
        radarMu(a, m) = mean(allStat(3, a, :, m));
    end
end
% 归一化
radarNorm = zeros(size(radarMu));
for m = 1:nMetrics
    mn = min(radarMu(:, m));
    mx = max(radarMu(:, m));
    if mx - mn > 1e-6
        radarNorm(:, m) = (radarMu(:, m) - mn) / (mx - mn);
    end
end

% 绘制雷达/蜘蛛图 (用极坐标近似)
angles = linspace(0, 2*pi, nMetrics+1);
angles = angles(1:end-1);

for a = 1:nAlg
    vals = radarNorm(a, :);
    vals_closed = [vals, vals(1)];
    angles_closed = [angles, angles(1)];

    [xr, yr] = pol2cart(angles_closed, vals_closed);
    if a == 1
        fill(xr, yr, algColors_comp(a,:), 'FaceAlpha', 0.2, 'EdgeColor', algColors_comp(a,:), 'LineWidth', 2.5);
    else
        plot(xr, yr, '-', 'Color', [algColors_comp(a,:) 0.7], 'LineWidth', 1.5);
    end
end

% 坐标轴标签
for m = 1:nMetrics
    [lx, ly] = pol2cart(angles(m), 1.15);
    text(lx, ly, metricStatNames{m}, 'HorizontalAlignment', 'center', ...
        'FontSize', 10, 'FontWeight', 'bold');
end

% 画同心圆参考线
for r = [0.25, 0.5, 0.75, 1.0]
    theta_ref = linspace(0, 2*pi, 60);
    [xref, yref] = pol2cart(theta_ref, r);
    plot(xref, yref, ':', 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
end
% 画辐射线
for m = 1:nMetrics
    [xax, yax] = pol2cart(angles(m), 1.05);
    plot([0, xax], [0, yax], '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
end

axis equal; axis off;
legend(algNames, 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 9);
title({'Algorithm Profile Radar (High Complexity)', '0 = best, 1 = worst'}, ...
    'FontSize', 12, 'FontWeight', 'bold');
exportPublicationFigure(fig_radar, fullfile(outDir, 'exp5_radar_comparison.png'));
fprintf('  → 图已保存: exp5_radar_comparison.png\n');

%% ==================== 保存全部结果 ====================

%% ====================================================================
%%  ★ 口径核查：final_unified_cost vs internal_search_cost 全局汇总
%%  本表是论文"方法公平性"的核心证据：
%%    所有算法的 final_J 均来自同一个 evaluatePath；
%%    RA-ALA 的 internal_J 仅是搜索时用到的导向值，不参与比较。
%% ====================================================================

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  口径核查：final_unified_cost vs internal_search_cost                       ║\n');
fprintf('║  final_J   = costModel.evaluatePath(finalPath)  ← 论文中报告的 J           ║\n');
fprintf('║  internal_J= evalRA_v2 内部适应度               ← 仅 RA-ALA 搜索导向用     ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  实验4：多随机种子统计 (20 seeds, high 城市)                               ║\n');
fprintf('╠══════════════╦══════════════════════════════╦══════════════════════════════╣\n');
fprintf('║ %-12s ║ %26s ║ %26s ║\n', '算法', 'final_J (均值±标准差)', 'internal_J (均值±标准差)');
fprintf('╠══════════════╬══════════════════════════════╬══════════════════════════════╣\n');
c_high = 3;   % high 城市索引
for a = 1:nAlg
    fu_mu  = mean(stat_cost(c_high, a, :));
    fu_sd  = std(stat_cost(c_high, a, :));
    if a == 1
        in_mu = mean(stat_internal_cost(c_high, a, :), 'omitnan');
        in_sd = std(stat_internal_cost(c_high, a, :), 0, 3);
        if isnan(in_sd), in_sd = 0; end
        in_str = sprintf('%10.2f ± %6.2f', in_mu, in_sd);
        delta_str = sprintf('差值 %+.1f', in_mu - fu_mu);
    else
        in_str    = sprintf('%26s', 'N/A (统一口径)');
        delta_str = '';
    end
    mk = ' '; if a==1, mk='★'; end
    fprintf('║%s%-11s ║ %10.2f ± %6.2f       ║ %-26s ║  %s\n', ...
        mk, algNames{a}, fu_mu, fu_sd, in_str, delta_str);
end
fprintf('╚══════════════╩══════════════════════════════╩══════════════════════════════╝\n');
fprintf('  结论: final_J 列是算法间公平比较的唯一依据;\n');
fprintf('        internal_J 高出 final_J 的部分 = 搜索阶段导向罚项总量\n');
fprintf('        (smoothPenalty + nfzPenalty + obsPenalty + headwindPenalty)\n\n');

%% ==================== 保存全部结果 ====================

save(fullfile(outDir, 'all_results.mat'), ...
    'abl_costs', 'abl_internal_costs', ...
    'abl_energy', 'abl_time', 'abl_risk', 'abl_penalty', ...
    'ablationNames', ...
    'wpValues', 'nWP_costs', 'nWP_times', ...
    'popValues', 'pop_costs', 'pop_times', ...
    'iterValues', 'iter_costs', 'iter_times', ...
    'departureTimes', 'tv_costs', 'tv_internal_costs', 'tv_energy', 'tv_risk', ...
    'stat_cost', 'stat_internal_cost', 'stat_energy', 'stat_time', 'stat_risk', ...
    'improv_all', 'pval_all', 'delta_all', ...
    'algNames', 'cityLevels', 'cfg_base');

fprintf('\n全部结果已保存至 %s/all_results.mat\n', outDir);
fprintf('实验完成!\n');

%% ====================================================================
%%  以下为必需的局部函数
%%  (从 run_RA_ALA.m 中复制, 支持消融开关)
%% ====================================================================

function [path, cost, details, stage_details] = runRA_ALA_exp(planner, costModel, env, start, goal, t_start, hasPayload, cfg)
    % This experiment entry mirrors the released RA-ALA configuration:
    % the physical wind field and headwind guidance remain active, whereas
    % the search update does not use a wind-biased walk operator.
% 与 run_RA_ALA.m 中的 runRA_ALA 功能完全一致, 支持消融开关
% ★ 新增第4输出 stage_details (三阶段代价分解: raw/smooth/repair)
% ★ 统一评估口径: Top-K 阶段所有候选均用 costModel.evaluatePath 重新计算

    start = start(:)'; goal = goal(:)';
    if length(start)<3, start=[start,60]; end
    if length(goal)<3, goal=[goal,60]; end

    nWP     = cfg.nWaypoints;
    popSize = cfg.popSize;
    maxIter = cfg.maxIter;
    minH = planner.minH;
    maxH = planner.maxH;
    MS   = env.MAP_SIZE;

    dirVec    = goal(1:2) - start(1:2);
    totalDist = norm(dirVec);
    dirUnit   = dirVec / max(totalDist, 1);
    perpUnit  = [-dirUnit(2), dirUnit(1)];

    dim    = nWP * 2;
    maxLat = 200;
    lb = repmat([-maxLat, minH], 1, nWP);
    ub = repmat([maxLat,  maxH], 1, nWP);

    param2path = @(x) paramToPath_exp(x, start, goal, nWP, dirUnit, perpUnit, totalDist, env, minH);
    evalFcn    = @(x) evalRA_v2_exp(x, param2path, t_start, hasPayload, costModel, env, cfg);

    % ---- 初始化种群 ----
    pop = zeros(popSize, dim);
    for j = 1:nWP
        pop(1,(j-1)*2+1) = 0;
        pop(1,(j-1)*2+2) = start(3);
    end
    try
        initPath = planner.greedyPath(start, goal, t_start);
        if size(initPath,1) >= nWP+2
            idx = round(linspace(2, size(initPath,1)-1, nWP));
            for j = 1:nWP
                pt = initPath(idx(j),1:2);
                baseXY = start(1:2) + (j/(nWP+1))*dirVec;
                offset = pt - baseXY;
                latOff = dot(offset, perpUnit);
                pop(2,(j-1)*2+1) = max(-maxLat, min(maxLat, latOff));
                pop(2,(j-1)*2+2) = max(minH, min(maxH, initPath(idx(j),3)));
            end
        end
    catch
    end
    for i = 3:popSize
        for j = 1:nWP
            pop(i,(j-1)*2+1) = (rand-0.5)*120;
            pop(i,(j-1)*2+2) = minH + rand*(maxH-minH);
        end
    end
    pop = max(lb, min(ub, pop));

    fitness = zeros(popSize, 1);
    for i = 1:popSize
        fitness(i) = evalFcn(pop(i,:));
    end
    [bestFit, bestIdx] = min(fitness);
    bestPos = pop(bestIdx,:);

    % ---- ALA 迭代 (支持风场消融) ----
    for iter = 1:maxIter
        theta = 2*atan(1 - iter/maxIter);
        sigma_decay = 1 - 0.6*iter/maxIter;
        for i = 1:popSize
            E = 2*log(1/rand)*theta;
            r1 = rand;
            if r1 < 0.3
                newPos = pop(i,:) + E*(bestPos - pop(i,:));
            elseif r1 < 0.55
                noise = randn(1,dim).*(ub-lb)*0.08*sigma_decay;
                newPos = pop(i,:) + E*noise;
            elseif r1 < 0.8
                l = rand*2-1;
                newPos = bestPos + E*exp(l)*cos(2*pi*l)*(pop(i,:)-bestPos)*sigma_decay;
            else
                beta_v=1.5;
                sig_l=(gamma(1+beta_v)*sin(pi*beta_v/2)/(gamma((1+beta_v)/2)*beta_v*2^((beta_v-1)/2)))^(1/beta_v);
                u=randn(1,dim)*sig_l; v=randn(1,dim);
                step=u./abs(v).^(1/beta_v).*(ub-lb)*0.025*sigma_decay;
                newPos=pop(i,:)+step;
            end
            newPos = max(lb, min(ub, newPos));
            newFit = evalFcn(newPos);
            if newFit < fitness(i)
                pop(i,:)=newPos; fitness(i)=newFit;
                if newFit<bestFit, bestFit=newFit; bestPos=newPos; end
            end
        end
    end

    % =========================================================
    % 捕获搜索阶段内部最优适应度.
    % evalRA_v2 内部 J = evaluatePath.J + smoothPenalty
    %                  + 5000×infeasible + nfzPenalty
    %                  + obsPenalty + headwindPenalty
    % 这是优化器"看到"的最优值, 不用于算法间比较.
    % =========================================================
    internal_search_cost = bestFit;
    % ★ 目标2: 所有候选均用 costModel.evaluatePath 重新计算最终 J
    % ★ 目标3: 记录 raw/smooth/repair 三阶段代价
    % 阶段二: Top-K → 三路径择优 (与 runRA_ALA 逻辑完全一致)
    K = min(5, popSize);
    [~, sortIdx] = sort(fitness,'ascend');
    topIdx = sortIdx(1:K);

    bestCost_f  = inf;
    bestPath_f  = [];
    bestDet_f   = struct();
    best_raw_det    = struct();
    best_smooth_det = struct();
    best_repair_det = struct();
    best_chosen_tag_f = 'none';
    bestFeas_f = false;
    bestPen_f  = inf;

    isBetterF = @(cJ,cPen,cFeas,curJ,curPen,curFeas) ...
        (cFeas && ~curFeas) || ...
        (cFeas &&  curFeas  && cJ   < curJ   - 1e-9) || ...
        (~cFeas && ~curFeas && cPen < curPen  - 1e-9) || ...
        (~cFeas && ~curFeas && abs(cPen-curPen)<1e-9 && cJ < curJ - 1e-9);

    for ci = 1:K
        rawPath  = param2path(pop(topIdx(ci),:));
        smoothed = smoothPathSpline_exp(rawPath, env, minH, maxH);
        if isfield(cfg,'ablate_repair') && cfg.ablate_repair
            repaired = smoothed;
        else
            repaired = postSmoothRepair_exp(smoothed, env, costModel, t_start, minH, maxH);
        end
        [raw_J,    raw_det]    = costModel.evaluatePath(rawPath,  t_start, hasPayload);
        [smooth_J, smooth_det] = costModel.evaluatePath(smoothed, t_start, hasPayload);
        [cCost,    cDet]       = costModel.evaluatePath(repaired, t_start, hasPayload);
        cDet.repair_penalty = cCost - smooth_J;

        cand_paths = {rawPath,  smoothed,  repaired};
        cand_dets  = {raw_det,  smooth_det, cDet};
        cand_Js    = [raw_J,    smooth_J,   cCost];
        cand_tags  = {'raw','smooth','repair'};

        for ci2 = 1:3
            cJ   = cand_Js(ci2);
            cD   = cand_dets{ci2};
            cFeas= cD.feasible;
            cPen = cD.penalty_total;
            if isBetterF(cJ,cPen,cFeas,bestCost_f,bestPen_f,bestFeas_f)
                bestCost_f  = cJ;
                bestPath_f  = cand_paths{ci2};
                bestDet_f   = cD;
                best_raw_det    = raw_det;
                best_smooth_det = smooth_det;
                best_repair_det = cDet;
                best_chosen_tag_f = cand_tags{ci2};
                bestFeas_f  = cFeas;
                bestPen_f   = cPen;
            end
        end
        raw_J=raw_J; smooth_J=smooth_J; %#ok<ASGSL>
    end
    if isempty(bestPath_f) || bestCost_f>=inf
        bestPath_f = param2path(bestPos);
        [bestCost_f, bestDet_f] = costModel.evaluatePath(bestPath_f, t_start, hasPayload);
        bestDet_f.repair_penalty = 0;
        best_raw_det=bestDet_f; best_smooth_det=bestDet_f; best_repair_det=bestDet_f;
        best_chosen_tag_f = 'raw(fallback)';
    end
    % 定向二次救援 v10 (与 run_RA_ALA 完全一致 — 多位置插点)
    if ~bestFeas_f
        MS_rce=env.MAP_SIZE; cl_rce=costModel.H_clearance;
        minH_rce=minH; maxH_rce=maxH;
        SUB_SP_E=12; MIN_SUB_E=3; MAX_INS_E=6;

        try
            rp_Ae=bestPath_f; ins_tot_e=0; passA_mod_e=false;
            if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'movingObs')
                DIRS_E=zeros(8,2); for di_=1:8,ag=(di_-1)*pi/4;DIRS_E(di_,:)=[cos(ag),sin(ag)];end
                DETOUR_D_E=[30,60,100,150,200,300]; obsL_e=env.dynObstacles.movingObs;

                while ins_tot_e<MAX_INS_E
                    [j_ce,det_ce]=costModel.evaluatePath(rp_Ae,t_start,hasPayload);
                    tA_rce=det_ce.t_arrivals; nRP_Ae=size(rp_Ae,1);
                    if numel(tA_rce)~=nRP_Ae,tA_rce=computeApproxTArr_exp(rp_Ae,t_start);end
                    if ~isfield(det_ce,'dyn_col_segs')||all(det_ce.dyn_col_segs==0),break;end

                    [~,wsi_e]=max(det_ce.dyn_col_segs); k_ce=wsi_e;
                    if k_ce<1||k_ce>=size(rp_Ae,1),break;end
                    p1ce=rp_Ae(k_ce,:); p2ce=rp_Ae(k_ce+1,:);
                    t1ce=tA_rce(min(k_ce,numel(tA_rce))); t2ce=tA_rce(min(k_ce+1,numel(tA_rce)));
                    d3ce=norm(p2ce-p1ce); if d3ce<0.01,break;end
                    nSub_ce=max(MIN_SUB_E,ceil(d3ce/SUB_SP_E));
                    best_r_ce=inf; op_cole=zeros(1,3); obs_r_cole=1; frac_cole=0.5;
                    for s_ce=1:nSub_ce
                        fr_ce=(s_ce-0.5)/nSub_ce; ptce=p1ce+fr_ce*(p2ce-p1ce); tce=t1ce+fr_ce*(t2ce-t1ce);
                        for oi_ce=1:length(obsL_e)
                            ob_ce=obsL_e(oi_ce); op_ce=env.dynObstacles.getPosition(oi_ce,tce);
                            d_ce=norm(ptce-op_ce);
                            if d_ce<ob_ce.radius&&d_ce/ob_ce.radius<best_r_ce
                                best_r_ce=d_ce/ob_ce.radius; op_cole=op_ce; obs_r_cole=ob_ce.radius; frac_cole=fr_ce;
                            end
                        end
                    end
                    if best_r_ce>=1,break;end

                    best_j_rnd_e=j_ce; best_pen_rnd_e=det_ce.penalty_total; best_feas_rnd_e=det_ce.feasible; best_rp_rnd_e=rp_Ae; found_e=false;
                    ins_pos_e=[max(1,k_ce-1), k_ce];
                    if k_ce+1<size(rp_Ae,1), ins_pos_e=[ins_pos_e,k_ce+1]; end

                    for ip_e=1:length(ins_pos_e)
                        if found_e&&best_feas_rnd_e,break;end
                        ins_ae=ins_pos_e(ip_e);
                        for di_=1:8
                            if found_e&&best_feas_rnd_e,break;end
                            evde=DIRS_E(di_,:);
                            for dist_ie=1:length(DETOUR_D_E)
                                D_de=DETOUR_D_E(dist_ie);
                                for hz_e=1:2
                                    ptd=[op_cole(1)+evde(1)*(obs_r_cole+D_de),op_cole(2)+evde(2)*(obs_r_cole+D_de),0];
                                    ptd(1)=max(1,min(MS_rce,ptd(1))); ptd(2)=max(1,min(MS_rce,ptd(2)));
                                    if hz_e==1
                                        ptd(3)=rp_Ae(min(ins_ae,size(rp_Ae,1)),3);
                                    else
                                        above_ze=op_cole(3)+obs_r_cole+15;
                                        if above_ze>maxH_rce-5,continue;end
                                        ptd(3)=above_ze;
                                    end
                                    if ~isempty(env.heightMap)
                                        rx=max(1,min(MS_rce,round(ptd(1)))); ry=max(1,min(MS_rce,round(ptd(2))));
                                        ptd(3)=max(ptd(3),max(minH_rce,env.heightMap(rx,ry)+cl_rce));
                                    end
                                    ptd(3)=max(ptd(3),minH_rce); ptd(3)=min(ptd(3),maxH_rce);
                                    if norm(ptd(1:2)-op_cole(1:2))<(obs_r_cole+D_de)*0.5,continue;end
                                    if ins_ae>=size(rp_Ae,1),continue;end
                                    rp_try_e=[rp_Ae(1:ins_ae,:);ptd;rp_Ae(ins_ae+1:end,:)];
                                    [jTe,detTe]=costModel.evaluatePath(rp_try_e,t_start,hasPayload);
                                    penTe=detTe.penalty_total;
                                    if isBetterF(jTe,penTe,detTe.feasible,best_j_rnd_e,best_pen_rnd_e,best_feas_rnd_e)
                                        best_j_rnd_e=jTe; best_pen_rnd_e=penTe; best_feas_rnd_e=detTe.feasible;
                                        best_rp_rnd_e=rp_try_e; found_e=true;
                                        if detTe.feasible,break;end
                                    end
                                end
                                if found_e&&best_feas_rnd_e,break;end
                            end
                            if found_e&&best_feas_rnd_e,break;end
                        end
                        if found_e&&best_feas_rnd_e,break;end
                    end

                    if found_e&&isBetterF(best_j_rnd_e,best_pen_rnd_e,best_feas_rnd_e,j_ce,det_ce.penalty_total,det_ce.feasible)
                        rp_Ae=best_rp_rnd_e; ins_tot_e=ins_tot_e+1; passA_mod_e=true;
                    else,break;end
                    if best_feas_rnd_e,break;end
                end

                if passA_mod_e
                    [jAe,detAe]=costModel.evaluatePath(rp_Ae,t_start,hasPayload);
                    penAe=detAe.penalty_total; feasAe=detAe.feasible;
                    if isBetterF(jAe,penAe,feasAe,bestCost_f,bestPen_f,bestFeas_f)
                        bestPath_f=rp_Ae; bestCost_f=jAe;
                        [~,bestDet_f]=costModel.evaluatePath(rp_Ae,t_start,hasPayload); bestDet_f.repair_penalty=0;
                        bestFeas_f=feasAe; bestPen_f=penAe; best_chosen_tag_f=[best_chosen_tag_f,'+rescA'];
                    end
                end
            end
        catch;end

        try
            rp_Be=bestPath_f; nRP_Be=size(rp_Be,1);
            [~,det_B0e]=costModel.evaluatePath(rp_Be,t_start,hasPayload);
            tA_rcBe=det_B0e.t_arrivals;
            if numel(tA_rcBe)~=nRP_Be,tA_rcBe=computeApproxTArr_exp(rp_Be,t_start);end
            VIOL_SC_E=2.5; EXTRA_M_E=8; VT_E=0.02; NFXE=20;
            seg_vBe=zeros(nRP_Be-1,1); seg_lBe=zeros(nRP_Be-1,1); passB_mod_e=false;
            for k_Be=1:nRP_Be-1
                p1Be=rp_Be(k_Be,:); p2Be=rp_Be(k_Be+1,:);
                d3Be=norm(p2Be-p1Be); if d3Be<0.01,continue;end
                nSub_Be=max(MIN_SUB_E,ceil(d3Be/12));
                for s_Be=1:nSub_Be
                    fr_Be=(s_Be-0.5)/nSub_Be; ptBe=p1Be+fr_Be*(p2Be-p1Be);
                    if ~isempty(env.heightMap)
                        rx=max(1,min(MS_rce,round(ptBe(1)))); ry=max(1,min(MS_rce,round(ptBe(2))));
                        gHBe=env.heightMap(rx,ry); mfBe=max(minH_rce,gHBe+cl_rce);
                        if ptBe(3)<mfBe,vBe=mfBe-ptBe(3);seg_vBe(k_Be)=seg_vBe(k_Be)+vBe;seg_lBe(k_Be)=max(seg_lBe(k_Be),vBe*VIOL_SC_E+EXTRA_M_E);end
                        if ptBe(3)<gHBe+3,vBe2=gHBe+3-ptBe(3);seg_vBe(k_Be)=seg_vBe(k_Be)+vBe2;seg_lBe(k_Be)=max(seg_lBe(k_Be),vBe2*VIOL_SC_E+gHBe+cl_rce+EXTRA_M_E);end
                    end
                end
            end
            [~,sortBe]=sort(seg_vBe,'descend');
            for ki_Be=1:min(nRP_Be-1,NFXE)
                k_Be=sortBe(ki_Be); if seg_vBe(k_Be)<VT_E,break;end
                lBe=seg_lBe(k_Be);
                for fe_Be=[k_Be,k_Be+1]
                    if fe_Be==1||fe_Be==nRP_Be,continue;end
                    if ~isempty(env.heightMap)
                        rx=max(1,min(MS_rce,round(rp_Be(fe_Be,1)))); ry=max(1,min(MS_rce,round(rp_Be(fe_Be,2)))); gHfe=env.heightMap(rx,ry);
                    else,gHfe=0;end
                    tZ=max(rp_Be(fe_Be,3)+lBe,max(minH_rce,gHfe+cl_rce+EXTRA_M_E));
                    rp_Be(fe_Be,3)=min(maxH_rce,tZ); passB_mod_e=true;
                end
            end
            if passB_mod_e
                [jBe,detBe]=costModel.evaluatePath(rp_Be,t_start,hasPayload);
                penBe=detBe.penalty_total; feasBe=detBe.feasible;
                if isBetterF(jBe,penBe,feasBe,bestCost_f,bestPen_f,bestFeas_f)
                    bestPath_f=rp_Be; bestCost_f=jBe;
                    [~,bestDet_f]=costModel.evaluatePath(rp_Be,t_start,hasPayload); bestDet_f.repair_penalty=0;
                    bestFeas_f=feasBe; bestPen_f=penBe; best_chosen_tag_f=[best_chosen_tag_f,'+rescB'];
                end
            end
        catch;end
    end  % ~bestFeas_f

        path=bestPath_f; cost=bestCost_f; details=bestDet_f;
    details.internal_search_cost = internal_search_cost;
    details.chosen_path_type     = best_chosen_tag_f;
    stage_details.raw    = best_raw_det;
    stage_details.smooth = best_smooth_det;
    stage_details.repair = best_repair_det;
    stage_details.internal_search_cost = internal_search_cost;
end

%% ---- paramToPath ----
function path = paramToPath_exp(x, start, goal, nWP, dirUnit, perpUnit, ~, env, minH)
    path = zeros(nWP+2, 3);
    path(1,:) = start; path(end,:) = goal;
    for j = 1:nWP
        frac = j/(nWP+1);
        latOff = x((j-1)*2+1); alt = x((j-1)*2+2);
        baseXY = start(1:2) + frac*(goal(1:2)-start(1:2));
        ptXY = baseXY + latOff*perpUnit;
        ptXY(1)=max(1,min(env.MAP_SIZE,ptXY(1)));
        ptXY(2)=max(1,min(env.MAP_SIZE,ptXY(2)));
        ex=max(1,min(env.MAP_SIZE,round(ptXY(1))));
        ey=max(1,min(env.MAP_SIZE,round(ptXY(2))));
        alt=max(alt, env.heightMap(ex,ey)+5); alt=max(alt,minH);
        path(j+1,:) = [ptXY, alt];
    end
end

%% ---- evalRA_v2 (支持消融) ----
function J = evalRA_v2_exp(x, param2path, t_start, hasPayload, costModel, env, cfg)
% evalRA_v2_exp — 与 evalRA_v2 完全一致的校准版 (支持消融开关)
% 导向惩罚已缩减至与 final_unified_J 同量级:
%   smooth ≤15, infeasible=150, NFZ ≤25, obs ≤20, headwind ≤8
    path = param2path(x);
    if ~(isfield(cfg,'ablate_repair') && cfg.ablate_repair)
        path = repairRawPath_exp(path, env, costModel, t_start, costModel.H_min, costModel.H_max);
    end
    nPts = size(path, 1);
    [J, det] = costModel.evaluatePath(path, t_start, hasPayload);
    J = J + (cfg.riskWeight-10)*det.R_dynamic;

    % (1) smoothPenalty: coeff 8→0.3, cap 15
    if ~(isfield(cfg,'ablate_smoothPenalty') && cfg.ablate_smoothPenalty)
        sp=0; SMOOTH_COEFF=0.3; SMOOTH_CAP=15.0;
        for k=2:nPts-1
            v1=path(k,:)-path(k-1,:); v2=path(k+1,:)-path(k,:);
            n1=norm(v1); n2=norm(v2);
            if n1>1&&n2>1
                ca=max(-1,min(1,dot(v1,v2)/(n1*n2)));
                sp=sp+acos(ca)^2*SMOOTH_COEFF;
            end
        end
        J=J+min(sp,SMOOTH_CAP);
    end

    % (2) 不可行惩罚: 5000→150
    if ~det.feasible, J=J+150; end

    % (2b) 静态高度违规 proxy (与 run_RA_ALA evalRA_v2 完全一致)
    PROX_NSUB_E=4; PROX_CAP_E=50.0; PROX_HCOEF_E=2.0;
    heightProxy_e=0;
    if ~isempty(env.heightMap)
        MS_pe=env.MAP_SIZE; cl_pe=costModel.H_clearance;
        for k=1:nPts-1
            p1e=path(k,:); p2e=path(k+1,:);
            for s=1:PROX_NSUB_E
                frac=s/(PROX_NSUB_E+1); pt_pe=p1e+frac*(p2e-p1e);
                rx=max(1,min(MS_pe,round(pt_pe(1)))); ry=max(1,min(MS_pe,round(pt_pe(2))));
                gH_pe=env.heightMap(rx,ry);
                minF_pe=max(costModel.H_min,gH_pe+cl_pe);
                if pt_pe(3)<minF_pe
                    heightProxy_e=heightProxy_e+PROX_HCOEF_E*(minF_pe-pt_pe(3))/PROX_NSUB_E;
                end
                if pt_pe(3)>costModel.H_max
                    heightProxy_e=heightProxy_e+PROX_HCOEF_E*(pt_pe(3)-costModel.H_max)/PROX_NSUB_E;
                end
            end
        end
    end
    J=J+min(heightProxy_e,PROX_CAP_E);

    % (3) NFZ + obs —— v12: 已移除搜索期 Explicit 惩罚
    %     NFZ 现由 UnifiedCostModel.evaluatePath 作硬约束计入 J (pen_nfz);
    %     动态障碍碰撞在 pen_dyn(硬)、邻近在 R_dynamic(软), obsP 冗余, 同删。
    %     与 evalRA_v2.m 保持完全一致。

    % (4) 风场前瞻: 保持不变, cap 8
    if cfg.windLookahead>0&&nPts>2
        if isfield(det,'t_arrivals')&&numel(det.t_arrivals)==nPts
            tA_w=det.t_arrivals;
        else
            tA_w=t_start+det.T_total*(0:nPts-1)'/max(nPts-1,1);
        end
        T_end_w=tA_w(end); hp=0; HEADWIND_CAP=8.0;
        for la=1:cfg.windLookahead
            ft=t_start+(T_end_w-t_start)*la/(cfg.windLookahead+1);
            [~,mi]=min(abs(tA_w-ft)); mi=max(2,min(nPts-1,mi));
            pt=path(mi,:);
            w=env.windField.getWind(pt(1),pt(2),pt(3),ft);
            d3=path(mi+1,:)-path(mi-1,:); d3=d3/max(norm(d3),0.01);
            hw=-dot(w,d3); if hw>2, hp=hp+hw*0.5; end
        end
        J=J+min(hp,HEADWIND_CAP);
    end
end


%% ---- repairRawPath ----
function repaired = repairRawPath_exp(path, env, costModel, t_start, minH, maxH)
% 优化循环内轻量修复 + 3D 动态障碍规避
% 使用 cumDist/12 粗估时间 (循环内不调用 evaluatePath, 保证速度)
    MS=env.MAP_SIZE; nPts=size(path,1); repaired=path;
    cl=5; if isprop(costModel,'H_clearance'), cl=costModel.H_clearance; end
    tA = computeApproxTArr_exp(repaired, t_start);
    OBS_SAFE_MULT = 3.0;
    for k=2:nPts-1
        pt=repaired(k,:); t_k=tA(k);
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(pt(1)))); ey=max(1,min(MS,round(pt(2))));
            gH=env.heightMap(ex,ey); mA=max(minH,gH+cl);
            if pt(3)<mA
                if mA+3<=maxH, repaired(k,3)=mA+3;
                else
                    sR=30; bH=gH; bXY=pt(1:2);
                    for ai=1:8
                        ang=(ai-1)*pi/4; cXY=pt(1:2)+sR*[cos(ang),sin(ang)];
                        cXY(1)=max(1,min(MS,cXY(1))); cXY(2)=max(1,min(MS,cXY(2)));
                        cx=max(1,min(MS,round(cXY(1)))); cy=max(1,min(MS,round(cXY(2))));
                        cH=env.heightMap(cx,cy); if cH<bH, bH=cH; bXY=cXY; end
                    end
                    repaired(k,1:2)=bXY; repaired(k,3)=max(minH,bH+cl+3);
                end
            end
        end
        if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'tempNFZ')
            for ni=1:length(env.dynObstacles.tempNFZ)
                nfz=env.dynObstacles.tempNFZ(ni);
                if ~nfz.active||t_k<nfz.t_start||t_k>nfz.t_end,continue;end
                pn=repaired(k,:); dh=norm(pn(1:2)-nfz.center);
                if dh<nfz.radius*1.1&&pn(3)>=nfz.height(1)&&pn(3)<=nfz.height(2)
                    if nfz.height(2)+10<=maxH, repaired(k,3)=nfz.height(2)+10;
                    elseif nfz.height(1)-10>=minH, repaired(k,3)=nfz.height(1)-10;
                    else
                        do=pn(1:2)-nfz.center; dn=norm(do);
                        if dn<0.1,do=[1,0];dn=1;end
                        repaired(k,1:2)=nfz.center+(do/dn)*nfz.radius*1.25;
                    end
                end
            end
        end
        if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'movingObs')
            for oi=1:length(env.dynObstacles.movingObs)
                obs=env.dynObstacles.movingObs(oi);
                op=env.dynObstacles.getPosition(oi,t_k);
                sR=obs.radius*OBS_SAFE_MULT;
                pn=repaired(k,:); d=norm(pn-op);
                if d<sR
                    if d>0.01
                        ev=(pn-op)/d; newPt=op+ev*(sR+3);
                        if newPt(3)>maxH
                            ev2=[ev(1),ev(2),0]; n2=norm(ev2);
                            if n2>0.01, ev2=ev2/n2; newPt=op+ev2*(sR+3); end
                            newPt(3)=max(minH,min(maxH,pn(3)));
                        end
                        repaired(k,:)=newPt;
                    else
                        ang=rand*2*pi;
                        repaired(k,1)=op(1)+sR*cos(ang)+3;
                        repaired(k,2)=op(2)+sR*sin(ang)+3;
                        repaired(k,3)=min(maxH,op(3)+obs.radius*2);
                    end
                end
            end
        end
        repaired(k,1)=max(1,min(MS,repaired(k,1)));
        repaired(k,2)=max(1,min(MS,repaired(k,2)));
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
            gH=env.heightMap(ex,ey);
            repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
        else, repaired(k,3)=max(repaired(k,3),minH); end
        repaired(k,3)=min(repaired(k,3),maxH);
    end
    repaired(1,:)=path(1,:); repaired(end,:)=path(end,:);
end

%% ============== 平滑后复核 + 局部修复 (含段级 validate + 插点) ==============
function repaired = postSmoothRepair_exp(smoothed, env, costModel, t_start, minH, maxH)
% 与 run_RA_ALA.m postSmoothRepair 完全一致 (支持消融)
    MAX_PASS  = 3; SEG_NSUB  = 8; SEG_PASSES= 5; OBS_CLEAR = 3.2;
    MS=env.MAP_SIZE; repaired=smoothed;
    cl=5; if isprop(costModel,'H_clearance'), cl=costModel.H_clearance; end
    % Phase-1: 点级扫描
    for pass=1:MAX_PASS
        anyFix=false; nPts=size(repaired,1);
        try
            [~, pass_det_] = costModel.evaluatePath(repaired, t_start, true);
            tA = pass_det_.t_arrivals;
            if numel(tA) ~= nPts, tA = computeApproxTArr_exp(repaired, t_start); end
        catch
            tA = computeApproxTArr_exp(repaired, t_start);
        end
        for k=2:nPts-1
            pt=repaired(k,:); t_k=tA(k);
            if ~isempty(env.heightMap)
                ex=max(1,min(MS,round(pt(1)))); ey=max(1,min(MS,round(pt(2))));
                gH=env.heightMap(ex,ey); mA=max(minH,gH+cl);
                if pt(3)<mA
                    if mA+2<=maxH, repaired(k,3)=mA+2; anyFix=true;
                    else
                        sR=25; bH=gH; bXY=pt(1:2);
                        for ai=0:7
                            ang=ai*pi/4; cXY=pt(1:2)+sR*[cos(ang),sin(ang)];
                            cXY(1)=max(1,min(MS,cXY(1))); cXY(2)=max(1,min(MS,cXY(2)));
                            cx=max(1,min(MS,round(cXY(1)))); cy=max(1,min(MS,round(cXY(2))));
                            cH=env.heightMap(cx,cy); if cH<bH,bH=cH;bXY=cXY;end
                        end
                        repaired(k,1:2)=bXY; repaired(k,3)=max(minH,bH+cl+2);
                        repaired(k,3)=min(repaired(k,3),maxH); anyFix=true;
                    end
                end
            end
            if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'tempNFZ')
                for ni=1:length(env.dynObstacles.tempNFZ)
                    nfz=env.dynObstacles.tempNFZ(ni);
                    if ~nfz.active||t_k<nfz.t_start||t_k>nfz.t_end,continue;end
                    dh=norm(pt(1:2)-nfz.center);
                    if dh<nfz.radius*1.1&&pt(3)>=nfz.height(1)&&pt(3)<=nfz.height(2)
                        if nfz.height(2)+10<=maxH, repaired(k,3)=nfz.height(2)+10; anyFix=true;
                        else
                            do=pt(1:2)-nfz.center; dn=norm(do);
                            if dn<0.1,do=[1,0];dn=1;end
                            repaired(k,1:2)=nfz.center+(do/dn)*nfz.radius*1.2;
                            repaired(k,1)=max(1,min(MS,repaired(k,1)));
                            repaired(k,2)=max(1,min(MS,repaired(k,2))); anyFix=true;
                        end
                    end
                end
            end
            if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'movingObs')
                for oi=1:length(env.dynObstacles.movingObs)
                    obs=env.dynObstacles.movingObs(oi);
                    op=env.dynObstacles.getPosition(oi,t_k);
                    sR=obs.radius*OBS_CLEAR; pn=repaired(k,:); d=norm(pn-op);
                    if d<sR
                        if d>0.01
                            ev=(pn-op)/d; newPt=op+ev*(sR+5);
                            if newPt(3)>maxH
                                ev2=[ev(1),ev(2),0]; n2=norm(ev2);
                                if n2>0.01, ev2=ev2/n2; newPt=op+ev2*(sR+5); newPt(3)=max(minH,min(maxH,pn(3))); end
                            end
                            repaired(k,:)=newPt;
                        else
                            ang=rand*2*pi;
                            repaired(k,1)=op(1)+sR*cos(ang)+5;
                            repaired(k,2)=op(2)+sR*sin(ang)+5;
                            repaired(k,3)=min(maxH,op(3)+obs.radius*2);
                        end
                        anyFix=true;
                    end
                end
            end
            repaired(k,1)=max(1,min(MS,repaired(k,1))); repaired(k,2)=max(1,min(MS,repaired(k,2)));
            if ~isempty(env.heightMap)
                ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
                gH=env.heightMap(ex,ey); repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
            else, repaired(k,3)=max(repaired(k,3),minH); end
            repaired(k,3)=min(repaired(k,3),maxH);
        end
        if ~anyFix,break;end
    end
    % Phase-2: 轻量平滑
    nPts=size(repaired,1);
    for k=2:nPts-1
        repaired(k,:)=0.2*repaired(k-1,:)+0.6*repaired(k,:)+0.2*repaired(k+1,:);
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
            gH=env.heightMap(ex,ey); repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
        else, repaired(k,3)=max(repaired(k,3),minH); end
        repaired(k,3)=min(repaired(k,3),maxH);
        repaired(k,1)=max(1,min(MS,repaired(k,1))); repaired(k,2)=max(1,min(MS,repaired(k,2)));
    end
    repaired(1,:)=smoothed(1,:); repaired(end,:)=smoothed(end,:);
    % Phase-3: 强制高度合规
    nPts=size(repaired,1);
    for k=2:nPts-1
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
            gH=env.heightMap(ex,ey); repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
        else, repaired(k,3)=max(repaired(k,3),minH); end
        repaired(k,3)=min(repaired(k,3),maxH);
    end
    % Phase-4: 段级 validate + 插点
    nPts_init_e=size(repaired,1);
    MAX_TOTAL_INS_E=nPts_init_e;
    for sg_pass=1:SEG_PASSES
        nPts=size(repaired,1);
        try
            [~, sg_det_]=costModel.evaluatePath(repaired,t_start,true);
            sg_tArr=sg_det_.t_arrivals;
            if numel(sg_tArr)~=nPts, sg_tArr=computeApproxTArr_exp(repaired,t_start); end
        catch
            sg_tArr=computeApproxTArr_exp(repaired,t_start);
        end
        inserted=false; total_ins_e=0;
        for k=1:size(repaired,1)-1
            if total_ins_e>=MAX_TOTAL_INS_E, break; end
            nPts=size(repaired,1);
            p1=repaired(k,:); t1=sg_tArr(min(k,numel(sg_tArr)));
            p2=repaired(k+1,:); t2=sg_tArr(min(k+1,numel(sg_tArr)));
            hit_type_e=''; hit_s_e=0; hit_obs_e=0;
            hit_pt_e=[]; hit_op_e=[]; hit_viol_z_e=0;
            for s=1:SEG_NSUB_E
                frac=s/(SEG_NSUB_E+1); pt_s=p1+frac*(p2-p1); t_s=t1+frac*(t2-t1);
                if ~isempty(env.heightMap)
                    rx=max(1,min(MS,round(pt_s(1)))); ry=max(1,min(MS,round(pt_s(2))));
                    gH_s=env.heightMap(rx,ry); minFloor_s=max(minH,gH_s+cl);
                    if pt_s(3)<gH_s+3||pt_s(3)<minFloor_s
                        hit_type_e='static'; hit_s_e=s; hit_pt_e=pt_s;
                        hit_viol_z_e=max(minFloor_s,gH_s+cl+3); break;
                    end
                end
                if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'movingObs')
                    for oi=1:length(env.dynObstacles.movingObs)
                        obs=env.dynObstacles.movingObs(oi);
                        op=env.dynObstacles.getPosition(oi,t_s);
                        if norm(pt_s-op)<obs.radius
                            hit_type_e='dyn'; hit_s_e=s; hit_obs_e=oi;
                            hit_pt_e=pt_s; hit_op_e=op; break;
                        end
                    end
                    if strcmp(hit_type_e,'dyn'), break; end
                end
            end
            if isempty(hit_type_e), continue; end
            if strcmp(hit_type_e,'static')
                for fix_end=[k,k+1]
                    if fix_end<1||fix_end>nPts||fix_end==1||fix_end==nPts,continue;end
                    pf=repaired(fix_end,:);
                    pf_rx=max(1,min(MS,round(pf(1)))); pf_ry=max(1,min(MS,round(pf(2))));
                    gH_fix=env.heightMap(pf_rx,pf_ry);
                    newZ=min(maxH,max(pf(3),max(hit_viol_z_e,gH_fix+cl+3)));
                    if newZ>pf(3)+0.1, repaired(fix_end,3)=newZ; inserted=true; end
                end
            else
                obs_e=env.dynObstacles.movingObs(hit_obs_e);
                safe_rad_e=obs_e.radius*OBS_CLEAR;
                ev3=hit_pt_e-hit_op_e; ev3_n=norm(ev3);
                if ev3_n<0.01,ev3=[1,0,0];else,ev3=ev3/ev3_n;end
                ev_h=[ev3(1),ev3(2),0]; evh_n=norm(ev_h);
                if evh_n>0.01,ev_h=ev_h/evh_n;else,ar=rand*2*pi;ev_h=[cos(ar),sin(ar),0];end
                for fix_end=[k+1,k]
                    if fix_end<1||fix_end>nPts||fix_end==1||fix_end==nPts,continue;end
                    pf=repaired(fix_end,:);
                    newPf=hit_op_e+ev_h*(safe_rad_e+5); newPf(3)=pf(3);
                    op_f=env.dynObstacles.getPosition(hit_obs_e,sg_tArr(min(fix_end,numel(sg_tArr))));
                    if norm(newPf-op_f)<obs_e.radius,newPf(3)=min(maxH,op_f(3)+obs_e.radius*OBS_CLEAR+5);end
                    newPf(1)=max(1,min(MS,newPf(1))); newPf(2)=max(1,min(MS,newPf(2)));
                    if ~isempty(env.heightMap)
                        rx=max(1,min(MS,round(newPf(1)))); ry=max(1,min(MS,round(newPf(2))));
                        newPf(3)=max(newPf(3),max(minH,env.heightMap(rx,ry)+cl));
                    end
                    newPf(3)=max(newPf(3),minH); newPf(3)=min(newPf(3),maxH);
                    repaired(fix_end,:)=newPf;
                end
                ins_e=hit_op_e+ev_h*(safe_rad_e+8); ins_e(3)=hit_pt_e(3);
                if norm(ins_e-hit_op_e)<safe_rad_e,ins_e(3)=min(maxH,hit_op_e(3)+safe_rad_e+8);end
                ins_e(1)=max(1,min(MS,ins_e(1))); ins_e(2)=max(1,min(MS,ins_e(2)));
                if ~isempty(env.heightMap)
                    rx=max(1,min(MS,round(ins_e(1)))); ry=max(1,min(MS,round(ins_e(2))));
                    ins_e(3)=max(ins_e(3),max(minH,env.heightMap(rx,ry)+cl));
                end
                ins_e(3)=max(ins_e(3),minH); ins_e(3)=min(ins_e(3),maxH);
                repaired=[repaired(1:k,:); ins_e; repaired(k+1:end,:)];
                frac_ins=hit_s_e/(SEG_NSUB_E+1); t_ins=t1+frac_ins*(t2-t1);
                sg_tArr=[sg_tArr(1:k); t_ins; sg_tArr(k+1:end)];
                inserted=true; total_ins_e=total_ins_e+1;
            end
        end
        if ~inserted,break;end
    end
    % Phase-5: 插点后高度合规
    nPts=size(repaired,1);
    for k=2:nPts-1
        repaired(k,1)=max(1,min(MS,repaired(k,1))); repaired(k,2)=max(1,min(MS,repaired(k,2)));
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
            gH=env.heightMap(ex,ey); repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
        else, repaired(k,3)=max(repaired(k,3),minH); end
        repaired(k,3)=min(repaired(k,3),maxH);
    end
    repaired(1,:)=smoothed(1,:); repaired(end,:)=smoothed(end,:);
end


%% ============== 辅助: 粗估到达时刻 (仅 evaluatePath 失败时回退用) ==============
function t_arr = computeApproxTArr_exp(path, t_start)
% 基于 cumDist/12m/s 的回退粗估; 仅在 evaluatePath 抛出异常时使用
    nPts = size(path, 1);
    cumDist = zeros(nPts, 1);
    for k = 2:nPts
        cumDist(k) = cumDist(k-1) + norm(path(k,:) - path(k-1,:));
    end
    t_arr = t_start + cumDist / 12;
end

%% ====================================================================
%%  统计分析辅助函数
%% ====================================================================

function p = signrankTest_local(x, y)
% Wilcoxon signed-rank 配对检验 (双边, 无需 toolbox)
    diffs = x(:) - y(:);
    diffs(abs(diffs) < eps) = [];
    n = length(diffs);
    if n < 3, p = 1; return; end

    [sortedAbs, sortIdx] = sort(abs(diffs));
    ranks = zeros(n, 1);
    i = 1;
    while i <= n
        j = i;
        while j < n && abs(sortedAbs(j+1)-sortedAbs(j)) < eps*max(1,sortedAbs(j))
            j = j + 1;
        end
        avgR = mean(i:j);
        for kk = i:j, ranks(kk) = avgR; end
        i = j + 1;
    end
    origRanks = zeros(n, 1);
    origRanks(sortIdx) = ranks;

    W_plus  = sum(origRanks(diffs > 0));
    W_minus = sum(origRanks(diffs < 0));
    W = min(W_plus, W_minus);

    if n >= 10
        mu_W    = n*(n+1)/4;
        sigma_W = sqrt(n*(n+1)*(2*n+1)/24);
        z = (W - mu_W) / sigma_W;
        p = 2 * approxNormCDF_local(-abs(z));
    else
        total = 2^n; count = 0;
        for mask = 0:total-1
            Tp = 0; Tn = 0;
            for bit = 1:n
                if bitand(mask, 2^(bit-1)), Tp=Tp+bit; else, Tn=Tn+bit; end
            end
            if min(Tp,Tn) <= W, count=count+1; end
        end
        p = count / total;
    end
    p = max(p, 1e-300);
end

function p = approxNormCDF_local(z)
% 标准正态 CDF 多项式近似
    t = 1 ./ (1 + 0.2316419 * abs(z));
    d = 0.3989422804014327;
    pu = d*exp(-z.^2/2) .* (t.*(0.319381530 + t.*(-0.356563782 + ...
         t.*(1.781477937 + t.*(-1.821255978 + t.*1.330274429)))));
    p = zeros(size(z));
    p(z >= 0) = 1 - pu(z >= 0);
    p(z < 0)  = pu(z < 0);
end

function d = cliffsDelta_local(x, y)
% Cliff's delta 非参数效应量
    x = x(isfinite(x)); y = y(isfinite(y));
    nx = numel(x); ny = numel(y);
    if nx==0 || ny==0, d=0; return; end
    more = 0; less = 0;
    for i = 1:nx
        for j = 1:ny
            if x(i) > y(j),     more = more + 1;
            elseif x(i) < y(j), less = less + 1;
            end
        end
    end
    d = (more - less) / (nx * ny);
end

function s = formatPval_local(p)
% p值科学计数法格式化
    if isnan(p),       s = 'NaN';
    elseif p < 1e-4,   s = '< 1e-4';
    elseif p < 0.001,  s = sprintf('%.2e', p);
    elseif p < 0.01,   s = sprintf('%.4f', p);
    elseif p < 1,      s = sprintf('%.3f', p);
    else,              s = '1.000';
    end
end

function s = interpretDelta_local(d)
% Cliff's delta 效应量解读
    ad = abs(d);
    if ad < 0.147,     s = 'negligible';
    elseif ad < 0.33,  s = 'small';
    elseif ad < 0.474, s = 'medium';
    else,              s = 'large';
    end
end

function cmap = makeRBColormap()
% 红白蓝渐变色图
    n = 128;
    r = [linspace(0.15, 1, n), linspace(1, 0.85, n)];
    g = [linspace(0.15, 1, n), linspace(1, 0.15, n)];
    b = [linspace(0.85, 1, n), linspace(1, 0.15, n)];
    cmap = [r' g' b'];
end
