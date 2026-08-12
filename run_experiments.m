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

% 结果: [nCity x nAlg x nSeeds] × 4 指�ן7��$z{-���jם2);
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
