%% ====================================================================
%%  metaheuristic_benchmark.m
%%  收敛性对比 (Fig13 式) + 基准表 (Table9 式)
%% ====================================================================
%
%  五个元启发式共用同一"统一代价"适应度 evalFcn、同一决策空间 [lb,ub]、
%  同一 popSize/maxIter、每个 run 同一初始随机种子, 做公平对比:
%     RA-ALA   : 本文方法 (ALA + DE + 停滞重启)      —— runRA_ALA 默认输出
%     ALA      : 基础 ALA  (原始 Lévy, 无 DE/重启)     —— 显示求解器增强的增益
%     PSO / GA / WOA : 本文件内新实现 (优化同一 evalFcn)
%
%  产出:
%     1) fig_convergence_comparison.png  (Fig13 式: best-so-far 适应度 vs 迭代, 对数 Y)
%     2) metaheuristic_benchmark.csv     (Table9 式: Min/Max/Mean/Std/Time/95%CI/p)
%
%  调试时可先将 N_RUNS 设为 3--5；正式结果使用预设的 20 次独立运行。
% ====================================================================

clear; clc;

%% ---- 配置 (与 RA_ALA_demo 实验3 一致) ----
N_RUNS     = 20;           % 独立运行次数 (pilot 时改 3~5)
mapSize=1000; gridStep=10; windLevel='medium'; riskLevel='dense'; complexity='high';
startPt=[80,80,60]; goalPt=[900,900,60];
ENV_SEED   = 2000;         % 基准场景种子 (固定一个 high 复杂度场景)

ala_cfg.popSize=30; ala_cfg.maxIter=60; ala_cfg.nWaypoints=8;
ala_cfg.riskWeight=15.0; ala_cfg.windLookahead=3;
popSize = ala_cfg.popSize; maxIter = ala_cfg.maxIter;

algLabels = {'RA-ALA','ALA','PSO','GA','WOA'};
nA = numel(algLabels);

%% ---- 构建环境/代价/优化问题 (与 runRA_ALA 内部完全一致) ----
fprintf('构建基准场景 (complexity=%s, seed=%d)...\n', complexity, ENV_SEED);
env = CityEnvironment(mapSize, gridStep);
env.generate(complexity, windLevel, riskLevel, ENV_SEED);
env.setTaskPoints(startPt, goalPt);
cm = UnifiedCostModel();
cm.setEnvironment(env.windField, env.dynObstacles, env.heightMap);
pl = PathPlanners(env, cm); pl.setBudget(15, 5000, 2000);

start = startPt; goal = goalPt;
nWP = ala_cfg.nWaypoints; minH = pl.minH; maxH = pl.maxH;
dirVec = goal(1:2)-start(1:2); totalDist = norm(dirVec);
dirUnit = dirVec/max(totalDist,1); perpUnit = [-dirUnit(2), dirUnit(1)];
dim = nWP*2; maxLat = 200;
lb = repmat([-maxLat, minH], 1, nWP);
ub = repmat([ maxLat, maxH], 1, nWP);
[diffS, diffInfo] = estimateEnvDifficulty(env, start, goal);
ala_cfg.difficultyScale = diffS; ala_cfg.difficultyInfo = diffInfo;
t_start = 0; hasPayload = true;
param2path = @(x) paramToPath(x, start, goal, nWP, dirUnit, perpUnit, totalDist, env, minH);
evalFcn    = @(x) evalRA_v2(x, param2path, t_start, hasPayload, cm, env, ala_cfg);

%% ---- 存储 ----
global RA_NFE                            % 搜索侧函数评估计数器 (evalRA_v2 内自增)
PLOT_NFE = true;                         % 是否额外输出"按 NFE 对齐"的公平对比图
curves   = nan(nA, maxIter, N_RUNS);   % 收敛曲线 (best-so-far 适应度)
finalFit = nan(nA, N_RUNS);            % 最终最优适应度
times    = nan(nA, N_RUNS);            % 单次运行耗时
nfe_total = nan(nA, N_RUNS);           % 每个算法每次运行的搜索评估总次数

for r = 1:N_RUNS
    fprintf('\n==================== Run %d / %d ====================\n', r, N_RUNS);
    runSeed = 1000 + r*17;             % 每个 run 一个固定种子, 5 算法共用 → 同初始条件

    % --- 1. RA-ALA (本文主方法: ALA + 自适应DE + 停滞重启 + 信息化初始化; 不含搜索侧逆风前瞻罚) ---
    rng(runSeed);
    cfg_ra = ala_cfg; cfg_ra.ablate_windPenalty = true;   % 主方法不引入逆风前瞻罚(A/B 显示其对终值无显著贡献)
    RA_NFE = 0;
    tic; [~,~,det] = runRA_ALA(pl, cm, env, start, goal, t_start, hasPayload, cfg_ra);
    times(1,r) = toc;
    curves(1,:,r) = padcurve(det.conv_hist, maxIter);
    nfe_total(1,r) = RA_NFE;

    % --- 2. ALA (基础版: 原始 Lévy, 无 DE / 无停滞重启; 用于显示 DE+重启的增益) ---
    rng(runSeed);
    cfg_va = ala_cfg; cfg_va.useDE = false; cfg_va.stagRestart = false; cfg_va.useWindBias = false;
    RA_NFE = 0;
    tic; [~,~,det] = runRA_ALA(pl, cm, env, start, goal, t_start, hasPayload, cfg_va);
    times(2,r) = toc;
    curves(2,:,r) = padcurve(det.conv_hist, maxIter);
    nfe_total(2,r) = RA_NFE;

    % --- 3. PSO ---
    rng(runSeed);
    RA_NFE = 0;
    tic; [~,~,cv] = run_pso(evalFcn, lb, ub, popSize, maxIter); times(3,r) = toc;
    curves(3,:,r) = cv;
    nfe_total(3,r) = RA_NFE;

    % --- 4. GA ---
    rng(runSeed);
    RA_NFE = 0;
    tic; [~,~,cv] = run_ga(evalFcn, lb, ub, popSize, maxIter); times(4,r) = toc;
    curves(4,:,r) = cv;
    nfe_total(4,r) = RA_NFE;

    % --- 5. WOA ---
    rng(runSeed);
    RA_NFE = 0;
    tic; [~,~,cv] = run_woa(evalFcn, lb, ub, popSize, maxIter); times(5,r) = toc;
    curves(5,:,r) = cv;
    nfe_total(5,r) = RA_NFE;

    for a = 1:nA, finalFit(a,r) = curves(a,end,r); end
    fprintf('  最终适应度: RA-ALA=%.2f | ALA=%.2f | PSO=%.2f | GA=%.2f | WOA=%.2f\n', ...
        finalFit(1,r), finalFit(2,r), finalFit(3,r), finalFit(4,r), finalFit(5,r));
end

%% ====================================================================
%%  图13: 收敛曲线 (均值, 对数 Y)
%% ====================================================================
meanCurve = mean(curves, 3, 'omitnan');     % nA × maxIter
figure('Units','centimeters','Position',[2 2 20 12],'Color','w'); hold on;
styles = {'-','--','-.',':','-'};
cols   = [0.85 0.15 0.15;   % RA-ALA 红 (主角)
          0.20 0.55 0.78;   % ALA 蓝
          0.22 0.70 0.35;   % PSO 绿
          0.55 0.30 0.70;   % GA 紫
          0.90 0.55 0.10];  % WOA 橙
lwd = [2.6 2.0 2.0 2.0 2.0];
hP = gobjects(nA,1);
for a = nA:-1:1     % 倒序画, RA-ALA 最后画在最上层
    hP(a) = plot(1:maxIter, meanCurve(a,:), styles{a}, 'Color', cols(a,:), 'LineWidth', lwd(a));
end
set(gca, 'YScale', 'log', 'FontSize', 12, 'FontName','Times New Roman');
xlabel('Iterations', 'FontSize', 14);
ylabel('Fitness Value (log scale)', 'FontSize', 14);
legend(hP, algLabels, 'Location','northeast', 'FontSize', 12, 'Box','on');
title('Convergence Comparison of Optimization Algorithms (High Complexity)', 'FontSize', 14, 'FontWeight','bold');
grid on; box on; xlim([1 maxIter]);
saveas(gcf, 'fig_convergence_comparison.png');
fprintf('\n✓ 收敛图已保存: fig_convergence_comparison.png\n');

%% ====================================================================
%%  图13b (可选): 按 NFE 对齐的收敛曲线 (公平对比, 横轴=累计函数评估次数)
%% ====================================================================
% 原理: 各算法搜索侧评估次数不同(RA-ALA 含重启/多种子等), 仅按"迭代"对比对
%       评估更多的一方有利。此图把每条曲线横轴线性映射到其【真实累计 NFE】,
%       端点落在各自实际评估总数处, 提供公平视角。
meanNFE = mean(nfe_total, 2, 'omitnan');     % nA × 1
fprintf('\n  各算法平均搜索评估次数 (NFE):\n');
for a = 1:nA
    fprintf('    %-14s : %8.0f\n', algLabels{a}, meanNFE(a));
end
if PLOT_NFE
    figure('Units','centimeters','Position',[2 2 20 12],'Color','w'); hold on;
    hN = gobjects(nA,1);
    for a = nA:-1:1
        x_nfe = (meanNFE(a)/maxIter) * (1:maxIter);   % iter t → 累计 NFE
        hN(a) = plot(x_nfe, meanCurve(a,:), styles{a}, 'Color', cols(a,:), 'LineWidth', lwd(a));
    end
    set(gca, 'YScale', 'log', 'FontSize', 12, 'FontName','Times New Roman');
    xlabel('Cumulative Function Evaluations (NFE)', 'FontSize', 14);
    ylabel('Fitness Value (log scale)', 'FontSize', 14);
    legend(hN, algLabels, 'Location','northeast', 'FontSize', 12, 'Box','on');
    title('Convergence Aligned by Function Evaluations (Fair Comparison)', 'FontSize', 14, 'FontWeight','bold');
    grid on; box on;
    saveas(gcf, 'fig_convergence_NFE.png');
    fprintf('  ✓ NFE 对齐图已保存: fig_convergence_NFE.png\n');
end

%% ====================================================================
%%  收敛速度量化: 首次达到适应度阈值所需迭代数 (越少越快, 本文核心卖点)
%% ====================================================================
THR_CONV = 15;     % "达到良好解"的适应度门槛 (可行解通常 10~12, 15 为合理阈值)
itHit = nan(nA, N_RUNS);
for a = 1:nA
    for r = 1:N_RUNS
        idx = find(curves(a,:,r) <= THR_CONV, 1, 'first');
        if ~isempty(idx), itHit(a,r) = idx; end
    end
end
fprintf('\n');
fprintf('================================================================================\n');
fprintf('  收敛速度: 首次达到 fitness <= %.0f 所需迭代数 (N=%d; 越少越快)\n', THR_CONV, N_RUNS);
fprintf('================================================================================\n');
fprintf('  %-12s | %8s | %9s | %10s\n', 'Method','Reach%','Med-Iter','Mean-Iter');
fprintf('  %s\n', repmat('-',1,48));
for a = 1:nA
    hits = itHit(a,:); reached = hits(~isnan(hits));
    rp = 100*numel(reached)/N_RUNS;
    if ~isempty(reached)
        fprintf('  %-12s | %7.0f%% | %9.1f | %10.1f\n', algLabels{a}, rp, median(reached), mean(reached));
    else
        fprintf('  %-12s | %7.0f%% | %9s | %10s\n', algLabels{a}, rp, '—', '—');
    end
end
fprintf('================================================================================\n');
fprintf('  注: 仅统计 %d 代内达标的运行; Reach%%=达标占比。RA-ALA 应有最小迭代数(收敛最快)。\n', maxIter);

%% ====================================================================
%%  Table9 式基准表
%% ====================================================================
fprintf('\n');
fprintf('================================================================================\n');
fprintf('  Table. 不同优化方法在统一代价下的对比 (N=%d 次独立运行)\n', N_RUNS);
fprintf('================================================================================\n');
FEAS_THR = 100;   % finalFit >= 此值视为不可行(吃了 +150 罚; 实测可行<25, 不可行>200)
feas_base = finalFit(1, finalFit(1,:) < FEAS_THR & isfinite(finalFit(1,:)));   % RA-ALA 可行解(p 检验基线)
fprintf('  %-12s | %6s | %9s | %-15s | %9s | %8s | %8s | %-10s\n', ...
    'Method','Succ%','Feas-Med','Feas-IQR[Q1,Q3]','Feas-Mean','Feas-Std','Time(s)','p(vs RA)');
fprintf('  %s\n', repmat('-',1,104));

rows = {};
for a = 1:nA
    allv = finalFit(a,:); allv = allv(isfinite(allv));
    feas = allv(allv < FEAS_THR);                       % 可行解
    succ = 100 * numel(feas) / max(numel(allv),1);
    if ~isempty(feas)
        fmed=median(feas); q1=qtile(feas,25); q3=qtile(feas,75); fmean=mean(feas); fstd=std(feas);
    else
        fmed=NaN; q1=NaN; q3=NaN; fmean=NaN; fstd=NaN;
    end
    tm = mean(times(a,:),'omitnan');
    if a == 1
        pstr = '—'; pval = NaN;
    elseif numel(feas) >= 2 && numel(feas_base) >= 2
        pval = ttest2p(feas_base, feas);                % 仅用可行解做双尾 t 检验
        if pval<0.001, pstr='<0.001'; elseif pval<0.01, pstr='<0.01'; ...
        elseif pval<0.05, pstr='<0.05'; else, pstr=sprintf('%.3f',pval); end
    else
        pstr = 'n/a'; pval = NaN;
    end
    fprintf('  %-12s | %5.0f%% | %9.2f | [%6.2f,%6.2f] | %9.2f | %8.3f | %8.2f | %-10s\n', ...
        algLabels{a}, succ, fmed, q1, q3, fmean, fstd, tm, pstr);
    rows(end+1,:) = {algLabels{a}, succ, fmed, q1, q3, fmean, fstd, tm, pval}; %#ok<AGROW>
end
fprintf('================================================================================\n');
fprintf('  * 成功率=可行解占比(可行: finalFit<%g); 中位数/IQR/均值/Std 仅基于可行解;\n', FEAS_THR);
fprintf('    p 为可行解的双尾 t 检验(正态近似, vs RA-ALA)。\n');

T = cell2table(rows, 'VariableNames', ...
    {'Method','Success_pct','Feas_Median','Feas_Q1','Feas_Q3','Feas_Mean','Feas_Std','Time_s','p_vs_RAALA'});
writetable(T, 'metaheuristic_benchmark.csv');
fprintf('\n✓ 基准表已保存: metaheuristic_benchmark.csv\n');
fprintf('\n========== 收敛性基准完成 ==========\n');


%% ====================================================================
%%  局部函数
%% ====================================================================
function c = padcurve(h, T)
    % runRA_ALA 的 conv_hist (maxIter×1) → 1×T 的 best-so-far 单调曲线
    h = h(:)';
    if numel(h) < T
        if isempty(h), h = 1e12; end
        h = [h, repmat(h(end), 1, T-numel(h))];
    elseif numel(h) > T
        h = h(1:T);
    end
    % 前向填充: 把 0 / 非有限值替换为前一个有效值, 避免 cummin 被 0 压垮
    last = NaN;
    for i = 1:numel(h)
        if ~isfinite(h(i)) || h(i) <= 0
            h(i) = last;
        else
            last = h(i);
        end
    end
    if all(~isfinite(h)), c = ones(1,T)*1e12; return; end
    fv = find(isfinite(h), 1, 'first');           % 开头若仍为 NaN, 用首个有效值回填
    if fv > 1, h(1:fv-1) = h(fv); end
    c = cummin(h);    % 保证非增 (best-so-far)
end

function f = safeEval(fit, x)
    try
        f = fit(x);
        if ~isfinite(f), f = 1e12; end
    catch
        f = 1e12;
    end
end

%% ---------- PSO ----------
function [gbest, gbestF, curve] = run_pso(fit, lb, ub, n, T)
    dim = numel(lb);
    X = lb + rand(n,dim).*(ub-lb);
    V = zeros(n,dim);
    F = zeros(n,1);
    for i=1:n, F(i) = safeEval(fit, X(i,:)); end
    pbest = X; pbestF = F;
    [gbestF, gi] = min(F); gbest = X(gi,:);
    curve = zeros(1,T);
    w1=0.9; w2=0.4; c1=1.5; c2=1.5; vmax = 0.2*(ub-lb);
    for t = 1:T
        w = w1 - (w1-w2)*t/T;
        for i = 1:n
            r1 = rand(1,dim); r2 = rand(1,dim);
            V(i,:) = w*V(i,:) + c1*r1.*(pbest(i,:)-X(i,:)) + c2*r2.*(gbest - X(i,:));
            V(i,:) = max(-vmax, min(vmax, V(i,:)));
            X(i,:) = max(lb, min(ub, X(i,:) + V(i,:)));
            f = safeEval(fit, X(i,:));
            if f < pbestF(i), pbestF(i)=f; pbest(i,:)=X(i,:); end
            if f < gbestF,   gbestF=f;   gbest=X(i,:);       end
        end
        curve(t) = gbestF;
    end
end

%% ---------- GA (实数编码: 锦标赛 + BLX-α 交叉 + 高斯变异 + 精英) ----------
function [best, bestF, curve] = run_ga(fit, lb, ub, n, T)
    dim = numel(lb);
    X = lb + rand(n,dim).*(ub-lb);
    F = zeros(n,1);
    for i=1:n, F(i)=safeEval(fit, X(i,:)); end
    [bestF, bi] = min(F); best = X(bi,:);
    curve = zeros(1,T);
    pc=0.9; pm=0.12; alpha=0.5; eta=0.1;
    for t = 1:T
        newX = zeros(n,dim);
        newX(1,:) = best;             % 精英保留
        k = 2;
        while k <= n
            p1 = ga_tour(X,F); p2 = ga_tour(X,F);
            if rand < pc
                g  = (1+2*alpha)*rand(1,dim) - alpha;   % BLX-alpha
                c1 = p1 + g.*(p2-p1);
                c2 = p2 + g.*(p1-p2);
            else
                c1 = p1; c2 = p2;
            end
            c1 = ga_mut(c1, lb, ub, pm, eta);
            c2 = ga_mut(c2, lb, ub, pm, eta);
            newX(k,:) = max(lb, min(ub, c1));
            if k+1 <= n, newX(k+1,:) = max(lb, min(ub, c2)); end
            k = k + 2;
        end
        X = newX;
        for i=1:n, F(i)=safeEval(fit, X(i,:)); end
        [mf, mi] = min(F);
        if mf < bestF, bestF=mf; best=X(mi,:); end
        [wf, wi] = max(F);                         % 保证精英在群中
        if bestF < wf, X(wi,:)=best; F(wi)=bestF; end
        curve(t) = bestF;
    end
end

function p = ga_tour(X, F)
    n = size(X,1); i = randi(n); j = randi(n);
    if F(i) <= F(j), p = X(i,:); else, p = X(j,:); end
end

function c = ga_mut(c, lb, ub, pm, eta)
    m = rand(1,numel(c)) < pm;
    c(m) = c(m) + eta*(ub(m)-lb(m)).*randn(1,sum(m));
end

%% ---------- WOA ----------
function [best, bestF, curve] = run_woa(fit, lb, ub, n, T)
    dim = numel(lb);
    X = lb + rand(n,dim).*(ub-lb);
    F = zeros(n,1);
    for i=1:n, F(i)=safeEval(fit, X(i,:)); end
    [bestF, bi] = min(F); best = X(bi,:);
    curve = zeros(1,T);
    b = 1;
    for t = 1:T
        a = 2 - 2*t/T;                 % 线性 2→0
        for i = 1:n
            r = rand; A = 2*a*r - a; C = 2*rand; p = rand;
            if p < 0.5
                if abs(A) < 1
                    D = abs(C*best - X(i,:));
                    X(i,:) = best - A*D;
                else
                    rl = randi(n); Xr = X(rl,:);
                    D = abs(C*Xr - X(i,:));
                    X(i,:) = Xr - A*D;
                end
            else
                l = 2*rand - 1;        % l ∈ [-1,1]
                D = abs(best - X(i,:));
                X(i,:) = D.*exp(b*l).*cos(2*pi*l) + best;
            end
            X(i,:) = max(lb, min(ub, X(i,:)));
            f = safeEval(fit, X(i,:));
            if f < bestF, bestF=f; best=X(i,:); end
        end
        curve(t) = bestF;
    end
end

%% ---------- 百分位 (线性插值, 无需 Statistics Toolbox) ----------
function q = qtile(x, p)
    x = sort(x(:)); n = numel(x);
    if n==0, q=NaN; return; end
    if n==1, q=x(1); return; end
    idx = (p/100)*(n-1) + 1;
    lo = floor(idx); hi = ceil(idx); f = idx - lo;
    q = x(lo)*(1-f) + x(hi)*f;
end

%% ---------- 双尾两样本 t 检验 (正态近似, 无需 Statistics Toolbox) ----------
function p = ttest2p(x, y)
    x = x(isfinite(x)); y = y(isfinite(y));
    nx = numel(x); ny = numel(y);
    if nx < 2 || ny < 2, p = 1; return; end
    se = sqrt(var(x)/nx + var(y)/ny);
    if se == 0, p = 1; return; end
    tval = (mean(x) - mean(y)) / se;
    p = erfc(abs(tval)/sqrt(2));    % 正态近似双尾 p
end
