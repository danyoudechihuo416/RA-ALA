%% =========================================================================
%%  diagnoseSingleCasePerformance.m
%%  单案例诊断模式 — 分析 RA-ALA 在指定场景下 J 异常的根本原因
%% =========================================================================
%%
%%  问题背景:
%%    high 城市 / t=0s 场景下, RA-ALA 输出 E≈18Wh、T≈210s、Risk=0,
%%    但 J≈699.8, 远高于其他时刻 (~16~20). 本脚本对该场景进行
%%    全链路代价拆解, 定位异常来源.
%%
%%  诊断内容:
%%    1. raw / smooth / repair 三阶段路径的完整代价分解
%%    2. 最终路径逐段诊断 (时间/风险/惩罚/NFZ/障碍接近度)
%%    3. 异常段标红警告
%%    4. 重复计费检测 (对比修复前/修复后 UnifiedCostModel 版本)
%%    5. penalty 拆解饼图 + 逐段惩罚热图
%%
%%  用法:
%%    直接运行, 无需修改其他文件.
%%    依赖: CityEnvironment.m, UnifiedCostModel.m, PathPlanners.m,
%%           runRA_ALA.m (需在同一目录下以获取 runRA_ALA 函数)
%% =========================================================================

clear; clc; close all;
warning off;

fprintf('╔═════════════════════════════════════════════════════════════╗\n');
fprintf('║  RA-ALA 单案例诊断 — J 异常根因分析                        ║\n');
fprintf('╚═════════════════════════════════════════════════════════════╝\n\n');

%% ==================== 可调参数 ====================
DIAG_SEED       = 42;
DIAG_CITY       = 'high';    % 城市复杂度: low / medium / high
DIAG_T_START    = 0;         % 出发时刻 (s), 改成其他值可对比
DIAG_WIND       = 'medium';
DIAG_RISK       = 'dense';
DIAG_MAP_SIZE   = 1000;
DIAG_GRID_STEP  = 10;
DIAG_START_PT   = [80,  80,  60];
DIAG_GOAL_PT    = [900, 900, 60];

% ALA 超参 (与 runRA_ALA.m 保持一致)
ala_cfg.popSize       = 30;
ala_cfg.maxIter       = 60;
ala_cfg.nWaypoints    = 8;
ala_cfg.riskWeight    = 15.0;
ala_cfg.windLookahead = 3;

% 诊断阈值 (超过即标为异常)
THRESH_SEG_PENALTY  = 0.05;   % 单段 penalty > 此值视为异常
THRESH_NFZ_MARGIN   = 1.5;    % 到 NFZ 中心的距离 < radius×此倍数 时警告
THRESH_OBS_MARGIN   = 2.5;    % 到动态障碍的距离 < radius×此倍数 时警告
THRESH_HEIGHT_BELOW = 5;      % 低于 H_min 超过此值 (m) 视为严重违规

%% ==================== 构建环境 ====================

fprintf('[1/6] 构建 %s 城市环境 (t=%.0fs)...\n', DIAG_CITY, DIAG_T_START);
rng(DIAG_SEED);
env = CityEnvironment(DIAG_MAP_SIZE, DIAG_GRID_STEP);
env.generate(DIAG_CITY, DIAG_WIND, DIAG_RISK, DIAG_SEED);
env.setTaskPoints(DIAG_START_PT, DIAG_GOAL_PT);

costModel = UnifiedCostModel();
costModel.setEnvironment(env.windField, env.dynObstacles, env.heightMap);

planner = PathPlanners(env, costModel);
planner.setBudget(15, 5000, 2000);

fprintf('   建筑物: %d  动态障碍: %d  禁飞区: %d\n', ...
    size(env.buildings, 1), ...
    length(env.dynObstacles.movingObs), ...
    length(env.dynObstacles.tempNFZ));

% 打印 NFZ 详情
fprintf('   NFZ 列表:\n');
for ni = 1:length(env.dynObstacles.tempNFZ)
    nfz = env.dynObstacles.tempNFZ(ni);
    active_at_t0 = (DIAG_T_START >= nfz.t_start && DIAG_T_START <= nfz.t_end);
    fprintf('     NFZ%d  center=[%.0f,%.0f]  r=%.0f  h=[%.0f,%.0f]  t=[%.0f,%.0f]', ...
        ni, nfz.center(1), nfz.center(2), nfz.radius, ...
        nfz.height(1), nfz.height(2), nfz.t_start, nfz.t_end);
    if active_at_t0
        fprintf('  ← ★ 在 t=%.0fs 时 ACTIVE', DIAG_T_START);
    end
    fprintf('\n');
end

%% ==================== 运行 RA-ALA 并取三阶段路径 ====================

fprintf('\n[2/6] 运行 RA-ALA (seed=%d)...\n', DIAG_SEED + 100);
rng(DIAG_SEED + 100);
[final_path, final_cost, final_det, stage_det] = ...
    runRA_ALA(planner, costModel, env, DIAG_START_PT, DIAG_GOAL_PT, ...
              DIAG_T_START, true, ala_cfg);

fprintf('   完成. final_J=%.3f  internal_search_J=%.3f\n', ...
    final_cost, final_det.internal_search_cost);

%% ==================== [诊断1] 三阶段代价分解 ====================

fprintf('\n[3/6] 三阶段代价分解\n');
fprintf('%s\n', repmat('═', 1, 72));

stages  = {stage_det.raw, stage_det.smooth, stage_det.repair};
slabels = {'raw path (ALA直接输出)', 'smooth path (样条平滑后)', 'repair path (修复后=最终)'};

for si = 1:3
    d   = stages{si};
    lbl = slabels{si};
    fprintf('\n  ─── 阶段 %d: %s ───\n', si, lbl);
    fprintf('  路径点数: %d\n', length(d.t_arrivals));
    fprintf('  J_final      = %10.4f\n', d.J_final);
    fprintf('    w_e×E      = %10.4f   (E       = %.4f Wh)\n',  1.0*d.E_total, d.E_total);
    fprintf('    w_t×T/60   = %10.4f   (T       = %.2f s)\n',   0.5*d.T_total/60, d.T_total);
    fprintf('    w_c×C      = %10.4f   (C_climb = %.6f)\n',     2.0*d.C_climb, d.C_climb);
    fprintf('    w_r×R      = %10.4f   (R_dyn   = %.6f)\n',     10.0*d.R_dynamic, d.R_dynamic);
    fprintf('    λ×Pen      = %10.4f   (Pen     = %.6f)\n',     100*d.penalty_total, d.penalty_total);
    fprintf('  ┌─ Penalty 分解 ──────────────────────────────────────────\n');
    fprintf('  │  height            = %8.6f  → J贡献 %7.3f\n', d.penalty_height, 100*d.penalty_height);
    fprintf('  │  static_collision  = %8.6f  → J贡献 %7.3f\n', d.penalty_static_collision, 100*d.penalty_static_collision);
    fprintf('  │  dynamic_collision = %8.6f  → J贡献 %7.3f\n', d.penalty_dynamic_collision, 100*d.penalty_dynamic_collision);
    fprintf('  │  battery           = %8.6f  → J贡献 %7.3f\n', d.penalty_battery, 100*d.penalty_battery);
    fprintf('  └─ NFZ硬罚(已入J)      = %8.3f\n', d.NFZ_penalty);
    fprintf('  feasible=%d  heightViol=%d  SoC_end=%.1f%%\n', ...
        d.feasible, d.heightViolations, d.SoC_end*100);
end

% 阶段间对比
fprintf('\n  ─── 阶段间代价变化 ───\n');
fprintf('  raw→smooth:  ΔJ=%+.3f  ΔPen=%+.6f  ΔhViol=%+d\n', ...
    stages{2}.J_final - stages{1}.J_final, ...
    stages{2}.penalty_total - stages{1}.penalty_total, ...
    stages{2}.heightViolations - stages{1}.heightViolations);
fprintf('  smooth→repair: ΔJ=%+.3f  ΔPen=%+.6f  ΔhViol=%+d\n', ...
    stages{3}.J_final - stages{2}.J_final, ...
    stages{3}.penalty_total - stages{2}.penalty_total, ...
    stages{3}.heightViolations - stages{2}.heightViolations);
fprintf('  internal_search_J=%+.3f  vs  final_J=%.3f  差值=%+.3f\n', ...
    stage_det.internal_search_cost, final_det.J_final, ...
    stage_det.internal_search_cost - final_det.J_final);

% 重复计费检测
if stages{3}.penalty_total > stages{2}.penalty_total + 0.01
    fprintf('\n  [!] ★ 检测到 repair 阶段引入额外惩罚!\n');
    fprintf('      smooth.penalty=%.4f → repair.penalty=%.4f (增加%.4f)\n', ...
        stages{2}.penalty_total, stages{3}.penalty_total, ...
        stages{3}.penalty_total - stages{2}.penalty_total);
    fprintf('      可能原因: postSmoothRepair 移点操作把路径点推入建筑或低于 H_min\n');
end

%% ==================== [诊断2] 逐段诊断 ====================

fprintf('\n[4/6] 最终路径逐段诊断\n');
fprintf('%s\n', repmat('═', 1, 72));

path  = final_path;
det   = final_det;
N     = size(path, 1);
nSegs = N - 1;

tArr  = det.t_arrivals;       % [N×1] 每点到达时刻
ESeg  = det.E_segments;       % [N-1]
TSeg  = det.T_segments;       % [N-1]

% 从 evaluatePath 中我们拿到了段级数组; 但 R/penalty 按段级暂无单独字段,
% 需要对最终路径做一次"带段级输出"的详细再评估
[~, seg_diag] = evaluatePath_segLevel(costModel, env, path, DIAG_T_START, true);

% 确认段数一致
assert(length(seg_diag) == nSegs, '段数不一致, 检查 evaluatePath_segLevel 输出');

% 打印表头
fprintf('\n  %4s %8s %8s %8s %8s %6s %6s %6s %6s  %s\n', ...
    'seg', 't_start', 'dt(s)', 'E(Wh)', 'R_risk', ...
    'Pen', 'Pen_h', 'Pen_s', 'Pen_d', '警告');
fprintf('  %s\n', repmat('-', 1, 90));

n_anomalous  = 0;
anomaly_segs = [];

for k = 1:nSegs
    sd = seg_diag(k);

    % 基础值
    t_seg_start = tArr(k);
    dt          = TSeg(k);
    e_seg       = ESeg(k);
    r_seg       = sd.R_risk;
    p_total     = sd.Pen_total;
    p_h         = sd.Pen_height;
    p_s         = sd.Pen_static;
    p_d         = sd.Pen_dyn;

    % 构造警告字符串
    warn = '';
    if p_total > THRESH_SEG_PENALTY
        warn = [warn sprintf('[PEN=%.3f!]', p_total)];
        n_anomalous = n_anomalous + 1;
        anomaly_segs(end+1) = k; %#ok<AGROW>
    end
    if sd.nfz_proximity_flag
        warn = [warn sprintf('[NFZ近(%.0fm)]', sd.nfz_min_dist)];
    end
    if sd.obs_proximity_flag
        warn = [warn sprintf('[OBS近(%.0fm)]', sd.obs_min_dist)];
    end
    if sd.height_below > THRESH_HEIGHT_BELOW
        warn = [warn sprintf('[H_低%.1fm!]', sd.height_below)];
    end
    if sd.static_hit
        warn = [warn '[穿楼!]'];
    end

    % 选择性打印: 异常段全打; 正常段每5段打一次
    if p_total > THRESH_SEG_PENALTY || sd.nfz_proximity_flag || ...
       sd.obs_proximity_flag || sd.height_below > 1 || mod(k, 5) == 0
        fprintf('  %4d %8.1f %8.2f %8.4f %8.5f %6.4f %6.4f %6.4f %6.4f  %s\n', ...
            k, t_seg_start, dt, e_seg, r_seg, p_total, p_h, p_s, p_d, warn);
    end
end

fprintf('\n  汇总: 共 %d 段, 异常段 %d 个 (penalty>%.3f)\n', ...
    nSegs, n_anomalous, THRESH_SEG_PENALTY);
if ~isempty(anomaly_segs)
    fprintf('  ★ 异常段索引: [%s]\n', num2str(anomaly_segs));
end

%% ==================== [诊断3] 逐点高度检查 ====================

fprintf('\n[5/6] 逐点高度合规性检查\n');
fprintf('%s\n', repmat('═', 1, 72));

MS       = env.MAP_SIZE;
H_min    = costModel.H_min;
H_max    = costModel.H_max;
H_clear  = costModel.H_clearance;

n_viol = 0;
viol_detail = {};

fprintf('  %4s %8s %8s %8s %8s %8s  %s\n', ...
    'idx', 'x', 'y', 'z', 'ground_h', 'min_floor', '状态');
fprintf('  %s\n', repmat('-', 1, 72));

for k = 1:N
    pt = path(k, :);
    ex = max(1, min(MS, round(pt(1))));
    ey = max(1, min(MS, round(pt(2))));
    ground_h  = env.heightMap(ex, ey);
    min_floor = max(H_min, ground_h + H_clear);

    if pt(3) < min_floor || pt(3) > H_max
        n_viol = n_viol + 1;
        if pt(3) < min_floor
            status = sprintf('★ 低%.1fm (需>=%.1f)', min_floor - pt(3), min_floor);
        else
            status = sprintf('★ 高%.1fm (需<=%.1f)', pt(3) - H_max, H_max);
        end
        viol_detail{end+1} = sprintf('  点%3d: [%.1f,%.1f,%.1f]  ground=%.1f  min=%.1f  → %s', ...
            k, pt(1), pt(2), pt(3), ground_h, min_floor, status);
        fprintf('  %4d %8.1f %8.1f %8.1f %8.1f %8.1f  %s\n', ...
            k, pt(1), pt(2), pt(3), ground_h, min_floor, status);
    end
end

if n_viol == 0
    fprintf('  ✓ 所有 %d 个路径点均满足高度约束\n', N);
else
    fprintf('\n  ★ 共 %d 个路径点违反高度约束\n', n_viol);
    fprintf('  高度惩罚 = Σ(违反量/10) = %.4f  → λ×Pen_h = %.3f\n', ...
        final_det.penalty_height, 100*final_det.penalty_height);
end

%% ==================== [诊断4] NFZ 与障碍接近度全路径扫描 ====================

fprintf('\n[6/6] 全路径 NFZ / 动态障碍接近度扫描\n');
fprintf('%s\n', repmat('═', 1, 72));

nfzList = env.dynObstacles.tempNFZ;
obsList = env.dynObstacles.movingObs;

% NFZ 接近度
fprintf('  NFZ 接近度 (路径点 vs 所有活跃 NFZ):\n');
fprintf('  %4s %8s %8s  %s\n', 'idx', 't(s)', 'z(m)', '最近 NFZ 信息');
fprintf('  %s\n', repmat('-', 1, 64));

for k = 1:N
    pt = path(k,:);
    tk = tArr(k);
    for ni = 1:length(nfzList)
        nfz = nfzList(ni);
        if ~nfz.active || tk < nfz.t_start || tk > nfz.t_end, continue; end
        dh = norm(pt(1:2) - nfz.center);
        if dh < nfz.radius * THRESH_NFZ_MARGIN
            inXY = (dh < nfz.radius);
            inZ  = (pt(3) >= nfz.height(1) && pt(3) <= nfz.height(2));
            status = '接近';
            if inXY && inZ,  status = '★ 穿越!'; end
            if inXY && ~inZ, status = '★ XY在内(Z规避)'; end
            fprintf('  %4d %8.1f %8.1f  NFZ%d: dh=%.1fm(r=%.0f) z=[%.0f,%.0f] → %s\n', ...
                k, tk, pt(3), ni, dh, nfz.radius, nfz.height(1), nfz.height(2), status);
        end
    end
end

% 动态障碍接近度
fprintf('\n  动态障碍接近度 (路径点 vs 所有移动障碍):\n');
fprintf('  %4s %8s  %s\n', 'idx', 't(s)', '接近障碍信息');
fprintf('  %s\n', repmat('-', 1, 64));

for k = 1:N
    pt = path(k,:);
    tk = tArr(k);
    for oi = 1:length(obsList)
        obs = obsList(oi);
        op  = env.dynObstacles.getPosition(oi, tk);
        d   = norm(pt - op);
        if d < obs.radius * THRESH_OBS_MARGIN
            status = '接近';
            if d < obs.radius, status = '★ 碰撞!'; end
            fprintf('  %4d %8.1f  Obs%d: d=%.1fm(r=%.0f) pos=[%.0f,%.0f,%.0f] → %s\n', ...
                k, tk, oi, d, obs.radius, op(1), op(2), op(3), status);
        end
    end
end

%% ==================== 生成可视化图表 ====================

fprintf('\nGenerating diagnostic figures...\n');

% ---- 图1: J 组成饼图 (三阶段对比) ----
fig_pie = figure('Units','centimeters','Position',[1 1 28 10],'Color','w');
stage_labels_pie = {'raw', 'smooth', 'repair'};
for si = 1:3
    d = stages{si};
    J_e   = 1.0   * d.E_total;
    J_t   = 0.5   * d.T_total / 60;
    J_c   = 2.0   * d.C_climb;
    J_r   = 10.0  * d.R_dynamic;
    J_pen = 100.0 * d.penalty_total;

    subplot(1, 3, si);
    vals  = [J_e, J_t, J_c, J_r, J_pen];
    lbls  = {'w_e×E', 'w_t×T/60', 'w_c×C', 'w_r×R', 'λ×Pen'};
    clrs  = [0.2 0.6 0.9; 0.3 0.8 0.4; 0.9 0.7 0.2; 0.9 0.4 0.4; 0.7 0.2 0.7];

    % 过滤掉零或负值
    mask = vals > 1e-6;
    if any(mask)
        pie_h = pie(vals(mask));
        % 着色
        pie_patches = pie_h(1:2:end);
        text_labels = pie_h(2:2:end);
        colors_used = clrs(mask, :);
        for pi_i = 1:length(pie_patches)
            pie_patches(pi_i).FaceColor = colors_used(pi_i,:);
        end
    end
    title(sprintf('%s  J=%.2f', stage_labels_pie{si}, d.J_final), ...
        'FontSize', 10, 'FontWeight', 'bold');
    legend(lbls(mask), 'Location', 'southoutside', 'FontSize', 7);
end
sgtitle(sprintf('J 组成 (三阶段对比)  city=%s  t=%.0fs', DIAG_CITY, DIAG_T_START), ...
    'FontSize', 12, 'FontWeight', 'bold');
exportPublicationFigure(fig_pie, sprintf('diag_pie_%s_t%d.png', DIAG_CITY, DIAG_T_START));

% ---- 图2: 逐段惩罚热图 ----
fig_heat = figure('Units','centimeters','Position',[1 1 30 14],'Color','w');

pen_matrix = zeros(4, nSegs);  % 行: h/s/d/total
for k = 1:nSegs
    pen_matrix(1, k) = seg_diag(k).Pen_height;
    pen_matrix(2, k) = seg_diag(k).Pen_static;
    pen_matrix(3, k) = seg_diag(k).Pen_dyn;
    pen_matrix(4, k) = seg_diag(k).Pen_total;
end

subplot(2, 1, 1);
imagesc(pen_matrix(4, :));
colorbar; colormap(gca, hot);
set(gca, 'YTick', 1, 'YTickLabel', {'Pen_total'}, 'FontSize', 9);
xlabel('Segment index');
title(sprintf('逐段惩罚热图 (repair path, %d 段)', nSegs), 'FontSize', 10);
% 标注异常段
hold on;
for k = anomaly_segs
    text(k, 1, sprintf('%.2f', pen_matrix(4,k)), ...
        'HorizontalAlignment','center','VerticalAlignment','middle', ...
        'FontSize', 7, 'Color', 'w', 'FontWeight', 'bold');
end

subplot(2, 1, 2);
b = bar(1:nSegs, pen_matrix(1:3,:)', 'stacked');
b(1).FaceColor = [0.9 0.4 0.1];   % height
b(2).FaceColor = [0.5 0.2 0.8];   % static
b(3).FaceColor = [0.2 0.6 0.9];   % dynamic
legend({'penalty\_height','penalty\_static','penalty\_dyn'}, ...
    'Location', 'northeast', 'FontSize', 7);
xlabel('Segment index'); ylabel('Penalty');
title('逐段惩罚分解 (stacked bar)', 'FontSize', 10);
grid on;
% 标注异常段
hold on;
for k = anomaly_segs
    xline(k, 'r--', 'LineWidth', 1.2);
end

sgtitle(sprintf('逐段惩罚诊断  city=%s  t=%.0fs  (红虚线=异常段)', ...
    DIAG_CITY, DIAG_T_START), 'FontSize', 12, 'FontWeight', 'bold');
exportPublicationFigure(fig_heat, sprintf('diag_segheat_%s_t%d.png', DIAG_CITY, DIAG_T_START));

% ---- 图3: 路径高度剖面 + H_min 线 + 违规点标记 ----
fig_alt = figure('Units','centimeters','Position',[1 1 30 10],'Color','w');

cumDist = zeros(N, 1);
for k = 2:N
    cumDist(k) = cumDist(k-1) + norm(path(k,1:2) - path(k-1,1:2));
end

subplot(1, 2, 1);
hold on;
% 建筑物高度轮廓
for k = 1:N
    ex = max(1, min(MS, round(path(k,1))));
    ey = max(1, min(MS, round(path(k,2))));
    bh = env.heightMap(ex, ey);
    plot(cumDist(k), bh, '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 4);
end
% 路径高度
plot(cumDist, path(:,3), 'b-', 'LineWidth', 2);
yline(H_min, 'r--', sprintf('H_{min}=%.0fm', H_min), 'LineWidth', 1, 'FontSize', 8);
yline(H_max, 'k--', sprintf('H_{max}=%.0fm', H_max), 'LineWidth', 1, 'FontSize', 8);
% 标注违规点
for k = 1:N
    ex = max(1, min(MS, round(path(k,1))));
    ey = max(1, min(MS, round(path(k,2))));
    gh = env.heightMap(ex, ey);
    mf = max(H_min, gh + H_clear);
    if path(k,3) < mf
        plot(cumDist(k), path(k,3), 'rv', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    end
end
xlabel('水平距离 (m)'); ylabel('高度 (m)');
title('高度剖面  (▼=高度违规点)', 'FontSize', 10);
grid on; set(gca, 'FontSize', 8);

subplot(1, 2, 2);
% 二维路径 + NFZ + 障碍
hold on; axis equal;
for i = 1:size(env.buildings, 1)
    cx=env.buildings(i,1); cy=env.buildings(i,2);
    hw=env.buildings(i,4); hh=env.buildings(i,5);
    rectangle('Position',[cx-hw,cy-hh,2*hw,2*hh], ...
        'FaceColor',[0.85 0.85 0.85], 'EdgeColor',[0.6 0.6 0.6], 'LineWidth',0.2);
end
for ni = 1:length(nfzList)
    nfz = nfzList(ni);
    theta = linspace(0,2*pi,60);
    active_at_t0 = (DIAG_T_START >= nfz.t_start && DIAG_T_START <= nfz.t_end);
    fc = [1 0.85 0.85]; ec = 'r';
    if ~active_at_t0, fc=[0.95 0.95 0.95]; ec=[0.7 0.7 0.7]; end
    fill(nfz.center(1)+nfz.radius*cos(theta), nfz.center(2)+nfz.radius*sin(theta), ...
        fc, 'FaceAlpha',0.5, 'EdgeColor',ec, 'LineStyle','--', 'LineWidth',1);
    text(nfz.center(1), nfz.center(2), sprintf('NFZ%d', ni), ...
        'HorizontalAlignment','center','FontSize',6,'Color',ec*0.7);
end
for oi = 1:length(obsList)
    op = env.dynObstacles.getPosition(oi, DIAG_T_START);
    theta = linspace(0,2*pi,20);
    fill(op(1)+obsList(oi).radius*cos(theta), op(2)+obsList(oi).radius*sin(theta), ...
        [1 0.65 0.2], 'FaceAlpha',0.55, 'EdgeColor',[0.85 0.45 0]);
end
plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
% 标注异常段
for k = anomaly_segs
    p1 = path(k,1:2); p2 = path(k+1,1:2);
    plot([p1(1) p2(1)],[p1(2) p2(2)], 'r-', 'LineWidth', 3);
end
plot(DIAG_START_PT(1),DIAG_START_PT(2),'gp','MarkerSize',12,'MarkerFaceColor','g');
plot(DIAG_GOAL_PT(1), DIAG_GOAL_PT(2), 'rh','MarkerSize',12,'MarkerFaceColor','r');
xlabel('X (m)'); ylabel('Y (m)');
title('二维路径 (红色段=高惩罚)', 'FontSize', 10);
xlim([0 DIAG_MAP_SIZE]); ylim([0 DIAG_MAP_SIZE]);
grid on; set(gca,'FontSize',8);

sgtitle(sprintf('路径高度+二维轨迹诊断  city=%s  t=%.0fs', DIAG_CITY, DIAG_T_START), ...
    'FontSize', 12, 'FontWeight', 'bold');
exportPublicationFigure(fig_alt, sprintf('diag_path_%s_t%d.png', DIAG_CITY, DIAG_T_START));

% ---- 图4: penalty 逐段条形 (颜色标注高度/静态/动态) + 到 NFZ 距离 ----
fig_pen2 = figure('Units','centimeters','Position',[1 1 30 14],'Color','w');

subplot(2, 1, 1);
hold on;
x_seg = 1:nSegs;
bar(x_seg, [seg_diag.Pen_height],  0.8, 'FaceColor', [0.9 0.3 0.1], 'DisplayName', 'penalty\_height');
bar(x_seg, [seg_diag.Pen_static],  0.6, 'FaceColor', [0.5 0.1 0.8], 'DisplayName', 'penalty\_static');
bar(x_seg, [seg_diag.Pen_dyn],     0.4, 'FaceColor', [0.1 0.5 0.9], 'DisplayName', 'penalty\_dyn');
yline(THRESH_SEG_PENALTY, 'k--', sprintf('阈值 %.3f', THRESH_SEG_PENALTY), 'FontSize', 8);
for k = anomaly_segs
    text(k, max([seg_diag(k).Pen_total, THRESH_SEG_PENALTY*1.1]), ...
        sprintf('★%.3f', seg_diag(k).Pen_total), ...
        'HorizontalAlignment','center','FontSize',7,'Color','r','FontWeight','bold');
end
legend('Location','northeast','FontSize',7);
xlabel('Segment index'); ylabel('Penalty');
title('逐段惩罚构成', 'FontSize', 10); grid on;
xlim([0 nSegs+1]);

subplot(2, 1, 2);
hold on;
min_nfz_dist = inf(nSegs, 1);
for k = 1:nSegs
    t_mid  = tArr(k) + TSeg(k)/2;
    pt_mid = path(k,:) + 0.5*(path(k+1,:)-path(k,:));
    for ni = 1:length(nfzList)
        nfz = nfzList(ni);
        if ~nfz.active || t_mid < nfz.t_start || t_mid > nfz.t_end, continue; end
        dh = norm(pt_mid(1:2) - nfz.center);
        min_nfz_dist(k) = min(min_nfz_dist(k), dh);
    end
end
valid_nfz = isfinite(min_nfz_dist);
if any(valid_nfz)
    plot(find(valid_nfz), min_nfz_dist(valid_nfz), 'mo-', 'LineWidth', 1.5, 'MarkerSize', 5);
end

% 各 NFZ 半径参考线
nfz_radii = [nfzList.radius];
if ~isempty(nfz_radii)
    for ni = 1:length(nfzList)
        if any(valid_nfz)
            yline(nfzList(ni).radius, '--', sprintf('NFZ%d r=%.0f', ni, nfzList(ni).radius), ...
                'Color', [1 0.4 0.4], 'FontSize', 7);
        end
    end
end
xlabel('Segment index'); ylabel('到最近活跃 NFZ 的距离 (m)');
title('逐段 NFZ 接近度 (越低越危险)', 'FontSize', 10); grid on;
xlim([0 nSegs+1]);

sgtitle(sprintf('逐段惩罚与NFZ接近度  city=%s  t=%.0fs', DIAG_CITY, DIAG_T_START), ...
    'FontSize', 12, 'FontWeight', 'bold');
exportPublicationFigure(fig_pen2, sprintf('diag_pen2_%s_t%d.png', DIAG_CITY, DIAG_T_START));

%% ==================== 最终诊断结论 ====================

fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  诊断结论                                                    ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

J_pen_contribution = 100 * final_det.penalty_total;
J_non_pen          = final_det.J_final - J_pen_contribution;

fprintf('  J_final = %.3f\n', final_det.J_final);
fprintf('  └─ 正常代价 (E+T+C+R)  = %.3f  (占比 %.1f%%)\n', ...
    J_non_pen, 100*J_non_pen/final_det.J_final);
fprintf('  └─ 惩罚项 λ×Penalty   = %.3f  (占比 %.1f%%)\n', ...
    J_pen_contribution, 100*J_pen_contribution/final_det.J_final);

if J_pen_contribution > 0.5 * final_det.J_final
    fprintf('\n  ★ 主要异常来源: Penalty 项占比超过 50%%\n');
    [~, pen_order] = sort([ ...
        final_det.penalty_height, ...
        final_det.penalty_static_collision, ...
        final_det.penalty_dynamic_collision, ...
        final_det.penalty_battery], 'descend');
    pen_names = {'penalty_height', 'penalty_static', 'penalty_dyn', 'penalty_battery'};
    pen_vals  = [final_det.penalty_height, final_det.penalty_static_collision, ...
                 final_det.penalty_dynamic_collision, final_det.penalty_battery];
    fprintf('  最大 penalty 贡献项: %s = %.4f → J贡献 %.2f\n', ...
        pen_names{pen_order(1)}, pen_vals(pen_order(1)), 100*pen_vals(pen_order(1)));
end

% 阶段溯源
j_raw    = stages{1}.J_final;
j_smooth = stages{2}.J_final;
j_repair = stages{3}.J_final;
fprintf('\n  阶段溯源:\n');
if j_smooth - j_raw > 10
    fprintf('  → smoothPathSpline 引入: ΔJ=%+.2f  (可能平滑经过建筑)\n', j_smooth-j_raw);
end
if j_repair - j_smooth > 10
    fprintf('  → postSmoothRepair 引入: ΔJ=%+.2f  ← ★ 主要来源\n', j_repair-j_smooth);
    fprintf('     repair 后 heightViol 从 %d → %d\n', ...
        stages{2}.heightViolations, stages{3}.heightViolations);
    fprintf('     可能机制: 修复某障碍时把点推入建筑高度以下;\n');
    fprintf('               末尾平滑将 z 坐标进一步下拉\n');
end
if stage_det.internal_search_cost > j_repair * 1.5
    fprintf('\n  内部搜索代价 (%.2f) >> 最终 J (%.2f):\n', ...
        stage_det.internal_search_cost, j_repair);
    fprintf('  → 差值 %.2f = 搜索导向罚项总量\n', ...
        stage_det.internal_search_cost - j_repair);
    fprintf('     (smoothPenalty + nfzPenalty + obsPenalty + headwindPenalty)\n');
    fprintf('     搜索阶段对路径形状有较强导向; 最终 J 与搜索目标已解耦\n');
end

if n_anomalous > 0
    fprintf('\n  异常段 (%d 个):\n', n_anomalous);
    for k = anomaly_segs
        fprintf('    段 %3d: t=%.1fs  Pen=%.4f  (height=%.4f, static=%.4f, dyn=%.4f)\n', ...
            k, tArr(k), seg_diag(k).Pen_total, ...
            seg_diag(k).Pen_height, seg_diag(k).Pen_static, seg_diag(k).Pen_dyn);
        p1 = path(k,:); p2 = path(k+1,:);
        fprintf('         p%d=[%.1f,%.1f,%.1f] → p%d=[%.1f,%.1f,%.1f]\n', ...
            k, p1(1),p1(2),p1(3), k+1, p2(1),p2(2),p2(3));
    end
end

fprintf('\n图表已保存:\n');
fprintf('  diag_pie_%s_t%d.png\n',     DIAG_CITY, DIAG_T_START);
fprintf('  diag_segheat_%s_t%d.png\n', DIAG_CITY, DIAG_T_START);
fprintf('  diag_path_%s_t%d.png\n',    DIAG_CITY, DIAG_T_START);
fprintf('  diag_pen2_%s_t%d.png\n',    DIAG_CITY, DIAG_T_START);
fprintf('诊断完成.\n');

%% ====================================================================
%%  辅助函数 1: evaluatePath_segLevel
%%  对已有路径做一次带逐段详细诊断的评估
%%  不修改 UnifiedCostModel; 独立计算段级 penalty 分解 + NFZ/障碍接近度
%% ====================================================================
function [J_total, seg_diag] = evaluatePath_segLevel(costModel, env, pathPts, t_start, hasPayload)
% evaluatePath_segLevel — 逐段诊断评估
%
%  复用 UnifiedCostModel 的代价公式, 额外输出:
%    seg_diag(k).R_risk        段内风险积分
%    seg_diag(k).Pen_total     段总惩罚
%    seg_diag(k).Pen_height    段高度惩罚
%    seg_diag(k).Pen_static    段静态碰撞惩罚
%    seg_diag(k).Pen_dyn       段动态碰撞惩罚
%    seg_diag(k).nfz_proximity_flag  是否接近 NFZ
%    seg_diag(k).nfz_min_dist        到最近活跃 NFZ 的距离 (m)
%    seg_diag(k).obs_proximity_flag  是否接近动态障碍
%    seg_diag(k).obs_min_dist        到最近障碍的距离 (m)
%    seg_diag(k).height_below        低于地板多少米 (0=合规)
%    seg_diag(k).static_hit          是否穿越建筑

    if nargin < 5, hasPayload = true; end
    if nargin < 4, t_start = 0; end

    N = size(pathPts, 1);

    % 物理常量 (与 UnifiedCostModel 完全一致)
    if hasPayload
        m_total = costModel.m_frame + costModel.m_payload;
    else
        m_total = costModel.m_frame;
    end
    W         = m_total * 9.81;
    eta       = costModel.eta_motor * costModel.eta_prop;
    A_disc    = costModel.n_rotor * pi * costModel.r_prop^2;
    v_i_hover = sqrt(W / (2 * costModel.rho_air * A_disc));
    P_hover   = W^1.5 / sqrt(2*costModel.rho_air*A_disc) / eta;

    H_min   = costModel.H_min;
    H_max   = costModel.H_max;
    H_clear = costModel.H_clearance;
    MS      = env.MAP_SIZE;

    NFZ_MARGIN = 1.5;   % 接近度警戒倍数
    OBS_MARGIN = 2.5;

    nfzList = env.dynObstacles.tempNFZ;
    obsList = env.dynObstacles.movingObs;

    SUB_SPACING = 12;
    MIN_SUB     = 3;

    % 预分配
    seg_diag = struct(...
        'R_risk',              num2cell(zeros(N-1,1)), ...
        'Pen_total',           num2cell(zeros(N-1,1)), ...
        'Pen_height',          num2cell(zeros(N-1,1)), ...
        'Pen_static',          num2cell(zeros(N-1,1)), ...
        'Pen_dyn',             num2cell(zeros(N-1,1)), ...
        'nfz_proximity_flag',  num2cell(false(N-1,1)), ...
        'nfz_min_dist',        num2cell(inf(N-1,1)), ...
        'obs_proximity_flag',  num2cell(false(N-1,1)), ...
        'obs_min_dist',        num2cell(inf(N-1,1)), ...
        'height_below',        num2cell(zeros(N-1,1)), ...
        'static_hit',          num2cell(false(N-1,1)));
    seg_diag = seg_diag(:);

    t_current = t_start;
    E_total = 0; T_total = 0; Pen_total = 0; R_total = 0; C_total = 0;

    for k = 1:N-1
        p1 = pathPts(k, :);
        p2 = pathPts(k+1, :);

        dx = p2(1)-p1(1); dy = p2(2)-p1(2); dz = p2(3)-p1(3);
        d_horiz = sqrt(dx^2+dy^2);
        d_3d    = sqrt(dx^2+dy^2+dz^2);
        if d_3d < 0.01, continue; end

        if d_horiz > 0.01
            dir_h = [dx,dy]/d_horiz;
        else
            dir_h = [0,0];
        end
        gamma   = atan2(dz, d_horiz);
        v_horiz = costModel.v_cruise * cos(gamma);
        v_vert  = costModel.v_cruise * sin(gamma);

        nSub  = max(MIN_SUB, ceil(d_3d/SUB_SPACING));
        d_sub = d_3d / nSub;

        E_acc=0; T_acc=0; R_acc=0; Ph=0; Ps=0; Pd=0; P_acc=0;
        h_below_max = 0;
        static_hit  = false;
        nfz_min_d   = inf;
        obs_min_d   = inf;
        t_sub       = t_current;

        for s = 1:nSub
            frac_mid = (s-0.5)/nSub;
            pt_sub   = p1 + frac_mid*(p2-p1);

            % 风场
            wind_vec = [0,0,0];
            if ~isempty(costModel.windField)
                try
                    wind_vec = costModel.windField.getWind(pt_sub(1),pt_sub(2),pt_sub(3),t_sub);
                catch; end
            end
            v_wa = dot(wind_vec(1:2), dir_h);
            v_wc = norm(wind_vec(1:2) - v_wa*dir_h);
            v_wv = wind_vec(3);

            v_ah = v_horiz - v_wa;
            v_av = v_vert  - v_wv;
            v_air    = sqrt(v_ah^2 + v_wc^2 + v_av^2);  v_air = max(v_air, 0.5);
            v_ground = sqrt((v_horiz+v_wa)^2 + v_wc^2); v_ground = max(v_ground, 0.5);

            mu = max(v_ah,0)/(v_i_hover+0.01);
            if mu<0.1, v_i=v_i_hover*(1-mu^2/4);
            else,      v_i=v_i_hover^2/(2*max(abs(v_ah),1)); end
            P_ind  = W*v_i/eta;
            P_par  = 0.5*costModel.rho_air*costModel.C_d*costModel.A_body*v_air^3/eta;
            P_pro  = 0.15*P_hover;
            if v_av>0, P_cl=W*v_av/costModel.climb_efficiency/eta;
            else,      P_cl=W*v_av*costModel.descent_recovery; end
            if v_wc>0.5
                tilt=atan2(v_wc,v_i_hover*3); P_cr=P_hover*(1/cos(tilt)-1);
            else, P_cr=0; end
            P_tot = max(P_ind+P_par+P_pro+P_cl+P_cr, P_hover*0.3);

            dt_sub = d_sub/v_ground;
            E_sub  = P_tot*dt_sub/3600;

            % 动态风险 + 碰撞
            if ~isempty(costModel.dynObstacles)
                t_risk = t_sub + dt_sub/2;
                r_sub  = costModel.evaluateRiskAtPoint(pt_sub, t_risk);
                R_acc  = R_acc + r_sub*dt_sub/60;
                if costModel.checkCollision(pt_sub, t_risk)
                    pen = 1/nSub; P_acc=P_acc+pen; Pd=Pd+pen;
                end
            end

            % 静态碰撞
            if ~isempty(costModel.staticMap)
                rx=max(1,min(size(costModel.staticMap,1),round(pt_sub(1))));
                ry=max(1,min(size(costModel.staticMap,2),round(pt_sub(2))));
                if pt_sub(3) < costModel.staticMap(rx,ry)+3
                    pen=1/nSub; P_acc=P_acc+pen; Ps=Ps+pen;
                    static_hit=true;
                end
            end

            % 高度约束
            if ~isempty(costModel.staticMap)
                rx=max(1,min(size(costModel.staticMap,1),round(pt_sub(1))));
                ry=max(1,min(size(costModel.staticMap,2),round(pt_sub(2))));
                gh=costModel.staticMap(rx,ry);
            else, gh=0; end
            mf = max(H_min, gh+H_clear);
            if pt_sub(3)<mf
                pen=(mf-pt_sub(3))/10/nSub; P_acc=P_acc+pen; Ph=Ph+pen;
                h_below_max=max(h_below_max, mf-pt_sub(3));
            end
            if pt_sub(3)>H_max
                pen=(pt_sub(3)-H_max)/10/nSub; P_acc=P_acc+pen; Ph=Ph+pen;
            end

            % NFZ 接近度
            for ni=1:length(nfzList)
                nfz=nfzList(ni);
                if ~nfz.active||t_sub<nfz.t_start||t_sub>nfz.t_end,continue;end
                dh=norm(pt_sub(1:2)-nfz.center);
                nfz_min_d=min(nfz_min_d,dh);
            end

            % 障碍接近度
            for oi=1:length(obsList)
                op=env.dynObstacles.getPosition(oi, t_sub+dt_sub/2);
                d_obs=norm(pt_sub-op);
                obs_min_d=min(obs_min_d,d_obs);
            end

            E_acc=E_acc+E_sub; T_acc=T_acc+dt_sub; t_sub=t_sub+dt_sub;
        end

        % 端点高度检查 p1 (与 UnifiedCostModel 修复后一致)
        if ~isempty(costModel.staticMap)
            px=max(1,min(MS,round(p1(1)))); py=max(1,min(MS,round(p1(2))));
            gh=costModel.staticMap(px,py);
        else, gh=0; end
        mf=max(H_min,gh+H_clear);
        if p1(3)<mf
            pen=(mf-p1(3))/10; P_acc=P_acc+pen; Ph=Ph+pen;
            h_below_max=max(h_below_max,mf-p1(3));
        end
        if p1(3)>H_max
            pen=(p1(3)-H_max)/10; P_acc=P_acc+pen; Ph=Ph+pen;
        end

        C_total = C_total + abs(dz)*0.01;
        E_total = E_total + E_acc;
        T_total = T_total + T_acc;
        R_total = R_total + R_acc;
        Pen_total = Pen_total + P_acc;

        % 写入段诊断
        seg_diag(k).R_risk             = R_acc;
        seg_diag(k).Pen_total          = P_acc;
        seg_diag(k).Pen_height         = Ph;
        seg_diag(k).Pen_static         = Ps;
        seg_diag(k).Pen_dyn            = Pd;
        if isempty(nfzList)
            seg_diag(k).nfz_proximity_flag = false;
        else
            seg_diag(k).nfz_proximity_flag = (isfinite(nfz_min_d) && ...
                nfz_min_d < min([nfzList.radius])*NFZ_MARGIN);
        end
        seg_diag(k).nfz_min_dist       = nfz_min_d;
        if isempty(obsList)
            seg_diag(k).obs_proximity_flag = false;
        else
            seg_diag(k).obs_min_dist       = obs_min_d;
            seg_diag(k).obs_proximity_flag = (isfinite(obs_min_d) && ...
                obs_min_d < min([obsList.radius])*OBS_MARGIN);
        end
        seg_diag(k).height_below       = h_below_max;
        seg_diag(k).static_hit         = static_hit;

        t_current = t_sub;
    end

    % 末尾检查最后一点
    pt_last = pathPts(N,:);
    if ~isempty(costModel.staticMap)
        px=max(1,min(MS,round(pt_last(1)))); py=max(1,min(MS,round(pt_last(2))));
        gh=costModel.staticMap(px,py);
    else, gh=0; end
    mf=max(H_min,gh+H_clear);
    P_last=0;
    if pt_last(3)<mf, P_last=P_last+(mf-pt_last(3))/10; end
    if pt_last(3)>H_max, P_last=P_last+(pt_last(3)-H_max)/10; end
    Pen_total = Pen_total + P_last;

    % 电池惩罚
    pen_bat = 0;
    if E_total > costModel.E_batt*0.9
        pen_bat = (E_total - costModel.E_batt*0.9)/10;
        Pen_total = Pen_total + pen_bat;
    end

    J_total = costModel.w_energy*E_total + ...
              costModel.w_time*(T_total/60) + ...
              costModel.w_climb*C_total + ...
              costModel.w_risk*R_total + ...
              costModel.lambda_penalty*Pen_total;
end
