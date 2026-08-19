%% =========================================================================
%%  面向时变城市低空环境的风险感知能耗优化 ALA 三维路径规划方法
%%  (Risk-Aware Energy-Optimized ALA 3D Path Planning
%%   for Time-Varying Urban Low-Altitude Environments)
%% =========================================================================
%%
%%  【诊断版修改说明】
%%    目标1: 每个算法的结果输出均包含完整代价分解
%%           (J, E, T, Risk, penalty_total, penalty_height,
%%            penalty_static/dyn_collision, NFZ诊断等)
%%    目标2: 所有算法 (RA-ALA / Energy-A* / RRT* / Greedy) 的最终 J
%%           统一通过 costModel.evaluatePath(finalPath, t_start, true)
%%           重新计算, 不使用搜索阶段内部适应度
%%    目标3: RA-ALA 输出三阶段代价分解
%%           (raw path → smooth path → final path, Repair 模块已移除)
%%           以便定位 J 异常究竟发生在哪个阶段
%%
%%  依赖: CityEnvironment.m, UnifiedCostModel.m, PathPlanners.m (同目录)
%% =========================================================================

% Optional execution modes. The default remains the full experiment.
% Create or redraw the five-case departure-time archive:
%   RA_ALA_RUN_MODE = 'departure-time-only'; runMainExperiments
%   RA_ALA_RUN_MODE = 'replot-departure-time'; runMainExperiments
% Create or redraw the original representative path-comparison archive:
%   RA_ALA_RUN_MODE = 'path-comparison-only'; runMainExperiments
%   RA_ALA_RUN_MODE = 'replot-path-comparison'; runMainExperiments
if ~exist('RA_ALA_RUN_MODE','var') || isempty(RA_ALA_RUN_MODE)
    RA_ALA_RUN_MODE = 'full';
end
clearvars -except RA_ALA_RUN_MODE; clc; close all;
warning off;

%% ====================================================================
%%  实时显示控制开关（可根据需要开关，关掉可加速运行）
%% ====================================================================
SHOW_LIVE        = true;   % 总开关：是否开启实时显示
SHOW_ITER_BAR    = true;   % 每轮迭代进度条（命令行）
SHOW_CONV_CURVE  = true;   % 实时收敛曲线（figure窗口）
SHOW_PATH_LIVE   = true;   % 迭代中实时路径预览（figure窗口）
REFRESH_EVERY    = 5;      % 每隔多少次迭代刷新图形（太小会慢）
PRINT_ITER_EVERY = 10;     % 每隔多少次迭代打印一行进度（0=不打印）
RUN_WEIGHT_SENSITIVITY = true; % 审稿补充：第5.5节同队列代价权重敏感性实验
RUN_SPATIAL_RESOLUTION_SENSITIVITY = false; % 设为 true 可在5.5后运行12/6/3/1.5 m分辨率实验

fprintf('╔═══════════════════════════════════════════════════════════════╗\n');
fprintf('║  面向时变城市低空环境的风险感知能耗优化 ALA 路径规划         ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════╝\n\n');

%% ==================== 全局参数 ====================

seed      = 42;
mapSize   = 1000;
gridStep  = 10;
windLevel = 'medium';
riskLevel = 'dense';

startPt   = [80,  80,  60];
goalPt    = [900, 900, 60];

departureTimes = [0, 60, 120, 180, 240];
nDepart = length(departureTimes);
departLabels = arrayfun(@(t) sprintf('t=%ds', t), departureTimes, 'UniformOutput', false);
departColors = [
    0.15, 0.30, 0.75;
    0.20, 0.60, 0.65;
    0.30, 0.75, 0.35;
    0.85, 0.60, 0.15;
    0.85, 0.15, 0.15;
];

cityLevels = {'low', 'medium', 'high'};
cityLabels = {'Low (15 bldg)', 'Medium (35 bldg)', 'High (60 bldg)'};
nCity = length(cityLevels);

algNames  = {'RA-ALA', 'Energy-A*', 'Informed-RRT*', 'ST-EA*', 'Greedy'};
nAlg = length(algNames);
algColors = [
    0.85, 0.10, 0.10;
    0.00, 0.45, 0.74;
    0.47, 0.67, 0.19;
    0.49, 0.18, 0.56;
    0.70, 0.70, 0.70;
];
algStyles = {'-', '--', '-.', '-', ':'};
algWidths = [3.2, 2.6, 2.6, 2.6, 2.2];
st_time_step = 2;
st_time_horizon = 300;

ala_cfg.popSize       = 30;
ala_cfg.maxIter       = 60;
ala_cfg.nWaypoints    = 8;
ala_cfg.riskWeight    = 15.0;
ala_cfg.windLookahead = 3;

PATH_COMPARISON_ONLY = strcmpi(string(RA_ALA_RUN_MODE),'path-comparison-only');
DEPARTURE_TIME_ONLY = strcmpi(string(RA_ALA_RUN_MODE),'departure-time-only');
if strcmpi(string(RA_ALA_RUN_MODE),'replot-departure-time')
    archiveFile=fullfile(fileparts(mfilename('fullpath')),'departure_time_case_data.mat');
    if ~isfile(archiveFile)
        error('RAALA:MissingDepartureArchive', ...
            ['Cannot redraw the departure-time case study because %s is missing. ', ...
             'Run RA_ALA_RUN_MODE = ''departure-time-only'' once to create it.'],archiveFile);
    end
    archived=load(archiveFile);
    rng(archived.seed);
    env=CityEnvironment(archived.mapSize,archived.gridStep);
    env.generate('high',archived.windLevel,archived.riskLevel,archived.seed);
    env.setTaskPoints(archived.startPt,archived.goalPt);
    fig1=renderDepartureTimeAdaptation(env,archived.exp1_paths, ...
        archived.departureTimes,archived.departColors,archived.startPt, ...
        archived.goalPt,archived.mapSize);
    outputBase=fullfile(fileparts(mfilename('fullpath')),'fig1_temporal_adaptation.png');
    exportPublicationFigure(fig1,outputBase,600);
    savefig(fig1,replace(outputBase,'.png','.fig'));
    fprintf(['Departure-time figure redrawn from its dedicated archive ', ...
        '(environment seed %d). No planner was executed.\n'],archived.seed);
    return;
end
if strcmpi(string(RA_ALA_RUN_MODE),'replot-path-comparison')
    archiveFile=fullfile(fileparts(mfilename('fullpath')),'representative_path_case_data.mat');
    if ~isfile(archiveFile)
        error('RAALA:MissingArchive', ...
            ['Cannot redraw the original representative case because %s is missing. ', ...
             'Run RA_ALA_RUN_MODE = ''path-comparison-only'' once to create it.'],archiveFile);
    end
    archived=load(archiveFile);
    rng(archived.seed);
    env=CityEnvironment(archived.mapSize,archived.gridStep);
    env.generate('high',archived.windLevel,archived.riskLevel,archived.seed);
    env.setTaskPoints(archived.startPt,archived.goalPt);
    fig3=renderPathQualityComparison(env,archived.exp2_paths, ...
        archived.algNames,algColors,algStyles,algWidths, ...
        archived.startPt,archived.goalPt,archived.mapSize);
    outputBase=fullfile(fileparts(mfilename('fullpath')),'representative_path_comparison.png');
    exportPublicationFigure(fig3,outputBase,600);
    savefig(fig3,replace(outputBase,'.png','.fig'));
    fprintf(['Original representative path figure redrawn from its dedicated ', ...
        'path archive (environment seed %d). No planner was executed.\n'],archived.seed);
    return;
end
if ~PATH_COMPARISON_ONLY && ~DEPARTURE_TIME_ONLY && ~strcmpi(string(RA_ALA_RUN_MODE),'full')
    error('RAALA:UnknownRunMode','Unknown RA_ALA_RUN_MODE: %s',string(RA_ALA_RUN_MODE));
end

%% ====================================================================
%%  实验1: 同一任务、不同出发时刻 → RA-ALA 路径差异
%% ====================================================================

if ~PATH_COMPARISON_ONLY
fprintf('━━━ 实验1: 不同出发时刻下的 RA-ALA 路径差异 ━━━\n');
end

rng(seed);
env = CityEnvironment(mapSize, gridStep);
env.generate('high', windLevel, riskLevel, seed);
env.setTaskPoints(startPt, goalPt);

costModel = UnifiedCostModel();
costModel.setEnvironment(env.windField, env.dynObstacles, env.heightMap);
planner = PathPlanners(env, costModel);
planner.setBudget(15, 5000, 2000);

% ★ 目标3: 新增 stage_details 存储三阶段分解
exp1_paths         = cell(nDepart, 1);
exp1_costs         = zeros(nDepart, 1);
exp1_details       = cell(nDepart, 1);
exp1_stage_details = cell(nDepart, 1);

if ~PATH_COMPARISON_ONLY
for d = 1:nDepart
    t_dep = departureTimes(d);
    if exist('SHOW_LIVE','var') && SHOW_LIVE
        fprintf('\n');
        fprintf('  ┌─────────────────────────────────────────────────────────┐\n');
        fprintf('  │  出发时刻 t=%3ds  (%d/%d)  请稍候...                     │\n', ...
            t_dep, d, nDepart);
        fprintf('  └─────────────────────────────────────────────────────────┘\n');
    else
        fprintf('  出发时刻 t=%3ds:\n', t_dep);
    end

    rng(seed + 100);
    % ★ runRA_ALA 第4输出为 stage_details (raw/smooth 两阶段，Repair 已移除)
    [exp1_paths{d}, exp1_costs(d), exp1_details{d}, exp1_stage_details{d}] = ...
        runRA_ALA(planner, costModel, env, startPt, goalPt, t_dep, true, ala_cfg);

    % ★ 目标1: 完整代价分解
    printCostDecomposition(exp1_details{d}, sprintf('RA-ALA t=%ds [final]', t_dep));

    % ★ 目标3: 两阶段对比（Repair 已从算法移除，仅保留 raw / smooth）
    sd = exp1_stage_details{d};
    fprintf('    ┌─── 阶段分解 ─────────────────────────────────────────────┐\n');
    fprintf('    │  [搜索] internal_search_J  = %8.2f  (ALA内部适应度)\n', sd.internal_search_cost);
    fprintf('    │         含: smooth/NFZ/headwind 导向罚项; 不参与最终比较\n');
    fprintf('    │  [路径] raw    final_J = %8.2f  Pen=%.4f  hViol=%d\n', ...
        sd.raw.J_final, sd.raw.penalty_total, sd.raw.heightViolations);
    fprintf('    │         smooth final_J = %8.2f  Pen=%.4f  hViol=%d\n', ...
        sd.smooth.J_final, sd.smooth.penalty_total, sd.smooth.heightViolations);
    fprintf('    │  ★ 最终选择: [%s]  (Top-K 在 raw 与 smooth 间择优)\n', ...
        sd.chosen_path_type);
    fprintf('    │  Δ(internal→final): %+.2f\n', ...
        exp1_details{d}.J_final - sd.internal_search_cost);
    fprintf('    └──────────────────────────────────────────────────────────┘\n\n');
end

% 审稿补充：导出五个出发时刻的分阶段耗时、Top-K 候选数量、
% RescueA 冲突段/候选计数及运行平台信息。
% 若正文定义了重规划周期，可将第4参数 NaN 改为该周期（秒）。
exp1_latency_table = exportDepartureTimeLatency( ...
    departureTimes, exp1_details, pwd, NaN);
end

if DEPARTURE_TIME_ONLY
    case_metadata=struct( ...
        'description','Five-case illustrative departure-time study', ...
        'environment_seed',seed, ...
        'departure_times_s',departureTimes, ...
        'selection_rule','Predefined departure times in one fixed High-complexity environment');
    archiveFile=fullfile(fileparts(mfilename('fullpath')),'departure_time_case_data.mat');
    save(archiveFile,'exp1_paths','exp1_costs','exp1_details','exp1_stage_details', ...
        'case_metadata','departureTimes','departColors','seed','mapSize','gridStep', ...
        'windLevel','riskLevel','startPt','goalPt','ala_cfg','-v7');
    fig1=renderDepartureTimeAdaptation(env,exp1_paths,departureTimes, ...
        departColors,startPt,goalPt,mapSize);
    outputBase=fullfile(fileparts(mfilename('fullpath')),'fig1_temporal_adaptation.png');
    exportPublicationFigure(fig1,outputBase,600);
    savefig(fig1,replace(outputBase,'.png','.fig'));
    fprintf(['Departure-time case-study archive and figure exported. ', ...
        'No subsequent experiment was executed.\n']);
    return;
end

%% ====================================================================
%%  实验2: 固定出发时刻 t=0 → RA-ALA vs 三种基线
%% ====================================================================

fprintf('\n━━━ 实验2: RA-ALA vs 基线算法 (t=0, high城市) ━━━\n');
fprintf('  ★ 统一评估口径: 所有算法均通过 costModel.evaluatePath(finalPath,0,true)\n');
fprintf('    重新计算最终 J; 不使用各算法搜索阶段的内部适应度\n\n');

exp2_paths         = cell(nAlg, 1);
exp2_costs         = zeros(nAlg, 1);
exp2_details       = cell(nAlg, 1);
exp2_stage_details = cell(nAlg, 1);

for a = 1:nAlg
    fprintf('  %-18s :\n', algNames{a});
    rng(seed + a);

    switch a
        case 1  % RA-ALA: 运行优化器取 finalPath; 搜索适应度不作为最终 J
            [exp2_paths{a}, ~, ~, exp2_stage_details{a}] = ...
                runRA_ALA(planner, costModel, env, startPt, goalPt, 0, true, ala_cfg);

        case 2  % Energy-A*: 只取路径
            [exp2_paths{a}, ~, ~] = planner.energyAStar(startPt, goalPt, 0, true);

        case 3  % Informed-RRT*: 只取路径
            [exp2_paths{a}, ~, ~] = planner.informedRRTStar(startPt, goalPt, 0, true, 2000);

        case 4  % ST-EA*: 时空状态与到达时刻递推
            [exp2_paths{a}, ~, ~] = planner.timeExpandedEnergyAStar( ...
                startPt, goalPt, 0, true, st_time_step, st_time_horizon);

        case 5  % Greedy: 只取路径
            [exp2_paths{a}, ~, ~] = planner.greedyPlanner(startPt, goalPt, 0, true);
    end

    % ★ 目标2: 统一评估口径 —— 所有算法均用同一个 evaluatePath
    [exp2_costs(a), exp2_details{a}] = costModel.evaluatePath(exp2_paths{a}, 0, true);

    % ★ 目标1: 完整代价分解
    printCostDecomposition(exp2_details{a}, algNames{a});

    % ★ 目标3: RA-ALA 额外输出两阶段对比（Repair 已移除）
    if a == 1
        sd = exp2_stage_details{a};
        fprintf('    ┌─── RA-ALA 阶段分解 (t=0) ───────────────────────────────────┐\n');
        fprintf('    │  [搜索] internal_search_J  = %7.2f  (evaluateRAALASearchFitness 内部适应度)\n', ...
            sd.internal_search_cost);
        fprintf('    │         = final_J + smooth罚 + NFZ罚 + headwind罚 + ...\n');
        fprintf('    │         差值 = internal - final_J = %+.2f\n', ...
            sd.internal_search_cost - exp2_details{a}.J_final);
        fprintf('    │  [路径] raw    final_J=%7.2f  (h=%.4f, s=%.4f, d=%.4f)\n', ...
            sd.raw.J_final, sd.raw.penalty_height, sd.raw.penalty_static_collision, ...
            sd.raw.penalty_dynamic_collision);
        fprintf('    │         smooth final_J=%7.2f  (h=%.4f, s=%.4f, d=%.4f)\n', ...
            sd.smooth.J_final, sd.smooth.penalty_height, sd.smooth.penalty_static_collision, ...
            sd.smooth.penalty_dynamic_collision);
        fprintf('    │  ★ Top-K 最终选择: [%s]\n', sd.chosen_path_type);
        fprintf('    │  NFZ硬罚(已入J): raw=%.1f  smooth=%.1f\n', ...
            sd.raw.NFZ_penalty, sd.smooth.NFZ_penalty);
        fprintf('    └────────────────────────────────────────────────────────────┘\n\n');
    end
end

% ★ 汇总对比表 (final_unified_cost 口径)
%   所有 J 值 = costModel.evaluatePath(finalPath) 的结果
%   RA-ALA 额外显示 internal_search_cost 供透明对比
fprintf('  ╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('  ║  算法对比汇总  ★ 所有 J = final_unified_cost (evaluatePath 口径)     ║\n');
fprintf('  ║  t=0 / high 城市 / RA-ALA 额外显示 internal_search_cost               ║\n');
fprintf('  ╠═══════════════╦═══════════╦══════════╦═══════╦═══════╦════════╦════════╣\n');
fprintf('  ║ %-13s ║ %9s ║ %8s ║ %5s ║ %5s ║ %6s ║ %6s ║\n', ...
    '算法', 'final_J', 'intern_J', 'E(Wh)', 'T(s)', 'Risk', 'Pen');
fprintf('  ╠═══════════════╬═══════════╬══════════╬═══════╬═══════╬════════╬════════╣\n');
for a = 1:nAlg
    det = exp2_details{a};
    mk  = ' '; if a==1, mk='★'; end
    if a == 1 && isfield(det, 'internal_search_cost')
        intern_str = sprintf('%8.1f', det.internal_search_cost);
    else
        intern_str = sprintf('%8s', 'N/A');   % 基线无内部适应度概念
    end
    fprintf('  ║%s%-12s ║ %9.3f ║ %s ║ %5.1f ║ %5.0f ║ %6.4f ║ %6.4f ║\n', ...
        mk, algNames{a}, det.J_final, intern_str, ...
        det.E_total, det.T_total, det.R_dynamic, det.penalty_total);
end
fprintf('  ╠═══════════════╩═══════════╩══════════╩═══════╩═══════╩════════╩════════╣\n');
fprintf('  ║ final_J  = costModel.evaluatePath(finalPath)  → 算法间比较的唯一依据 ║\n');
fprintf('  ║ intern_J = evaluateRAALASearchFitness 内部适应度 (含导向罚项)  → 仅供透明度诊断       ║\n');
fprintf('  ╚══════════════════════════════════════════════════════════════════════════╝\n\n');

if PATH_COMPARISON_ONLY
    case_metadata = struct( ...
        'description','Original representative High-complexity path-comparison case', ...
        'environment_seed',seed, ...
        'departure_time_s',0, ...
        'selection_rule','Predefined manuscript figure case; no performance-based screening');
    archiveFile=fullfile(fileparts(mfilename('fullpath')),'representative_path_case_data.mat');
    save(archiveFile,'exp2_paths','exp2_costs','exp2_details','exp2_stage_details', ...
        'case_metadata','seed','mapSize','gridStep','windLevel','riskLevel', ...
        'startPt','goalPt','algNames','algColors','algStyles','algWidths', ...
        'st_time_step','st_time_horizon','ala_cfg','-v7');
    fig3=renderPathQualityComparison(env,exp2_paths,algNames,algColors, ...
        algStyles,algWidths,startPt,goalPt,mapSize);
    outputBase=fullfile(fileparts(mfilename('fullpath')),'representative_path_comparison.png');
    exportPublicationFigure(fig3,outputBase,600);
    savefig(fig3,replace(outputBase,'.png','.fig'));
    fprintf(['Original representative path case exported with seed %d. ', ...
        'No other experiment was executed.\n'],seed);
    return;
end


%% ====================================================================
%%  实验3: 三种城市复杂度 × RA-ALA + 基线
%% ====================================================================

fprintf('\n━━━ 实验3: 三种城市复杂度 × 四种算法 ━━━\n');

exp3_costs   = zeros(nCity, nAlg);
exp3_details = cell(nCity, nAlg);
exp3_paths   = cell(nCity, nAlg);
exp3_envs    = cell(nCity, 1);

for c = 1:nCity
    fprintf('  [%s]\n', cityLabels{c});
    rng(seed);
    env_c = CityEnvironment(mapSize, gridStep);
    env_c.generate(cityLevels{c}, windLevel, riskLevel, seed);
    env_c.setTaskPoints(startPt, goalPt);
    exp3_envs{c} = env_c;

    cm_c = UnifiedCostModel();
    cm_c.setEnvironment(env_c.windField, env_c.dynObstacles, env_c.heightMap);
    pl_c = PathPlanners(env_c, cm_c);
    pl_c.setBudget(15, 5000, 2000);

    for a = 1:nAlg
        rng(seed + a);
        switch a
            case 1
                [exp3_paths{c,a}, ~, ~, ~] = ...
                    runRA_ALA(pl_c, cm_c, env_c, startPt, goalPt, 0, true, ala_cfg);
            case 2
                [exp3_paths{c,a}, ~, ~] = pl_c.energyAStar(startPt, goalPt, 0, true);
            case 3
                [exp3_paths{c,a}, ~, ~] = pl_c.informedRRTStar(startPt, goalPt, 0, true, 2000);
            case 4
                [exp3_paths{c,a}, ~, ~] = pl_c.timeExpandedEnergyAStar( ...
                    startPt, goalPt, 0, true, st_time_step, st_time_horizon);
            case 5
                [exp3_paths{c,a}, ~, ~] = pl_c.greedyPlanner(startPt, goalPt, 0, true);
        end
        % ★ 统一评估口径
        [exp3_costs(c,a), exp3_details{c,a}] = cm_c.evaluatePath(exp3_paths{c,a}, 0, true);
        det = exp3_details{c,a};
        fprintf('    %-16s J=%6.1f  E=%5.1f  T=%5.0f  R=%6.4f  Pen=%.4f\n', ...
            algNames{a}, exp3_costs(c,a), det.E_total, det.T_total, ...
            det.R_dynamic, det.penalty_total);
    end
end

%% ====================================================================
%%  图1: 不同出发时刻路径对比
%% ====================================================================

fprintf('\n生成图表...\n');

buildings = env.buildings;
movObs    = env.dynObstacles.movingObs;
tempNFZ   = env.dynObstacles.tempNFZ;


fig1=renderDepartureTimeAdaptation(env,exp1_paths,departureTimes, ...
    departColors,startPt,goalPt,mapSize);
exportPublicationFigure(fig1,'fig1_temporal_adaptation.png',600);
savefig(fig1,'fig1_temporal_adaptation.fig');
fprintf('  fig1 (Temporal Adaptation) saved\n');

%% ====================================================================
%%  图2: 风场快照 + 路径
%% ====================================================================

fig2=renderSpatiotemporalDynamics(env,exp1_paths,departureTimes, ...
    departColors,startPt,goalPt,mapSize);
exportPublicationFigure(fig2,'fig2_spatiotemporal_dynamics.png',600);
savefig(fig2,'fig2_spatiotemporal_dynamics.fig');
fprintf('  fig2 (Spatio-Temporal Dynamics) saved\n');

%% ====================================================================
%%  图3: RA-ALA vs 基线
%% ====================================================================

fig3=renderPathQualityComparison(env,exp2_paths,algNames,algColors, ...
    algStyles,algWidths,startPt,goalPt,mapSize);
exportPublicationFigure(fig3,'representative_path_comparison.png',600);
savefig(fig3,'representative_path_comparison.fig');
fprintf('  fig3 (Path Quality Comparison) saved\n');

%% ====================================================================
%%  图4: 性能柱状图
%% ====================================================================

fig4 = figure('Units','centimeters','Position',[0.5 0.5 32 18],'Color','w');
metricTitles = {'Unified Cost J', 'Energy (Wh)', 'Flight Time (s)', 'Dynamic Risk'};
% 标记哪些子图需要对数 Y 轴（数据跨度大的）
metricUseLog = [true, false, false, true];

metricData = zeros(nCity, nAlg, 4);
for c = 1:nCity
    for a = 1:nAlg
        d = exp3_details{c,a};
        metricData(c,a,:) = [exp3_costs(c,a), d.E_total, d.T_total, d.R_dynamic];
    end
end
for m = 1:4
    subplot(2,2,m); hold on; set(gca,'FontSize',14,'FontName','Times New Roman');
    barData = squeeze(metricData(:,:,m));
    hb = bar(barData, 'grouped');
    for a = 1:nAlg, hb(a).FaceColor = algColors(a,:); end
    set(gca,'XTick',1:nCity,'XTickLabel',{'Low','Medium','High'});
    ylabel(metricTitles{m},'FontSize',15);
    title(metricTitles{m},'FontSize',15,'FontWeight','bold');
    grid on;
    if m == 1, legend(algNames,'Location','northwest','FontSize',12); end

    % 对数轴：仅对 J 和 Risk 启用（数据跨多个数量级）
    use_log = metricUseLog(m);
    valid_vals = barData(barData > 0);
    if use_log && ~isempty(valid_vals)
        set(gca, 'YScale', 'log');
        % 下界 = 最小正值的 0.3 倍（给短柱留空间），上界 = 最大值的 5 倍（给文字留空间）
        ymin_log = max(min(valid_vals) * 0.3, 1e-5);
        ymax_log = max(valid_vals) * 5;
        ylim([ymin_log, ymax_log]);
    end

    nG=nCity; nB=nAlg; gW=min(0.8,nB/(nB+1.5));
    for a=1:nB
        xP=(1:nG)-gW/2+(2*a-1)*gW/(2*nB);
        for g=1:nG
            v=metricData(g,a,m);
            % 数值标签格式：根据量级自适应小数位数
            if abs(v) < 0.01 && v ~= 0
                vstr = sprintf('%.4f', v);
            elseif abs(v) < 1
                vstr = sprintf('%.3f', v);
            elseif abs(v) < 100
                vstr = sprintf('%.1f', v);
            else
                vstr = sprintf('%.0f', v);
            end
            % 文字位置：对数轴用乘法偏移，线性轴用加法偏移
            if use_log
                if v > 0
                    y_label = v * 1.4;   % 柱顶上方 0.15 dex
                else
                    continue;             % 跳过 0 值（对数轴不可见）
                end
            else
                y_label = v;
            end
            text(xP(g),y_label,vstr,'HorizontalAlignment','center',...
                'VerticalAlignment','bottom','FontSize',11,'FontWeight','bold',...
                'Color',algColors(a,:)*0.6);
        end
    end
end
sgtitle('Multi-Dimensional Performance Evaluation across Urban Complexity Levels', 'FontSize', 17, 'FontWeight','bold');
exportPublicationFigure(fig4, 'fig4_multidimensional_evaluation.png');
fprintf('  fig4 (Multi-Dimensional Evaluation) saved\n');

%% ====================================================================
%%  图5: 出发时刻敏感性
%% ====================================================================

metricByTime = zeros(nDepart, 4);
for d = 1:nDepart
    det = exp1_details{d};
    metricByTime(d,:) = [exp1_costs(d), det.E_total, det.T_total, det.R_dynamic];
end
fig5 = renderDepartureTimeSensitivity(metricByTime, departureTimes);
exportPublicationFigure(fig5, 'fig5_departure_time_sensitivity.png', 600);
savefig(fig5, 'fig5_departure_time_sensitivity.fig');
fprintf('  fig5 (Departure Time Sensitivity) saved\n');

%% ====================================================================
%%  图6: 动态障碍轨迹 + 时变路径
%% ====================================================================

fig6 = figure('Units','centimeters','Position',[0.5 0.5 26 24],'Color','w');
hold on; axis equal; set(gca, 'FontSize', 13, 'FontName', 'Times New Roman');
for i = 1:size(buildings,1)
    cx=buildings(i,1); cy=buildings(i,2); hw=buildings(i,4); hh=buildings(i,5);
    rectangle('Position',[cx-hw,cy-hh,2*hw,2*hh],'FaceColor',[0.9 0.9 0.9],'EdgeColor',[0.65 0.65 0.65],'LineWidth',0.3);
end
for i = 1:length(tempNFZ)
    nfz = tempNFZ(i); theta = linspace(0,2*pi,60);
    fill(nfz.center(1)+nfz.radius*cos(theta), nfz.center(2)+nfz.radius*sin(theta), ...
        [1 0.88 0.88], 'FaceAlpha',0.6, 'EdgeColor','r', 'LineWidth',1.3, 'LineStyle','--');
    text(nfz.center(1), nfz.center(2), sprintf('NFZ%d\nt=[%.0f,%.0f]s', i, nfz.t_start, nfz.t_end), ...
        'HorizontalAlignment','center','FontSize',9,'Color',[0.65 0 0],'FontWeight','bold');
end
obsC = lines(length(movObs));
tSpan = linspace(0, 300, 150);
for i = 1:length(movObs)
    traj = zeros(length(tSpan),2);
    for ti = 1:length(tSpan)
        p = env.dynObstacles.getPosition(i, tSpan(ti)); traj(ti,:) = p(1:2);
    end
    for ti = 2:length(tSpan)
        frac = ti/length(tSpan);
        plot(traj(ti-1:ti,1), traj(ti-1:ti,2), '-', 'Color', [obsC(i,:) 0.12+0.55*frac], 'LineWidth', 1);
    end
    pos0 = env.dynObstacles.getPosition(i, 0); theta = linspace(0,2*pi,24);
    fill(pos0(1)+movObs(i).radius*cos(theta), pos0(2)+movObs(i).radius*sin(theta), ...
        obsC(i,:), 'FaceAlpha',0.65, 'EdgeColor',obsC(i,:)*0.6, 'LineWidth',0.7);
    text(pos0(1), pos0(2)+movObs(i).radius+12, sprintf('Obs%d',i), ...
        'HorizontalAlignment','center','FontSize',9,'Color',obsC(i,:)*0.5);
end
p0=exp1_paths{1}; p180=exp1_paths{4};
if ~isempty(p0),   plot(p0(:,1),   p0(:,2),   '-', 'Color', departColors(1,:), 'LineWidth', 2.8); end
if ~isempty(p180), plot(p180(:,1), p180(:,2), '-', 'Color', departColors(4,:), 'LineWidth', 2.8); end
plot(startPt(1),startPt(2),'p','MarkerSize',16,'MarkerFaceColor',[0 0.8 0],'MarkerEdgeColor','k');
plot(goalPt(1),goalPt(2),'h','MarkerSize',16,'MarkerFaceColor',[1 0 0],'MarkerEdgeColor','k');
h_obs=fill(NaN,NaN,[1 0.6 0.2],'FaceAlpha',0.65,'EdgeColor',[0.85 0.45 0]);
h_nfz=fill(NaN,NaN,[1 0.88 0.88],'FaceAlpha',0.6,'EdgeColor','r','LineStyle','--');
h_p0=plot(NaN,NaN,'-','Color',departColors(1,:),'LineWidth',2.8);
h_p180=plot(NaN,NaN,'-','Color',departColors(4,:),'LineWidth',2.8);
legend([h_obs,h_nfz,h_p0,h_p180], {'Obstacle (t=0)','No-Fly Zone','RA-ALA (t=0s)','RA-ALA (t=180s)'}, ...
    'Location','northwest','FontSize',12);
xlabel('X (m)'); ylabel('Y (m)');
title('Spatio-Temporal Trajectories of Dynamic Obstacles and Time-Dependent Path Adaptation', 'FontSize', 16, 'FontWeight','bold');
xlim([0 mapSize]); ylim([0 mapSize]); grid on; box on;
exportPublicationFigure(fig6, 'fig6_spatiotemporal_trajectories.png');
fprintf('  fig6 (Spatio-Temporal Trajectories) saved\n');

%% ====================================================================
%%  完整汇总表
%% ====================================================================

fprintf('\n╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  实验1 汇总: 不同出发时刻 RA-ALA 代价分解                          ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════════╝\n');
fprintf('%-8s %7s %7s %6s %7s %8s %8s %7s\n', ...
    '时刻', 'J', 'E(Wh)', 'T(s)', 'Risk', 'Pen_tot', 'Pen_h', 'hViol');
fprintf('%s\n', repmat('-',1,72));
for d = 1:nDepart
    det = exp1_details{d};
    fprintf('%-8s %7.1f %7.1f %6.0f %7.4f %8.4f %8.4f %7d\n', ...
        departLabels{d}, det.J_final, det.E_total, det.T_total, det.R_dynamic, ...
        det.penalty_total, det.penalty_height, det.heightViolations);
end

fprintf('\n╔══════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  实验2 汇总: RA-ALA vs 基线 (统一评估口径, t=0, high城市)          ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════════╝\n');
fprintf('%-18s %7s %7s %6s %7s %8s %8s\n', ...
    '算法', 'J', 'E(Wh)', 'T(s)', 'Risk', 'Pen_tot', 'feasible');
fprintf('%s\n', repmat('-',1,72));
for a = 1:nAlg
    det = exp2_details{a};
    mk = '  '; if a == 1, mk = '★ '; end
    fprintf('%s%-16s %7.1f %7.1f %6.0f %7.4f %8.4f %8d\n', ...
        mk, algNames{a}, det.J_final, det.E_total, det.T_total, ...
        det.R_dynamic, det.penalty_total, det.feasible);
end

%% ====================================================================
%%  图7: 多随机种子箱线图 (fig7)
%%  图8: 环境级聚类感知配对统计图 (fig8)
%%
%%  两因素分层采样设计:
%%    外层 N_ENV 个不同城市环境 → 测试跨场景泛化性
%%    内层 N_SEED 个 ALA 内部种子 → 测试同场景算法稳定性
%%    环境队列: 固定复用旧版 3 m 实验中的 10 个预先指定环境
%%    提高容量: stat 轮次使用更大 popSize/maxIter
%%    总运行次数 = N_ENV × N_SEED = N_STAT
%%
%%    fig7 — 箱线图：环境内三次运行中位数的跨环境分布
%%    fig8 — 环境级精确符号秩检验、Holm 校正与配对秩二列效应量
%% ====================================================================

fprintf('\n━━━ Generating Statistical Figures (fig7: Distributional Robustness / fig8: Statistical Significance) ━━━\n');
fprintf('  设计: 两因素分层采样 (N_ENV 个环境 × N_SEED 次ALA内部种子)\n');

% ── 参数配置 ──
N_ENV  = 10;  % 环境数量（升至10以提升 vs 强基线对比的统计功效）
N_SEED = 3;   % 每个环境的 ALA 内部重复次数
N_STAT = N_ENV * N_SEED;   % 总样本量 = 30

% 统计轮次提高 ALA 容量（比主实验更强，确保应对更难的环境）
ala_cfg_stat                 = ala_cfg;
ala_cfg_stat.popSize         = 40;   % 主实验 30 → 统计轮次 40
ala_cfg_stat.maxIter         = 80;   % 主实验 60 → 统计轮次 80
ala_cfg_stat.rescue_max_ins  = 12;   % 主实验 6  → 统计轮次 12（增强救援）

stat_J    = zeros(nAlg, N_STAT);
stat_E    = zeros(nAlg, N_STAT);
stat_P    = zeros(nAlg, N_STAT);
stat_feasible = false(nAlg, N_STAT); % 严格标准：无任一硬约束违规
stat_env  = zeros(1, N_STAT);   % 记录每次使用的环境 seed（用于论文方法说明）
stat_ra_seed = zeros(1, N_STAT); % Exact RA-ALA run seeds reused by the ablation study
stat_paths = cell(nAlg,N_STAT); % Save selected paths for fixed-path resolution auditing

env_seeds_used = [483, 638, 855, 948, 1041, 1103, 1227, 1475, 2312, 2560];
if numel(env_seeds_used) ~= N_ENV
    error('The fixed Section 5.5 cohort must contain exactly %d environments.', N_ENV);
end
col_idx = 0;
fprintf('  固定环境 seeds: %s\n', mat2str(env_seeds_used));

for env_count = 1:N_ENV
    env_candidate = env_seeds_used(env_count);
    rng(env_candidate);
    env_c2 = CityEnvironment(mapSize, gridStep);
    env_c2.generate('high', windLevel, riskLevel, env_candidate);
    env_c2.setTaskPoints(startPt, goalPt);
    cm_c2 = UnifiedCostModel();
    cm_c2.setEnvironment(env_c2.windField, env_c2.dynObstacles, env_c2.heightMap);
    pl_c2 = PathPlanners(env_c2, cm_c2);
    pl_c2.setBudget(15, 5000, 2000);
    fprintf('  [环境 %d/%d] seed=%-5d\n', env_count, N_ENV, env_candidate);
    % ── 内层: N_SEED 次 ALA 内部种子 ──
    for s = 1:N_SEED
        col_idx = col_idx + 1;
        alg_seed = env_candidate + s * 53;   % 内部种子与环境 seed 解耦
        stat_env(col_idx) = env_candidate;
        stat_ra_seed(col_idx) = alg_seed + 11; % a=1: actual RA-ALA rng seed

        for a = 1:nAlg
            rng(alg_seed + a * 11);
            try
                switch a
                    case 1  % RA-ALA: 不同环境 + 不同内部种子
                        [p_s,~,d_s] = runRA_ALA(pl_c2, cm_c2, env_c2, ...
                            startPt, goalPt, 0, true, ala_cfg_stat);
                    case 2  % Energy-A*: 确定性，同一环境内结果相同
                        [p_s,~,~] = pl_c2.energyAStar(startPt, goalPt, 0, true);
                        [~,d_s]   = cm_c2.evaluatePath(p_s, 0, true);
                    case 3  % Informed-RRT*: 不同内部采样种子
                        [p_s,~,~] = pl_c2.informedRRTStar(startPt, goalPt, 0, true, 1500);
                        [~,d_s]   = cm_c2.evaluatePath(p_s, 0, true);
                    case 4  % ST-EA*: 确定性时空图搜索
                        [p_s,~,~] = pl_c2.timeExpandedEnergyAStar( ...
                            startPt, goalPt, 0, true, st_time_step, st_time_horizon);
                        [~,d_s]   = cm_c2.evaluatePath(p_s, 0, true);
                    case 5  % Greedy: 确定性
                        [p_s,~,~] = pl_c2.greedyPlanner(startPt, goalPt, 0, true);
                        [~,d_s]   = cm_c2.evaluatePath(p_s, 0, true);
                end
                stat_J(a,col_idx) = d_s.J_final;
                stat_E(a,col_idx) = d_s.E_total;
                stat_P(a,col_idx) = d_s.penalty_total;
                stat_feasible(a,col_idx) = logical(d_s.feasible);
                stat_paths{a,col_idx} = p_s;
            catch
                stat_J(a,col_idx) = NaN;
                stat_E(a,col_idx) = NaN;
                stat_P(a,col_idx) = NaN;
                stat_feasible(a,col_idx) = false;
            end
        end

        % 进度条
        pct = col_idx/N_STAT; blen=20; filled=round(pct*blen);
        fprintf('\r  [%s%s] %2d/%d (env%d seed%d)', ...
            repmat('#',1,filled), repmat('-',1,blen-filled), ...
            col_idx, N_STAT, env_count, s);
    end
end
fprintf('\n  数据收集完毕\n');
fprintf('  纳入环境 seeds: ');
fprintf('%d ', env_seeds_used); fprintf('\n');
fprintf('  ALA 配置: popSize=%d, maxIter=%d (统计轮次增强版)\n', ...
    ala_cfg_stat.popSize, ala_cfg_stat.maxIter);

% ── 颜色 ──
algColors_stat = [0.75 0.13 0.13;   % RA-ALA  红
                  0.16 0.50 0.73;   % EA*     蓝
                  0.15 0.63 0.25;   % RRT*    绿
                  0.49 0.18 0.56;   % ST-EA*  紫
                  0.58 0.58 0.58];  % Greedy  灰

%% ====================================================================
%%  图7: 环境级箱线图
%% ====================================================================
plotDistributionalRobustness(stat_J,stat_E,stat_feasible,stat_env, ...
    env_seeds_used,algNames,'fig7_distributional_robustness.png');
fprintf('  fig7 (all-path distributional robustness) saved\n');

%% ====================================================================
%%  图8在保存固定队列后由环境级聚类感知统计生成。
%%  三个内部种子先在每个环境内聚合，独立推断单位为 10 个城市环境。
% 保存第5.5节同队列数据，供消融、权重和分辨率实验严格复用。
main_collision_sample_spacing_m = costModel.collision_sample_spacing;
main_min_collision_samples = costModel.min_collision_samples;
save('main_experiment_cohort.mat', ...
    'env_seeds_used','stat_env','stat_ra_seed','stat_J','stat_E','stat_P','stat_feasible','stat_paths', ...
    'ala_cfg_stat','N_ENV','N_SEED','N_STAT', ...
    'mapSize','gridStep','windLevel','riskLevel','startPt','goalPt', ...
    'algNames','st_time_step','st_time_horizon', ...
    'main_collision_sample_spacing_m','main_min_collision_samples');
fprintf('  Main experiment cohort saved: main_experiment_cohort.mat\n');
clusterStats = runClusterAwareStatistics('main_experiment_cohort.mat');
plotClusterAwareStatistics(clusterStats, 'fig8_statistical_significance.png');


%% ====================================================================
%%  消融实验 (fig9)
%%  设计：每次只关闭一个组件，与完整 RA-ALA 对比。
%%  4 个有效变体：
%%    A1 w/o Smooth        — 去掉平滑度导向项 δ_smooth
%%    A2 w/o Headwind      — 去掉逆风前瞻导向项 G_headwind
%%    A3 w/o Unified Eval  — 跳过 Top-K 统一重评估，直接输出 raw
%%
%%  设计: 严格复用多环境统计实验（Section 5.5）的配对样本
%%    外层直接使用 env_seeds_used，不再为消融独立筛选环境
%%    内层直接使用 stat_ra_seed，不改变 RA-ALA 的随机过程
%%    Full 与所有消融变体均使用 ala_cfg_stat 的相同计算预算
%%    因此 Full RA-ALA 应逐案例复现 Section 5.5 的 RA-ALA 结果
%% ====================================================================

fprintf('\n━━━ Ablation Study (fig9): Component Contribution Analysis ━━━\n');

N_ABL_ENV = N_ENV;   % 与 Section 5.5 完全一致
N_ABL_REP = N_SEED;  % 与 Section 5.5 完全一致
N_ABL     = N_STAT;  % 同一批 10 环境 × 3 种子 = 30 个配对案例
fprintf('  设计: 严格复用 Section 5.5 (%d 环境 × %d 种子, N=%d)\n', ...
    N_ABL_ENV, N_ABL_REP, N_ABL);

ablationNames = {'Full RA-ALA', 'w/o Smooth', ...
                 'w/o Headwind Guidance', 'w/o Unified Eval'};
% 消融变体说明：
%   Full RA-ALA       — 完整方法（基准）
%   w/o Smooth        — 去掉 evaluateRAALASearchFitness 平滑度导向罚项
%   w/o Headwind      — 将 windLookahead 设为 0，去掉逆风前瞻导向项
%   w/o Unified Eval  — 去掉统一重评估，直接输出 evaluateRAALASearchFitness 内部适应度
%                       最优路径（raw），不经过 Top-K 比较 raw vs smooth。
%                       证明"统一评估驱动"的实质贡献：
%                       若内部适应度口径与报告口径不一致，搜索到的"最优路径"
%                       在统一评估下未必真的最优。
nAbl = length(ablationNames);

% ── 消融用 ALA 配置：严格复用 Section 5.5 的计算预算 ──
cfg_abl_base = ala_cfg_stat; % popSize=40, maxIter=80, rescue_max_ins=12

% ── 构建各变体 cfg ──
cfgs_abl = cell(nAbl, 1);
for aa = 1:nAbl
    cfgs_abl{aa} = cfg_abl_base;
end
cfgs_abl{2}.ablate_smoothPenalty = true;
cfgs_abl{3}.windLookahead = 0;
cfgs_abl{4}.ablate_unifiedEval = true;

% ── 严格复用 Section 5.5 的环境与 RA-ALA 内部随机种子 ──
if ~exist('env_seeds_used','var') || numel(env_seeds_used) ~= N_ABL_ENV
    error('Ablation requires the %d environment seeds produced by Section 5.5.', N_ABL_ENV);
end
if ~exist('stat_ra_seed','var') || numel(stat_ra_seed) ~= N_ABL
    error('Ablation requires the %d RA-ALA run seeds recorded in Section 5.5.', N_ABL);
end
abl_env_seeds = env_seeds_used;
abl_run_seeds = stat_ra_seed;
fprintf('  复用环境 seeds: %s\n', mat2str(abl_env_seeds));
fprintf('  复用 RA-ALA run seeds: %s\n\n', mat2str(abl_run_seeds));

% ── 结果矩阵 ──
abl_J   = zeros(nAbl, N_ABL);
abl_E   = zeros(nAbl, N_ABL);
abl_T   = zeros(nAbl, N_ABL);
abl_R   = zeros(nAbl, N_ABL);
abl_Pen = zeros(nAbl, N_ABL);
abl_feasible = false(nAbl, N_ABL);

% ── 正式运行 ──
for aa = 1:nAbl
    fprintf('  [%d/%d] %-20s: ', aa, nAbl, ablationNames{aa});
    col = 0;
    for ei = 1:N_ABL_ENV
        % 重建对应环境
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
            % Match the Section 5.5 RA-ALA seed exactly; all variants share it.
            rng(abl_run_seeds(col));
            try
                [~,~,det_abl] = runRA_ALA(pl_run, cm_run, env_run, ...
                    startPt, goalPt, 0, true, cfgs_abl{aa});
                abl_J(aa,col)   = det_abl.J_final;
                abl_E(aa,col)   = det_abl.E_total;
                abl_T(aa,col)   = det_abl.T_total;
                abl_R(aa,col)   = det_abl.R_dynamic;
                abl_Pen(aa,col) = det_abl.penalty_total;
                abl_feasible(aa,col) = logical(det_abl.feasible);
            catch
                abl_J(aa,col) = NaN; abl_E(aa,col) = NaN;
                abl_T(aa,col) = NaN; abl_R(aa,col) = NaN; abl_Pen(aa,col) = NaN;
                abl_feasible(aa,col) = false;
            end
            pct = col/N_ABL; blen = 15; filled = round(pct*blen);
            fprintf('\r  [%d/%d] %-20s: [%s%s] %2d/%d', aa, nAbl, ...
                ablationNames{aa}, repmat('█',1,filled), ...
                repmat('░',1,blen-filled), col, N_ABL);
        end
    end
    fprintf('\r  [%d/%d] %-20s: J=%.1f±%.1f  E=%.1f±%.1f  Pen=%.3f\n', ...
        aa, nAbl, ablationNames{aa}, ...
        mean(abl_J(aa,:),'omitnan'), std(abl_J(aa,:),'omitnan'), ...
        mean(abl_E(aa,:),'omitnan'), std(abl_E(aa,:),'omitnan'), ...
        mean(abl_Pen(aa,:),'omitnan'));
end

% Full RA-ALA should reproduce Section 5.5 case by case.
same_nan_J = isequal(isnan(abl_J(1,:)), isnan(stat_J(1,:)));
same_nan_E = isequal(isnan(abl_E(1,:)), isnan(stat_E(1,:)));
same_nan_P = isequal(isnan(abl_Pen(1,:)), isnan(stat_P(1,:)));
same_feasible = isequal(abl_feasible(1,:), stat_feasible(1,:));
max_diff_J = max(abs(abl_J(1,:)   - stat_J(1,:)), [], 'omitnan');
max_diff_E = max(abs(abl_E(1,:)   - stat_E(1,:)), [], 'omitnan');
max_diff_P = max(abs(abl_Pen(1,:) - stat_P(1,:)), [], 'omitnan');
repro_tol = 1e-9;
if same_nan_J && same_nan_E && same_nan_P && same_feasible && ...
        max_diff_J <= repro_tol && max_diff_E <= repro_tol && max_diff_P <= repro_tol
    fprintf('\n  [复现检查] PASS: Full RA-ALA 与 Section 5.5 的 30 个结果逐案例一致。\n');
else
    warning(['Full RA-ALA did not exactly reproduce Section 5.5: ', ...
        'max|dJ|=%.3g, max|dE|=%.3g, max|dPen|=%.3g.'], ...
        max_diff_J, max_diff_E, max_diff_P);
end

% ── 打印命令行汇总表 ──
fprintf('\n╔══════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║  消融实验汇总 (%d 环境×%d 种子=N=%d, High 复杂度, t=0)             ║\n', N_ABL_ENV, N_ABL_REP, N_ABL);
fprintf('╠══════════════════════╦══════════╦══════════╦══════════╦════════╦═════════╣\n');
fprintf('║  变体                ║  J (均值)║  E (Wh)  ║  风险    ║  惩罚  ║ 不可行率║\n');
fprintf('╠══════════════════════╬══════════╬══════════╬══════════╬════════╬═════════╣\n');
for aa = 1:nAbl
    rel = '';
    if aa > 1
        delta_J = (mean(abl_J(aa,:),'omitnan') - mean(abl_J(1,:),'omitnan')) ...
                  / mean(abl_J(1,:),'omitnan') * 100;
        rel = sprintf(' (%+.1f%%)', delta_J);
    end
    infeas_rate = sum(~abl_feasible(aa,:)) / N_ABL * 100;
    fprintf('║  %-20s║  %6.2f%-3s║  %6.2f  ║  %6.4f  ║ %6.4f ║  %4.0f%%   ║\n', ...
        ablationNames{aa}, ...
        mean(abl_J(aa,:),'omitnan'),   rel, ...
        mean(abl_E(aa,:),'omitnan'), ...
        mean(abl_R(aa,:),'omitnan'), ...
        mean(abl_Pen(aa,:),'omitnan'), infeas_rate);
end
fprintf('╚══════════════════════╩══════════╩══════════╩══════════╩════════╩═════════╝\n');
fprintf('  注: 不可行率按严格标准计算：任一硬约束违规或运行失败均不可行；相对变化 = vs Full RA-ALA 均值\n');
fprintf('  实验设计: 严格复用 Section 5.5 的环境、RA-ALA 种子与计算预算\n');
fprintf('  ALA容量: popSize=%d, maxIter=%d\n', cfg_abl_base.popSize, cfg_abl_base.maxIter);

%% ── 绘制 fig9 ──
plotAblationStudyFigure(abl_J, abl_E, abl_T, abl_R, abl_Pen, ...
    ablationNames, N_ABL, 'fig9_ablation_study.png');

save('ablation_same_cohort_results.mat', ...
    'abl_J','abl_E','abl_T','abl_R','abl_Pen','abl_feasible','ablationNames', ...
    'abl_env_seeds','abl_run_seeds','cfg_abl_base', ...
    'stat_J','stat_E','stat_P','stat_feasible','stat_env','stat_ra_seed', ...
    'N_ABL_ENV','N_ABL_REP','N_ABL');
fprintf('  Ablation raw data saved: ablation_same_cohort_results.mat\n');
fprintf('  fig9 (Ablation Study - Component Contribution) saved\n');

%% ====================================================================
%%  Cost-weight sensitivity analysis
%%  Uses one pre-specified seed from each of the same 10 Section 5.5
%%  environments. Raw J is compared only within a fixed weight setting.
%% ====================================================================
if RUN_WEIGHT_SENSITIVITY
    runWeightSensitivityAnalysis('main_experiment_cohort.mat');
end
if RUN_SPATIAL_RESOLUTION_SENSITIVITY
    runSpatialResolutionSensitivity('main_experiment_cohort.mat');
end


fprintf('\n全部完成!\n');

%% ====================================================================
%%  RA-ALA 核心算法函数
%% ====================================================================
function fig=renderPathQualityComparison(env,paths,algNames,algColors, ...
        algStyles,algWidths,startPt,goalPt,mapSize)
%RENDERPATHQUALITYCOMPARISON Common renderer for full and redraw-only modes.
    buildings=env.buildings;
    movObs=env.dynObstacles.movingObs;
    tempNFZ=env.dynObstacles.tempNFZ;
    nAlg=numel(algNames);
    fig=figure('Units','centimeters','Position',[0.5 0.5 36 17], ...
        'Color','w','Renderer','painters');
    tl=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');

    ax=nexttile(tl,1); hold(ax,'on'); axis(ax,'equal');
    set(ax,'FontSize',17,'FontName','Times New Roman');
    for i=1:size(buildings,1)
        cx=buildings(i,1); cy=buildings(i,2); bh=buildings(i,3);
        hw=buildings(i,4); hh=buildings(i,5); gv=max(0.45,0.88-bh/200);
        rectangle(ax,'Position',[cx-hw,cy-hh,2*hw,2*hh], ...
            'FaceColor',[gv gv gv],'EdgeColor',[.35 .35 .35], ...
            'LineWidth',.35,'HandleVisibility','off');
    end
    hNFZ=gobjects(1);
    for i=1:numel(tempNFZ)
        z=tempNFZ(i); th=linspace(0,2*pi,60);
        hThisNFZ=fill(ax,z.center(1)+z.radius*cos(th),z.center(2)+z.radius*sin(th), ...
            [1 .88 .88],'EdgeColor',[.85 .1 .1],'LineWidth',1, ...
            'LineStyle','--','HandleVisibility','off');
        if i==1
            hNFZ=hThisNFZ;
        end
    end
    for i=1:numel(movObs)
        pos=env.dynObstacles.getPosition(i,0); th=linspace(0,2*pi,36);
        fill(ax,pos(1)+movObs(i).radius*cos(th), ...
            pos(2)+movObs(i).radius*sin(th),[1 .72 .38], ...
            'EdgeColor',[.8 .4 0],'LineWidth',.7,'HandleVisibility','off');
    end
    h2=gobjects(nAlg,1);
    for a=1:nAlg
        p=paths{a}; if isempty(p),continue;end
        h2(a)=plot(ax,p(:,1),p(:,2),algStyles{a}, ...
            'Color',algColors(a,:),'LineWidth',algWidths(a));
    end
    plot(ax,startPt(1),startPt(2),'p','MarkerSize',13, ...
        'MarkerFaceColor',[0 .65 0],'MarkerEdgeColor','k','HandleVisibility','off');
    plot(ax,goalPt(1),goalPt(2),'h','MarkerSize',13, ...
        'MarkerFaceColor',[.9 .1 .1],'MarkerEdgeColor','k','HandleVisibility','off');
    xlabel(ax,'X (m)','FontSize',18); ylabel(ax,'Y (m)','FontSize',18);
    title(ax,'(a) 2D paths for departure time t_0 = 0 s', ...
        'FontSize',20,'FontWeight','bold');
    xlim(ax,[0 mapSize]); ylim(ax,[0 mapSize]); grid(ax,'on'); box(ax,'on');
    lgdNFZ=legend(ax,hNFZ,'Temporary NFZ footprint (time-dependent)', ...
        'Location','northwest','FontSize',17,'Box','on');
    lgdNFZ.ItemTokenSize=[24 12];

    ax3=nexttile(tl,2); hold(ax3,'on');
    set(ax3,'FontSize',17,'FontName','Times New Roman');
    for i=1:size(buildings,1)
        cx=buildings(i,1); cy=buildings(i,2); bh=buildings(i,3);
        hw=buildings(i,4); hh=buildings(i,5);
        vx=[cx-hw cx+hw cx+hw cx-hw cx-hw cx+hw cx+hw cx-hw];
        vy=[cy-hh cy-hh cy+hh cy+hh cy-hh cy-hh cy+hh cy+hh];
        vz=[0 0 0 0 bh bh bh bh];
        fc=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
        gv=.86-.12*min(bh/180,1);
        patch(ax3,'Vertices',[vx' vy' vz'],'Faces',fc, ...
            'FaceColor',[gv gv min(gv*1.02,1)],'EdgeColor',[.45 .45 .48], ...
            'LineWidth',.3,'HandleVisibility','off');
    end
    h3=gobjects(nAlg,1);
    for a=1:nAlg
        p=paths{a}; if isempty(p),continue;end
        h3(a)=plot3(ax3,p(:,1),p(:,2),p(:,3),algStyles{a}, ...
            'Color',algColors(a,:),'LineWidth',algWidths(a));
    end
    plot3(ax3,startPt(1),startPt(2),startPt(3),'p','MarkerSize',12, ...
        'MarkerFaceColor',[0 .65 0],'MarkerEdgeColor','k','HandleVisibility','off');
    plot3(ax3,goalPt(1),goalPt(2),goalPt(3),'h','MarkerSize',12, ...
        'MarkerFaceColor',[.9 .1 .1],'MarkerEdgeColor','k','HandleVisibility','off');
    xlabel(ax3,'X (m)','FontSize',18); ylabel(ax3,'Y (m)','FontSize',18);
    zlabel(ax3,'Z (m)','FontSize',18);
    title(ax3,'(b) 3D paths for departure time t_0 = 0 s', ...
        'FontSize',20,'FontWeight','bold');
    view(ax3,35,28); grid(ax3,'on'); box(ax3,'on');
    xlim(ax3,[0 mapSize]); ylim(ax3,[0 mapSize]); zlim(ax3,[0 200]);
    set(ax3,'ZTick',0:100:200); pbaspect(ax3,[1 1 .5]);

    ok=isgraphics(h3);
    lgd=legend(ax3,h3(ok),algNames(ok),'Orientation','horizontal', ...
        'NumColumns',sum(ok),'FontSize',19,'Location','southoutside');
    lgd.Layout.Tile='south'; lgd.ItemTokenSize=[30 16];
    title(tl,'Representative Paths in a High-Complexity Urban Environment', ...
        'FontName','Times New Roman','FontSize',21,'FontWeight','bold');
end
