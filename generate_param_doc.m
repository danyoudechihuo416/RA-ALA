%% generate_param_doc.m
%% =========================================================================
%%  RA-ALA 参数文档生成器
%%  运行方式: 在 MATLAB 中执行 generate_param_doc
%%  输出文件: RA_ALA_参数文档.txt (纯文本，可用任意编辑器查看)
%%           RA_ALA_参数文档.html (网页格式，浏览器打开)
%%
%%  文档涵盖六大参数类别:
%%    1. 全局实验参数       (run_RA_ALA.m)
%%    2. ALA 算法参数       (run_RA_ALA.m → runRA_ALA)
%%    3. 统一代价函数参数   (UnifiedCostModel.m)
%%    4. 无人机物理参数     (UnifiedCostModel.m)
%%    5. 城市环境参数       (CityEnvironment.m)
%%    6. 二次救援参数       (run_RA_ALA.m v10)
%% =========================================================================

clc;
fprintf('正在生成参数文档...\n');

%% ==================== 读取当前代码中的实际参数值 ====================
%% （与 run_RA_ALA.m / UnifiedCostModel.m / CityEnvironment.m 保持同步）

%% ---- 1. 全局实验参数 ----
p_seed           = 42;
p_mapSize        = 1000;      % 地图尺寸 (m)
p_gridStep       = 10;        % 网格分辨率 (m)
p_windLevel      = 'medium';  % 风场强度
p_riskLevel      = 'dense';   % 障碍密度
p_startPt        = [80, 80, 60];
p_goalPt         = [900, 900, 60];
p_departureTimes = [0, 60, 120, 180, 240];  % 出发时刻集合 (s)
p_cityLevels     = {'low', 'medium', 'high'};

%% ---- 2. ALA 算法参数 ----
p_popSize        = 30;     % 种群大小
p_maxIter        = 60;     % 最大迭代次数
p_nWaypoints     = 8;      % 航路点数目 (不含起终点)
p_topK           = 5;      % Top-K 候选数
p_riskWeight_cfg = 15.0;   % cfg.riskWeight (evalRA_v2 内部)
p_windLookahead  = 3;      % 风场预测步数
p_maxLat         = 200;    % 横向偏移最大范围 (m)
p_safeH3_frac    = 0.60;   % 初始化候选3高度比例 (60% H 层)
p_safeH4_frac    = 0.40;   % 初始化候选4高度比例 (40% H 层)
% ALA算子概率
p_prob_follow    = 0.30;   % 领导跟随
p_prob_windwalk  = 0.25;   % 风场偏置游走
p_prob_spiral    = 0.25;   % 螺旋搜索
p_prob_levy      = 0.20;   % Lévy飞行
% ALA迭代衰减
p_levy_beta      = 1.5;    % Lévy指数
p_wind_bias_fac  = 0.30;   % 风场偏置系数
p_wind_noise_sc  = 0.06;   % 随机游走噪声尺度
p_spiral_decay   = 0.80;   % 螺旋搜索衰减系数
% evalRA_v2 内部惩罚参数
p_smooth_coeff   = 0.3;    % 平滑度导向系数 (原8→0.3)
p_smooth_cap     = 15.0;   % 平滑度导向上界
p_infeas_pen     = 150.0;  % 不可行门槛惩罚 (原5000→150)
p_prox_nsub      = 4;      % 高度proxy子采样数
p_prox_cap       = 50.0;   % 高度proxy上界
p_prox_hcoef     = 2.0;    % 高度proxy系数
p_nfz_coeff_in   = 3.0;    % NFZ穿越系数 (原500→3)
p_nfz_coeff_mar  = 0.3;    % NFZ边缘系数 (原50→0.3)
p_obs_coeff      = 2.0;    % 动态障碍接近系数 (原300→2)
p_nfz_cap        = 25.0;   % NFZ惩罚上界
p_obs_cap        = 20.0;   % 障碍惩罚上界
p_eval_nsub      = 4;      % evalRA_v2段级子采样数
% Smooth / Repair 路径参数
p_smooth_dper    = 15;     % 样条采样间距 (m/点)
p_smooth_minpts  = 20;     % 样条最少点数
p_smooth_weights = [0.2, 0.6, 0.2]; % 轻量平滑权重 [prev,cur,next]
p_repair_maxpass = 3;      % Phase-1最大轮数
p_seg_passes     = 5;      % Phase-4段级validate轮数
p_seg_nsub       = 8;      % Phase-4每段子采样数

%% ---- 3. 统一代价函数权重参数 ----
p_w_energy       = 1.0;    % 能耗权重 w_e
p_w_time         = 0.5;    % 时间权重 w_t (T/60)
p_w_climb        = 2.0;    % 爬升代价权重 w_c
p_w_risk         = 10.0;   % 动态风险权重 w_r
p_w_height       = 50.0;   % 高度惩罚导向权重 w_height (仅内部)
p_lambda         = 100.0;  % 惩罚系数 λ
p_feas_thresh    = 0.1;    % 可行性判据 P < feas_thresh

%% ---- 4. 无人机物理参数 ----
p_m_frame        = 3.5;    % 机身质量 (kg)
p_m_payload      = 1.0;    % 载荷质量 (kg)
p_m_total        = 4.5;    % 总质量 (kg)
p_v_cruise       = 15.0;   % 巡航速度 (m/s)
p_eta_motor      = 0.85;   % 电机效率
p_eta_prop       = 0.75;   % 桨叶效率
p_eta_total      = 0.6375; % 综合效率 η_m × η_p
p_n_rotor        = 4;      % 旋翼数量
p_r_prop         = 0.15;   % 桨叶半径 (m)
p_A_disc         = 4*pi*0.15^2; % 总盘面积 (m²)
p_rho_air        = 1.225;  % 空气密度 (kg/m³)
p_C_d            = 0.25;   % 阻力系数
p_A_body         = 0.04;   % 机体横截面积 (m²)
p_E_batt         = 200.0;  % 电池容量 (Wh)
p_E_batt_limit   = 180.0;  % 可用电量上限 0.9×200 (Wh)
p_climb_eff      = 0.7;    % 爬升效率
p_descent_rec    = 0.3;    % 下降能量回收率
p_risk_horizon   = 60;     % 风险评估时间窗口 (s)
p_collision_r    = 15;     % 碰撞检测半径 (m)

%% ---- 5. 城市环境参数 ----
p_H_min          = 30;     % 最低飞行高度 (m)
p_H_max          = 120;    % 最高飞行高度 (m)
p_H_clearance    = 5;      % 建筑净空 (m)
p_nBldg_low      = 15;     % 低复杂度建筑数
p_nBldg_med      = 35;     % 中复杂度建筑数
p_nBldg_high     = 60;     % 高复杂度建筑数
p_hRange_low     = [30, 60];    % 低复杂度建筑高度范围 (m)
p_hRange_med     = [40, 120];   % 中复杂度建筑高度范围 (m)
p_hRange_high    = [50, 180];   % 高复杂度建筑高度范围 (m)
p_nObs_low       = 3;      % 低复杂度动态障碍数
p_nObs_med       = 6;      % 中复杂度动态障碍数
p_nObs_high      = 12;     % 高复杂度动态障碍数
p_obs_speed_rng  = [6, 12]; % 障碍速度范围 (m/s)
p_obs_r_rng      = [10, 35]; % 障碍安全半径范围 (m)
p_obs_orbit_rng  = [60, 140]; % 圆形障碍轨道半径范围 (m)
p_nNFZ_low       = 1;      % 低复杂度NFZ数
p_nNFZ_med       = 3;      % 中复杂度NFZ数
p_nNFZ_high      = 5;      % 高复杂度NFZ数
p_wind_z0        = 0.1;    % 风场粗糙度长度 (m)
p_wind_zref      = 10;     % 风场参考高度 (m)
p_wind_period    = 120;    % 风场时变周期 (s)
p_wind_amp       = 0.4;    % 风场时变幅度系数

%% ---- 6. 二次救援 v10 参数 ----
p_max_ins        = 6;      % Pass-A 最大插入次数
p_sub_sp         = 12;     % 碰撞检测子采样间距 (m)
p_min_sub_rc     = 3;      % 最小子采样数
p_detour_d       = [30, 60, 100, 150, 200, 300]; % 绕行距离选项 (m)
p_n_dirs         = 8;      % 搜索方向数 (等间隔)
p_n_hz_strat     = 2;      % 高度策略数 (横向+飞越顶部)
p_above_margin   = 15;     % 飞越顶部裕量 (m)
p_total_cands    = 3*8*6*2; % 每轮总候选数 = 288
p_viol_scale     = 2.5;    % Pass-B 违规抬升比例系数
p_extra_margin   = 8;      % Pass-B 额外裕量 (m)
p_viol_thresh    = 0.02;   % Pass-B 违规量阈值 (m)
p_nfix_b         = 20;     % Pass-B 最多修复段数

%% ==================== 生成文本文档 ====================

fid = fopen('RA_ALA_参数文档.txt', 'w', 'n', 'UTF-8');
W = @(s) fprintf(fid, '%s\n', s);

W('╔══════════════════════════════════════════════════════════════════╗');
W('║         RA-ALA 方法完整参数配置文档                              ║');
W('║  面向时变城市低空环境的风险感知能耗优化三维路径规划              ║');
W('╚══════════════════════════════════════════════════════════════════╝');
W(sprintf('  生成时间: %s', datestr(now, 'yyyy-mm-dd HH:MM:SS')));
W(sprintf('  生成脚本: generate_param_doc.m'));
W('');

%% --- 类别1 ---
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  【1】全局实验参数   (run_RA_ALA.m)');
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W(sprintf('  %-30s = %-12d  %% 随机种子', 'seed', p_seed));
W(sprintf('  %-30s = %-12d  %% 城市地图尺寸 (m)', 'mapSize', p_mapSize));
W(sprintf('  %-30s = %-12d  %% 网格分辨率 (m)', 'gridStep', p_gridStep));
W(sprintf('  %-30s = %-12s  %% 风场强度: weak/medium/strong/variable', 'windLevel', ['''',p_windLevel,'''']));
W(sprintf('  %-30s = %-12s  %% 障碍密度: sparse/medium/dense', 'riskLevel', ['''',p_riskLevel,'''']));
W(sprintf('  %-30s = [%d,%d,%d]     %% 起点坐标 (m)', 'startPt', p_startPt(1),p_startPt(2),p_startPt(3)));
W(sprintf('  %-30s = [%d,%d,%d]   %% 终点坐标 (m)', 'goalPt', p_goalPt(1),p_goalPt(2),p_goalPt(3)));
W(sprintf('  %-30s = [0,60,120,180,240]  %% 出发时刻集合 (s)', 'departureTimes'));
W('');

%% --- 类别2 ---
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  【2】ALA 算法参数   (run_RA_ALA.m → runRA_ALA / evalRA_v2)');
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  ---- 2.1 种群与迭代 ----');
W(sprintf('  %-30s = %-12d  %% 种群大小', 'ala_cfg.popSize', p_popSize));
W(sprintf('  %-30s = %-12d  %% 最大迭代次数', 'ala_cfg.maxIter', p_maxIter));
W(sprintf('  %-30s = %-12d  %% 航路点数 (不含起终点)', 'ala_cfg.nWaypoints', p_nWaypoints));
W(sprintf('  %-30s = %-12d  %% Top-K 候选数 (output阶段)', 'K', p_topK));
W(sprintf('  %-30s = %-12.1f  %% cfg.riskWeight (evalRA_v2内部)', 'ala_cfg.riskWeight', p_riskWeight_cfg));
W(sprintf('  %-30s = %-12d  %% 风场预测步数', 'ala_cfg.windLookahead', p_windLookahead));
W(sprintf('  %-30s = %-12d  %% 横向偏移最大范围 (m)', 'maxLat', p_maxLat));
W('');
W('  ---- 2.2 ALA 搜索算子概率 ----');
W(sprintf('  %-30s = %-12.2f  %% 领导跟随 (r1 < 0.30)', 'P_follow', p_prob_follow));
W(sprintf('  %-30s = %-12.2f  %% 风场偏置游走 (r1 < 0.55)', 'P_windwalk', p_prob_windwalk));
W(sprintf('  %-30s = %-12.2f  %% 螺旋搜索 (r1 < 0.80)', 'P_spiral', p_prob_spiral));
W(sprintf('  %-30s = %-12.2f  %% Lévy飞行 (r1 >= 0.80)', 'P_levy', p_prob_levy));
W('');
W('  ---- 2.3 迭代衰减与算子参数 ----');
W(sprintf('  %-30s = 2*atan(1-iter/maxIter)  %% 步长控制 (随迭代衰减)', 'theta'));
W(sprintf('  %-30s = 1-0.6*iter/maxIter      %% 步长衰减因子', 'sigma_decay'));
W(sprintf('  %-30s = %-12.1f  %% Lévy指数 β', 'levy_beta', p_levy_beta));
W(sprintf('  %-30s = %-12.2f  %% 风场偏置系数', 'wind_bias_factor', p_wind_bias_fac));
W(sprintf('  %-30s = %-12.2f  %% 游走噪声尺度 (相对范围)', 'wind_noise_scale', p_wind_noise_sc));
W('');
W('  ---- 2.4 种群初始化 (5类高度感知候选) ----');
W(sprintf('  %-30s  直线路径, z=startPt(3)=60m', 'C1'));
W(sprintf('  %-30s  贪心路径节点提取', 'C2'));
W(sprintf('  %-30s = %.2f  → z≈%.0fm  (60%%高度层)', 'C3: safeH3_frac', p_safeH3_frac, 30+p_safeH3_frac*90));
W(sprintf('  %-30s = %.2f  → z≈%.0fm  (40%%高度层)', 'C4: safeH4_frac', p_safeH4_frac, 30+p_safeH4_frac*90));
W(sprintf('  %-30s  贴建筑顶飞行 z=h_map+H_clearance+5', 'C5'));
W(sprintf('  %-30s  随机初始化 lat∈(-120,120), z∈(H_min,H_max)', 'C6~popSize'));
W('');
W('  ---- 2.5 evalRA_v2 内部惩罚参数 ----');
W(sprintf('  %-30s = %-12.1f  %% 平滑度导向系数 (原8→0.3)', 'SMOOTH_COEFF', p_smooth_coeff));
W(sprintf('  %-30s = %-12.1f  %% 平滑度导向上界', 'SMOOTH_CAP', p_smooth_cap));
W(sprintf('  %-30s = %-12.1f  %% 不可行门槛惩罚 (原5000→150)', 'INFEAS_PEN', p_infeas_pen));
W(sprintf('  %-30s = %-12d  %% 高度proxy子采样数', 'PROX_NSUB', p_prox_nsub));
W(sprintf('  %-30s = %-12.1f  %% 高度proxy上界', 'PROX_CAP', p_prox_cap));
W(sprintf('  %-30s = %-12.1f  %% 高度proxy系数', 'PROX_HCOEF', p_prox_hcoef));
W(sprintf('  %-30s = %-12.1f  %% NFZ穿越系数 (原500→3)', 'NFZ_COEFF_IN', p_nfz_coeff_in));
W(sprintf('  %-30s = %-12.1f  %% NFZ边缘系数 (原50→0.3)', 'NFZ_COEFF_MAR', p_nfz_coeff_mar));
W(sprintf('  %-30s = %-12.1f  %% 动态障碍接近系数 (原300→2)', 'OBS_COEFF', p_obs_coeff));
W(sprintf('  %-30s = %-12.1f  %% NFZ惩罚总量上界', 'NFZ_CAP', p_nfz_cap));
W(sprintf('  %-30s = %-12.1f  %% 障碍接近惩罚上界', 'OBS_CAP', p_obs_cap));
W(sprintf('  %-30s = %-12d  %% evalRA_v2段级子采样数', 'EVAL_NSUB', p_eval_nsub));
W('');
W('  ---- 2.6 三路径生成参数 ----');
W(sprintf('  %-30s = max(20, d_total/%-2d)  %% 样条上采样点数公式', 'smooth_M', p_smooth_dper));
W(sprintf('  %-30s = %-12s  %% 样条插值方法', 'smooth_method', '''PCHIP'''));
W(sprintf('  %-30s = [%.1f, %.1f, %.1f]   %% 轻量平滑权重 [prev,cur,next]', 'smooth_w', p_smooth_weights(1),p_smooth_weights(2),p_smooth_weights(3)));
W(sprintf('  %-30s = %-12d  %% Phase-1点级修复最大轮数', 'MAX_PASS', p_repair_maxpass));
W(sprintf('  %-30s = %-12d  %% Phase-4段级validate轮数', 'SEG_PASSES', p_seg_passes));
W(sprintf('  %-30s = %-12d  %% Phase-4每段子采样数', 'SEG_NSUB', p_seg_nsub));
W(sprintf('  %-30s = nPts_init         %% 全局动态插点上限 (=初始路径点数)', 'MAX_TOTAL_INS'));
W('');

%% --- 类别3 ---
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  【3】统一代价函数参数   (UnifiedCostModel.m)');
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  J(π,t₀) = w_e·E + w_t·T/60 + w_c·C_climb + w_r·R_dyn + λ·P');
W('');
W('  ---- 3.1 代价函数权重 ----');
W(sprintf('  %-30s = %-12.1f  %% 能耗权重 w_e', 'w_energy', p_w_energy));
W(sprintf('  %-30s = %-12.1f  %% 时间权重 w_t (T单位=min)', 'w_time', p_w_time));
W(sprintf('  %-30s = %-12.1f  %% 爬升代价权重 w_c', 'w_climb', p_w_climb));
W(sprintf('  %-30s = %-12.1f  %% 动态风险权重 w_r', 'w_risk', p_w_risk));
W(sprintf('  %-30s = %-12.1f  %% 高度导向权重 (仅内部search, 不进final J)', 'w_height', p_w_height));
W(sprintf('  %-30s = %-12.1f  %% 惩罚系数 λ', 'lambda_penalty', p_lambda));
W(sprintf('  %-30s = %-12.1f  %% 可行性判据阈值 P < %.1f', 'feas_thresh', p_feas_thresh, p_feas_thresh));
W('');
W('  ---- 3.2 惩罚分项子采样参数 ----');
W('  P_dyn: n_sub = max(3, ceil(d_k/12))  %% 动态碰撞子采样间距12m');
W('  P_height: 同段级子采样, 仅检查p1端点(末点单独检查, 防重复计费)');
W(sprintf('  %-30s = %-12.1f  %% 爬升效率 (对应 P_climb 项)', 'climb_efficiency', p_climb_eff));
W(sprintf('  %-30s = %-12.1f  %% 下降能量回收率', 'descent_recovery', p_descent_rec));
W(sprintf('  %-30s = %-12d  %% 风险评估时间窗口 (s)', 'risk_horizon', p_risk_horizon));
W(sprintf('  %-30s = %-12d  %% 碰撞检测安全半径 (m)', 'collision_radius', p_collision_r));
W('');

%% --- 类别4 ---
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  【4】无人机物理参数   (UnifiedCostModel.m properties)');
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  P_hover = (mg)^1.5 / (sqrt(2·ρ·A_disc)·η)   [动量理论]');
W(sprintf('  %-30s = %-12.1f  %% 机身质量 (kg)', 'm_frame', p_m_frame));
W(sprintf('  %-30s = %-12.1f  %% 载荷质量 (kg)', 'm_payload', p_m_payload));
W(sprintf('  %-30s = %-12.1f  %% 总质量 m_frame+m_payload (kg)', 'm_total', p_m_total));
W(sprintf('  %-30s = %-12.1f  %% 巡航速度 (m/s)', 'v_cruise', p_v_cruise));
W(sprintf('  %-30s = %-12.2f  %% 电机效率', 'eta_motor', p_eta_motor));
W(sprintf('  %-30s = %-12.2f  %% 桨叶效率', 'eta_prop', p_eta_prop));
W(sprintf('  %-30s = %-12.4f  %% 综合效率 η_m×η_p', 'eta_total', p_eta_total));
W(sprintf('  %-30s = %-12d  %% 旋翼数量', 'n_rotor', p_n_rotor));
W(sprintf('  %-30s = %-12.2f  %% 桨叶半径 (m)', 'r_prop', p_r_prop));
W(sprintf('  %-30s = %-12.4f  %% 总盘面积 = n_rotor·π·r² (m²)', 'A_disc', p_A_disc));
W(sprintf('  %-30s = %-12.3f  %% 空气密度 (kg/m³)', 'rho_air', p_rho_air));
W(sprintf('  %-30s = %-12.2f  %% 阻力系数 C_d', 'C_d', p_C_d));
W(sprintf('  %-30s = %-12.2f  %% 机体横截面积 (m²)', 'A_body', p_A_body));
W(sprintf('  %-30s = %-12.1f  %% 电池总容量 (Wh)', 'E_batt', p_E_batt));
W(sprintf('  %-30s = %-12.1f  %% 可用容量上限 0.9×E_batt (Wh)', 'E_batt_limit', p_E_batt_limit));
W('');

%% --- 类别5 ---
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  【5】城市环境参数   (CityEnvironment.m)');
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W(sprintf('  %-30s = %-12d  %% 地图边长 (m)', 'MAP_SIZE', p_mapSize));
W(sprintf('  %-30s = %-12d  %% 网格步长 (m)', 'GRID_STEP', p_gridStep));
W(sprintf('  %-30s = %-12d  %% 最低飞行高度 (m)', 'H_min', p_H_min));
W(sprintf('  %-30s = %-12d  %% 最高飞行高度 (m)', 'H_max', p_H_max));
W(sprintf('  %-30s = %-12d  %% 建筑净空裕量 (m)', 'H_clearance', p_H_clearance));
W('');
W('  ---- 5.1 建筑参数 (按复杂度) ----');
W(sprintf('  %-10s  nBuildings=%-4d  hRange=[%d,%d]m  sizeRange=[20,40]m', 'Low:', p_nBldg_low, p_hRange_low(1),p_hRange_low(2)));
W(sprintf('  %-10s  nBuildings=%-4d  hRange=[%d,%d]m  sizeRange=[15,35]m', 'Medium:', p_nBldg_med, p_hRange_med(1),p_hRange_med(2)));
W(sprintf('  %-10s  nBuildings=%-4d  hRange=[%d,%d]m  sizeRange=[10,30]m', 'High:', p_nBldg_high, p_hRange_high(1),p_hRange_high(2)));
W('');
W('  ---- 5.2 动态障碍参数 ----');
W(sprintf('  Low/Med/High: %d / %d / %d 个障碍', p_nObs_low, p_nObs_med, p_nObs_high));
W(sprintf('  %-30s = [%d, %d]  %% 障碍速度范围 (m/s)', 'obs_speed_range', p_obs_speed_rng(1),p_obs_speed_rng(2)));
W(sprintf('  %-30s = [%d, %d]  %% 安全半径范围 (m)', 'obs_radius_range', p_obs_r_rng(1),p_obs_r_rng(2)));
W(sprintf('  %-30s = [%d, %d]  %% 圆形轨道半径范围 (m)', 'obs_orbit_range', p_obs_orbit_rng(1),p_obs_orbit_rng(2)));
W('  轨迹类型: 圆形轨道 / 莱萨如曲线 / 线性往返 (各占约1/3)');
W('');
W('  ---- 5.3 临时禁飞区 (NFZ) ----');
W(sprintf('  Low/Med/High: %d / %d / %d 个NFZ', p_nNFZ_low, p_nNFZ_med, p_nNFZ_high));
W('  NFZ属性: center(x,y), radius, z_range[z_lo,z_hi], 激活时间窗口[t_start,t_end]');
W('');
W('  ---- 5.4 风场模型参数 ----');
W('  w_x(z,t) = w_0(t)·ln(z/z_0)/ln(z_ref/z_0)·(1+0.4·sin(2πt/120))');
W(sprintf('  %-30s = %-12.1f  %% 粗糙度长度 (m)', 'wind_z0', p_wind_z0));
W(sprintf('  %-30s = %-12d  %% 参考高度 (m)', 'wind_zref', p_wind_zref));
W(sprintf('  %-30s = %-12d  %% 时变周期 (s)', 'wind_period', p_wind_period));
W(sprintf('  %-30s = %-12.1f  %% 时变幅度系数', 'wind_amp', p_wind_amp));
W('');

%% --- 类别6 ---
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  【6】二次救援 v10 参数   (run_RA_ALA.m → 定向二次救援 v10)');
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  ---- 6.1 Pass-A 时空绕行点插入 ----');
W(sprintf('  %-30s = %-12d  %% 最大插入次数 (迭代上限)', 'MAX_INS_TOTAL', p_max_ins));
W(sprintf('  %-30s = %-12d  %% 碰撞检测子采样间距 (m)', 'SUB_SP', p_sub_sp));
W(sprintf('  %-30s = %-12d  %% 最小子采样数', 'MIN_SUB_RC', p_min_sub_rc));
W(sprintf('  %-30s = [30,60,100,150,200,300]  %% 绕行距离候选 (m)', 'DETOUR_D'));
W(sprintf('  %-30s = %-12d  %% 搜索方向数 (0~315度均匀)', 'n_dirs', p_n_dirs));
W(sprintf('  %-30s = %-12d  %% 高度策略数 (横向/飞越顶部)', 'n_hz_strategies', p_n_hz_strat));
W(sprintf('  %-30s = %-12d  %% 飞越顶部裕量 (m)', 'above_margin', p_above_margin));
W(sprintf('  %-30s = 3×8×6×2 = %-4d  %% 每轮总候选数', 'total_candidates', p_total_cands));
W('');
W('  ---- 6.2 三个插入位置 (v10核心创新) ----');
W('  Pos-A: max(1, k_c-1)  → 碰撞段之前 (延迟到达时刻)');
W('  Pos-B: k_c            → 碰撞段之中 (分割长段, 解耦碰撞子点) ★关键');
W('  Pos-C: k_c+1          → 碰撞段之后 (延迟离开时刻)');
W('');
W('  ---- 6.3 Pass-B 高度/静态比例抬升 ----');
W('  抬升公式: ℓ_k = VIOL_SCALE × v_k + EXTRA_MARGIN');
W(sprintf('  %-30s = %-12.1f  %% 违规抬升比例系数', 'VIOL_SCALE', p_viol_scale));
W(sprintf('  %-30s = %-12d  %% 额外裕量 (m)', 'EXTRA_MARGIN', p_extra_margin));
W(sprintf('  %-30s = %-12.2f  %% 违规量阈值 (低于此值跳过)', 'VIOL_THRESH', p_viol_thresh));
W(sprintf('  %-30s = %-12d  %% 最多修复段数', 'N_FIX_B', p_nfix_b));
W('');

W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  【参数速查表】代价函数权重汇总');
W('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
W('  J = 1.0×E + 0.5×T/60 + 2.0×C_climb + 10.0×R_dyn + 100×P');
W('');
W('  参数名          数值        单位         说明');
W('  ─────────────────────────────────────────────────────────────────');
W(sprintf('  w_e            %-8.1f    Wh^-1       能耗权重', p_w_energy));
W(sprintf('  w_t            %-8.1f    min^-1      时间权重', p_w_time));
W(sprintf('  w_c            %-8.1f    —           爬升代价权重', p_w_climb));
W(sprintf('  w_r            %-8.1f    —           动态风险权重', p_w_risk));
W(sprintf('  λ              %-8.1f    —           惩罚系数', p_lambda));
W(sprintf('  P_thresh       %-8.1f    —           可行性判据阈值', p_feas_thresh));
W(sprintf('  m_total        %-8.1f    kg          无人机总质量', p_m_total));
W(sprintf('  v_cruise       %-8.1f    m/s         巡航速度', p_v_cruise));
W(sprintf('  E_batt         %-8.1f    Wh          电池容量', p_E_batt));
W(sprintf('  H_min/H_max    %d/%d       m           飞行高度范围', p_H_min, p_H_max));
W(sprintf('  H_clearance    %-8d    m           建筑净空', p_H_clearance));
W(sprintf('  popSize/maxIter %-d/%-d     —           ALA种群/迭代', p_popSize, p_maxIter));
W(sprintf('  MAX_INS_TOTAL  %-8d    —           v10最大插入次数', p_max_ins));
W(sprintf('  total_cands    %-8d    —           v10每轮候选数', p_total_cands));
W('');
W('  文件来源对照:');
W('    run_RA_ALA.m    → 类别1,2,6  (全局参数/ALA算法/救援机制)');
W('    UnifiedCostModel.m → 类别3,4  (代价函数/无人机物理)');
W('    CityEnvironment.m  → 类别5    (城市环境/风场)');
W('');
W(sprintf('  文档生成完成: %s', datestr(now, 'yyyy-mm-dd HH:MM:SS')));

fclose(fid);
fprintf('✓ RA_ALA_参数文档.txt 已生成\n');

%% ==================== 同时生成 HTML 版本 ====================

fh = fopen('RA_ALA_参数文档.html', 'w', 'n', 'UTF-8');
WH = @(s) fprintf(fh, '%s\n', s);

WH('<!DOCTYPE html><html><head><meta charset="utf-8">');
WH('<title>RA-ALA 参数文档</title>');
WH('<style>');
WH('body{font-family:Consolas,Courier,monospace;font-size:13px;background:#1e1e2e;color:#cdd6f4;margin:20px;line-height:1.6}');
WH('h1{color:#89b4fa;border-bottom:2px solid #89b4fa;padding-bottom:8px}');
WH('h2{color:#a6e3a1;margin-top:24px;border-left:4px solid #a6e3a1;padding-left:10px}');
WH('.param{color:#cba6f7} .val{color:#fab387} .cmt{color:#6c7086}');
WH('.key{color:#f38ba8;font-weight:bold} .eq{color:#cdd6f4}');
WH('.section{background:#181825;border:1px solid #313244;border-radius:6px;padding:12px 16px;margin:10px 0}');
WH('.formula{background:#1e1e2e;color:#f9e2af;border-left:3px solid #f9e2af;padding:8px 12px;margin:8px 0;font-size:14px}');
WH('.highlight{background:#45475a;border-radius:3px;padding:1px 4px;color:#f38ba8}');
WH('.table{width:100%;border-collapse:collapse;margin:10px 0}');
WH('.table th{background:#313244;color:#89b4fa;padding:6px 10px;text-align:left}');
WH('.table td{padding:5px 10px;border-bottom:1px solid #313244}');
WH('.table tr:hover td{background:#262637}');
WH('</style></head><body>');
WH(sprintf('<h1>RA-ALA 参数文档 &nbsp;<small style="font-size:12px;color:#6c7086">生成于 %s</small></h1>', datestr(now)));
WH('<p style="color:#6c7086">面向时变城市低空环境的风险感知能耗优化三维路径规划</p>');

% 速查表
WH('<h2>&#9889; 参数速查表 — 代价函数</h2>');
WH('<div class="formula">J(&pi;, t<sub>0</sub>) = <b>1.0</b>&middot;E + <b>0.5</b>&middot;T/60 + <b>2.0</b>&middot;C<sub>climb</sub> + <b>10.0</b>&middot;R<sub>dyn</sub> + <b>100</b>&middot;P</div>');
WH('<table class="table"><tr><th>参数</th><th>数值</th><th>单位</th><th>说明</th></tr>');
params_table = {
    'w_e',    '1.0',    'Wh⁻¹',  '能耗权重';
    'w_t',    '0.5',    'min⁻¹', '时间权重 (T单位=min)';
    'w_c',    '2.0',    '—',      '爬升代价权重';
    'w_r',    '10.0',   '—',      '动态风险权重';
    'λ',      '100.0',  '—',      '惩罚系数';
    'P阈值',  '0.1',    '—',      '可行性判据 P<0.1';
    'm',      '4.5',    'kg',     '无人机总质量(3.5+1.0)';
    'v',      '15.0',   'm/s',    '巡航速度';
    'E_batt', '200.0',  'Wh',     '电池总容量';
    'H_min',  '30',     'm',      '最低飞行高度';
    'H_max',  '120',    'm',      '最高飞行高度';
    'popSize','30',     '—',      'ALA种群大小';
    'maxIter','60',     '—',      'ALA最大迭代次数';
    'MAX_INS','6',      '—',      'v10最大插入次数';
    '候选数', '288',    '/轮',    'v10每轮总候选(3×8×6×2)';
};
for ri = 1:size(params_table,1)
    WH(sprintf('<tr><td class="key">%s</td><td class="val">%s</td><td>%s</td><td class="cmt">%s</td></tr>', ...
        params_table{ri,1},params_table{ri,2},params_table{ri,3},params_table{ri,4}));
end
WH('</table>');

sections = {
    '全局实验参数', 'run_RA_ALA.m', ...
    ['seed=42  mapSize=1000m  gridStep=10m  windLevel=''medium''  riskLevel=''dense''<br>',...
     'startPt=[80,80,60]  goalPt=[900,900,60]<br>',...
     'departureTimes=[0,60,120,180,240]s'];
    'ALA种群与搜索参数', 'run_RA_ALA.m → runRA_ALA', ...
    ['popSize=30  maxIter=60  nWaypoints=8  Top-K=5  maxLat=200m<br>',...
     '算子概率: 领导跟随30% / 风场游走25% / 螺旋25% / Lévy20%<br>',...
     'θ=2·atan(1-iter/60)  σ_decay=1-0.6·iter/60  Lévy β=1.5'];
    '无人机物理参数', 'UnifiedCostModel.m', ...
    ['m_frame=3.5kg  m_payload=1.0kg  v_cruise=15m/s<br>',...
     'η_motor=0.85  η_prop=0.75  n_rotor=4  r_prop=0.15m<br>',...
     'ρ=1.225kg/m³  C_d=0.25  A_body=0.04m²  E_batt=200Wh'];
    '城市环境参数', 'CityEnvironment.m', ...
    ['Low:15栋[30-60m]  Med:35栋[40-120m]  High:60栋[50-180m]<br>',...
     '障碍 Low/Med/High: 3/6/12个  速度6-12m/s  半径10-35m  轨道60-140m<br>',...
     'NFZ Low/Med/High: 1/3/5个  风场: 对数剖面+时变扰动(周期120s)'];
    '二次救援v10参数', 'run_RA_ALA.m → v10救援', ...
    ['MAX_INS=6  SUB_SP=12m  DETOUR_D=[30,60,100,150,200,300]m<br>',...
     'Pos-A(段前)/Pos-B(段中★)/Pos-C(段后)  8方向×6距离×2高度=288候选/轮<br>',...
     'Pass-B: ℓ_k=2.5·v_k+8m  VIOL_SCALE=2.5  EXTRA_MARGIN=8m'];
};

for si = 1:size(sections,1)
    WH(sprintf('<h2>%d. %s <small style="font-size:11px;color:#6c7086">%s</small></h2>', ...
        si, sections{si,1}, sections{si,2}));
    WH(sprintf('<div class="section">%s</div>', sections{si,3}));
end

WH('<hr style="border-color:#313244;margin:20px 0">');
WH('<p style="color:#6c7086;font-size:11px">文件来源: run_RA_ALA.m (全局/ALA/救援) | UnifiedCostModel.m (代价/物理) | CityEnvironment.m (环境/风场)</p>');
WH('</body></html>');
fclose(fh);
fprintf('✓ RA_ALA_参数文档.html 已生成\n');
fprintf('\n所有文档已生成完毕。\n');
