%% =========================================================================
%%  diagnose_RA_ALA.m
%%  å•æ¡ˆä¾‹è¯Šæ–­æ¨¡å¼ â€” åˆ†æž RA-ALA åœ¨æŒ‡å®šåœºæ™¯ä¸‹ J å¼‚å¸¸çš„æ ¹æœ¬åŽŸå› 
%% =========================================================================
%%
%%  é—®é¢˜èƒŒæ™¯:
%%    high åŸŽå¸‚ / t=0s åœºæ™¯ä¸‹, RA-ALA è¾“å‡º Eâ‰ˆ18Whã€Tâ‰ˆ210sã€Risk=0,
%%    ä½† Jâ‰ˆ699.8, è¿œé«˜äºŽå…¶ä»–æ—¶åˆ» (~16~20). æœ¬è„šæœ¬å¯¹è¯¥åœºæ™¯è¿›è¡Œ
%%    å…¨é“¾è·¯ä»£ä»·æ‹†è§£, å®šä½å¼‚å¸¸æ¥æº.
%%
%%  è¯Šæ–­å†…å®¹:
%%    1. raw / smooth / repair ä¸‰é˜¶æ®µè·¯å¾„çš„å®Œæ•´ä»£ä»·åˆ†è§£
%%    2. æœ€ç»ˆè·¯å¾„é€æ®µè¯Šæ–­ (æ—¶é—´/é£Žé™©/æƒ©ç½š/NFZ/éšœç¢æŽ¥è¿‘åº¦)
%%    3. å¼‚å¸¸æ®µæ ‡çº¢è­¦å‘Š
%%    4. é‡å¤è®¡è´¹æ£€æµ‹ (å¯¹æ¯”ä¿®å¤å‰/ä¿®å¤åŽ UnifiedCostModel ç‰ˆæœ¬)
%%    5. penalty æ‹†è§£é¥¼å›¾ + é€æ®µæƒ©ç½šçƒ­å›¾
%%
%%  ç”¨æ³•:
%%    ç›´æŽ¥è¿è¡Œ, æ— éœ€ä¿®æ”¹å…¶ä»–æ–‡ä»¶.
%%    ä¾èµ–: CityEnvironment.m, UnifiedCostModel.m, PathPlanners.m,
%%           run_RA_ALA.m (éœ€åœ¨åŒä¸€ç›®å½•ä¸‹ä»¥èŽ·å– runRA_ALA å‡½æ•°)
%% =========================================================================

clear; clc; close all;
warning off;

fprintf('â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—\n');
fprintf('â•‘  RA-ALA å•æ¡ˆä¾‹è¯Šæ–­ â€” J å¼‚å¸¸æ ¹å› åˆ†æž                        â•‘\n');
fprintf('â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•\n\n');

%% ==================== å¯è°ƒå‚æ•° ====================
DIAG_SEED       = 42;
DIAG_CITY       = 'high';    % åŸŽå¸‚å¤æ‚åº¦: low / medium / high
DIAG_T_START    = 0;         % å‡ºå‘æ—¶åˆ» (s), æ”¹æˆå…¶ä»–å€¼å¯å¯¹æ¯”
DIAG_WIND       = 'medium';
DIAG_RISK       = 'dense';
DIAG_MAP_SIZE   = 1000;
DIAG_GRID_STEP  = 10;
DIAG_START_PT   = [80,  80,  60];
DIAG_GOAL_PT    = [900, 900, 60];

% ALA è¶…å‚ (ä¸Ž run_RA_ALA.m ä¿æŒä¸€è‡´)
ala_cfg.popSize       = 30;
ala_cfg.maxIter       = 60;
ala_cfg.nWaypoints    = 8;
ala_cfg.riskWeight    = 15.0;
ala_cfg.windLookahead = 3;

% è¯Šæ–­é˜ˆå€¼ (è¶…è¿‡å³æ ‡ä¸ºå¼‚å¸¸)
THRESH_SEG_PENALTY  = 0.05;   % å•æ®µ penalty > æ­¤å€¼è§†ä¸ºå¼‚å¸¸
THRESH_NFZ_MARGIN   = 1.5;    % åˆ° NFZ ä¸­å¿ƒçš„è·ç¦» < radiusÃ—æ­¤å€æ•° æ—¶è­¦å‘Š
THRESH_OBS_MARGIN   = 2.5;    % åˆ°åŠ¨æ€éšœç¢çš„è·ç¦» < radiusÃ—æ­¤å€æ•° æ—¶è­¦å‘Š
THRESH_HEIGHT_BELOW = 5;      % ä½ŽäºŽ H_min è¶…è¿‡æ­¤å€¼ (m) è§†ä¸ºä¸¥é‡è¿è§„

%% ==================== æž„å»ºçŽ¯å¢ƒ ====================

fprintf('[1/6] æž„å»º %s åŸŽå¸‚çŽ¯å¢ƒ (t=%.0fs)...\n', DIAG_CITY, DIAG_T_START);
rng(DIAG_SEED);
env = CityEnvironment(DIAG_MAP_SIZE, DIAG_GRID_STEP);
env.generate(DIAG_CITY, DIAG_WIND, DIAG_RISK, DIAG_SEED);
env.setTaskPoints(DIAG_START_PT, DIAG_GOAL_PT);

costModel = UnifiedCostModel();
costModel.setEnvironment(env.windField, env.dynObstacles, env.heightMap);

planner = PathPlanners(env, costModel);
planner.setBudget(15, 5000, 2000);

fprintf('   å»ºç­‘ç‰©: %d  åŠ¨æ€éšœç¢: %d  ç¦é£žåŒº: %d\n', ...
    size(env.buildings, 1), ...
    length(env.dynObstacles.movingObs), ...
    length(env.dynObstacles.tempNFZ));

% æ‰“å° NFZ è¯¦æƒ…
fprintf('   NFZ åˆ—è¡¨:\n');
for ni = 1:length(env.dynObstacles.tempNFZ)
    nfz = env.dynObstacles.tempNFZ(ni);
    active_at_t0 = (DIAG_T_START >= nfz.t_start && DIAG_T_START <= nfz.t_end);
    fprintf('     NFZ%d  center=[%.0f,%.0f]  r=%.0f  h=[%.0f,%.0f]  t=[%.0f,%.0f]', ...
        ni, nfz.center(1), nfz.center(2), nfz.radius, ...
        nfz.height(1), nfz.height(2), nfz.t_start, nfz.t_end);
    if active_at_t0
        fprintf('  â† â˜… åœ¨ t=%.0fs æ—¶ ACTIVE', DIAG_T_START);
    end
    fprintf('\n');
end

%% ==================== è¿è¡Œ RA-ALA å¹¶å–ä¸‰é˜¶æ®µè·¯å¾„ ====================

fprintf('\n[2/6] è¿è¡Œ RA-ALA (seed=%d)...\n', DIAG_SEED + 100);
rng(DIAG_SEED + 100);
[final_path, final_cost, final_det, stage_det] = ...
    runRA_ALA(planner, costModel, env, DIAG_START_PT, DIAG_GOAL_PT, ...
              DIAG_T_START, true, ala_cfg);

fprintf('   å®Œæˆ. final_J=%.3f  internal_search_J=%.3f\n', ...
    final_cost, final_det.internal_search_cost);

%% ==================== [è¯Šæ–­1] ä¸‰é˜¶æ®µä»£ä»·åˆ†è§£ ====================

fprintf('\n[3/6] ä¸‰é˜¶æ®µä»£ä»·åˆ†è§£\n');
fprintf('%s\n', repmat('â•', 1, 72));

stages  = {stage_det.raw, stage_det.smooth, stage_det.repair};
slabels = {'raw path (ALAç›´æŽ¥è¾“å‡º)', 'smooth path (æ ·æ¡å¹³æ»‘åŽ)', 'repair path (ä¿®å¤åŽ=æœ€ç»ˆ)'};

for si = 1:3
    d   = stages{si};
    lbl = slabels{si};
    fprintf('\n  â”€â”€â”€ é˜¶æ®µ %d: %s â”€â”€â”€\n', si, lbl);
    fprintf('  è·¯å¾„ç‚¹æ•°: %d\n', length(d.t_arrivals));
    fprintf('  J_final      = %10.4f\n', d.J_final);
    fprintf('    w_eÃ—E      = %10.4f   (E       = %.4f Wh)\n',  1.0*d.E_total, d.E_total);
    fprintf('    w_tÃ—T/60   = %10.4f   (T       = %.2f s)\n',   0.5*d.T_total/60, d.T_total);
    fprintf('    w_cÃ—C      = %10.4f   (C_climb = %.6f)\n',     2.0*d.C_climb, d.C_climb);
    fprintf('    w_rÃ—R      = %10.4f   (R_dyn   = %.6f)\n',     10.0*d.R_dynamic, d.R_dynamic);
    fprintf('    Î»Ã—Pen      = %10.4f   (Pen     = %.6f)\n',     100*d.penalty_total, d.penalty_total);
    fprintf('  â”Œâ”€ Penalty åˆ†è§£ â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€\n');
    fprintf('  â”‚  height            = %8.6f  â†’ Jè´¡çŒ® %7.3f\n', d.penalty_height, 100*d.penalty_height);
    fprintf('  â”‚  static_collision  = %8.6f  â†’ Jè´¡çŒ® %7.3f\n', d.penalty_static_collision, 100*d.penalty_static_collision);
    fprintf('  â”‚  dynamic_collision = %8.6f  â†’ Jè´¡çŒ® %7.3f\n', d.penalty_dynamic_collision, 100*d.penalty_dynamic_collision);
    fprintf('  â”‚  battery           = %8.6f  â†’ Jè´¡çŒ® %7.3f\n', d.penalty_battery, 100*d.penalty_battery);
    fprintf('  â””â”€ NFZç¡¬ç½š(å·²å…¥J)      = %8.3f\n', d.NFZ_penalty);
    fprintf('  feasible=%d  heightViol=%d  SoC_end=%.1f%%\n', ...
        d.feasible, d.heightViolations, d.SoC_end*100);
end

% é˜¶æ®µé—´å¯¹æ¯”
fprintf('\n  â”€â”€â”€ é˜¶æ®µé—´ä»£ä»·å˜åŒ– â”€â”€â”€\n');
fprintf('  rawâ†’smooth:  Î”J=%+.3f  Î”Pen=%+.6f  Î”hViol=%+d\n', ...
    stages{2}.J_final - stages{1}.J_final, ...
    stages{2}.penalty_total - stages{1}.penalty_total, ...
    stages{2}.heightViolations - stages{1}.heightViolations);
fprintf('  smoothâ†’repair: Î”J=%+.3f  Î”Pen=%+.6f  Î”hViol=%+d\n', ...
    stages{3}.J_final - stages{2}.J_final, ...
    stages{3}.penalty_total - stages{2}.penalty_total, ...
    stages{3}.heightViolations - stages{2}.heightViolations);
fprintf('  internal_search_J=%+.3f  vs  final_J=%.3f  å·®å€¼=%+.3f\n', ...
    stage_det.internal_search_cost, final_det.J_final, ...
    stage_det.internal_search_cost - final_det.J_final);

% é‡å¤è®¡è´¹æ£€æµ‹
if stages{3}.penalty_total > stages{2}.penalty_total + 0.01
    fprintf('\n  [!] â˜… æ£€æµ‹åˆ° repair é˜¶æ®µå¼•å…¥é¢å¤–æƒ©ç½š!\n');
    fprintf('      smooth.penalty=%.4f â†’ repair.penalty=%.4f (å¢žåŠ %.4f)\n', ...
        stages{2}.penalty_total, stages{3}.penalty_total, ...
        stages{3}.penalty_total - stages{2}.penalty_total);
    fprintf('      å¯èƒ½åŽŸå› : postSmoothRepair ç§»ç‚¹æ“ä½œæŠŠè·¯å¾„ç‚¹æŽ¨å…¥å»ºç­‘æˆ–ä½ŽäºŽ H_min\n');
end

%% ==================== [è¯Šæ–­2] é€æ®µè¯Šæ–­ ====================

fprintf('\n[4/6] æœ€ç»ˆè·¯å¾„é€æ®µè¯Šæ–­\n');
fprintf('%s\n', repmat('â•', 1, 72));

path  = final_path;
det   = final_det;
N     = size(path, 1);
nSegs = N - 1;

tArr  = det.t_arrivals;       % [NÃ—1] æ¯ç‚¹åˆ°è¾¾æ—¶åˆ»
ESeg  = det.E_segments;       % [N-1]
TSeg  = det.T_segments;       % [N-1]

% ä»Ž evaluatePath ä¸­æˆ‘ä»¬æ‹¿åˆ°äº†æ®µçº§æ•°ç»„; ä½† R/penalty æŒ‰æ®µçº§æš‚æ— å•ç‹¬å­—æ®µ,
% éœ€è¦å¯¹æœ€ç»ˆè·¯å¾„åšä¸€æ¬¡"å¸¦æ®µçº§è¾“å‡º"çš„è¯¦ç»†å†è¯„ä¼°
[~, seg_diag] = evaluatePath_segLevel(costModel, env, path, DIAG_T_START, true);

% ç¡®è®¤æ®µæ•°ä¸€è‡´
assert(length(seg_diag) == nSegs, 'æ®µæ•°ä¸ä¸€è‡´, æ£€æŸ¥ evaluatePath_segLevel è¾“å‡º');

% æ‰“å°è¡¨å¤´
fprintf('\n  %4s %8s %8s %8s %8s %6s %6s %6s %6s  %s\n', ...
    'seg', 't_start', 'dt(s)', 'E(Wh)', 'R_risk', ...
    'Pen', 'Pen_h', 'Pen_s', 'Pen_d', 'è­¦å‘Š');
fprintf('  %s\n', repmat('-', 1, 90));

n_anomalous  = 0;
anomaly_segs = [];

for k = 1:nSegs
    sd = seg_diag(k);

    % åŸºç¡€å€¼
    t_seg_start = tArr(k);
    dt          = TSeg(k);
    e_seg       = ESeg(k);
    r_seg       = sd.R_risk;
    p_total     = sd.Pen_total;
    p_h         = sd.Pen_height;
    p_s         = sd.Pen_static;
    p_d         = sd.Pen_dyn;

    % æž„é€ è­¦å‘Šå­—ç¬¦ä¸²
    warn = '';
    if p_total > THRESH_SEG_PENALTY
        warn = [warn sprintf('[PEN=%.3f!]', p_total)];
        n_anomalous = n_anomalous + 1;
        anomaly_segs(end+1) = k; %#ok<AGROW>
    end
    if sd.nfz_proximity_flag
        warn = [warn sprintf('[NFZè¿‘(%.0fm)]', sd.nfz_min_dist)];
    end
    if sd.obs_proximity_flag
        warn = [warn sprintf('[OBSè¿‘(%.0fm)]', sd.obs_min_dist)];
    end
    if sd.height_below > THRESH_HEIGHT_BELOW
        warn = [warn sprintf('[H_ä½Ž%.1fm!]', sd.height_below)];
    end
    if sd.static_hit
        warn = [warn '[ç©¿æ¥¼!]'];
    end

    % é€‰æ‹©æ€§æ‰“å°: å¼‚å¸¸æ®µå…¨æ‰“; æ­£å¸¸æ®µæ¯5æ®µæ‰“ä¸€æ¬¡
    if p_total > THRESH_SEG_PENALTY || sd.nfz_proximity_flag || ...
       sd.obs_proximity_flag || sd.height_below > 1 || mod(k, 5) == 0
        fprintf('  %4d %8.1f %8.2f %8.4f %8.5f %6.4f %6.4f %6.4f %6.4f  %s\n', ...
            k, t_seg_start, dt, e_seg, r_seg, p_total, p_h, p_s, p_d, warn);
    end
end

fprintf('\n  æ±‡æ€»: å…± %d æ®µ, å¼‚å¸¸æ®µ %d ä¸ª (penalty>%.3f)\n', ...
    nSegs, n_anomalous, THRESH_SEG_PENALTY);
if ~isempty(anomaly_segs)
    fprintf('  â˜… å¼‚å¸¸æ®µç´¢å¼•: [%s]\n', num2str(anomaly_segs));
end

%% ==================== [è¯Šæ–­3] é€ç‚¹é«˜åº¦æ£€æŸ¥ ====================

fprintf('\n[5/6] é€ç‚¹é«˜åº¦åˆè§„æ€§æ£€æŸ¥\n');
fprintf('%s\n', repmat('â•', 1, 72));

MS       = env.MAP_SIZE;
H_min    = costModel.H_min;
H_max    = costModel.H_max;
H_clear  = costModel.H_clearance;

n_viol = 0;
viol_detail = {};

fprintf('  %4s %8s %8s %8s %8s %8s  %s\n', ...
    'idx', 'x', 'y', 'z', 'ground_h', 'min_floor', 'çŠ¶æ€');
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
            status = sprintf('â˜… ä½Ž%.1fm (éœ€>=%.1f)', min_floor - pt(3), min_floor);
        else
            status = sprintf('â˜… é«˜%.1fm (éœ€<=%.1f)', pt(3) - H_max, H_max);
        end
        viol_detail{end+1} = sprintf('  ç‚¹%3d: [%.1f,%.1f,%.1f]  ground=%.1f  min=%.1f  â†’ %s', ...
            k, pt(1), pt(2), pt(3), ground_h, min_floor, status);
        fprintf('  %4d %8.1f %8.1f %8.1f %8.1f %8.1f  %s\n', ...
            k, pt(1), pt(2), pt(3), ground_h, min_floor, status);
    end
end

if n_viol == 0
    fprintf('  âœ“ æ‰€æœ‰ %d ä¸ªè·¯å¾„ç‚¹å‡æ»¡è¶³é«˜åº¦çº¦æŸ\n', N);
else
    fprintf('\n  â˜… å…± %d ä¸ªè·¯å¾„ç‚¹è¿åé«˜åº¦çº¦æŸ\n', n_viol);
    fprintf('  é«˜åº¦æƒ©ç½š = Î£(è¿åé‡/10) = %.4f  â†’ Î»Ã—Pen_h = %.3f\n', ...
        final_det.penalty_height, 100*final_det.penalty_height);
end

%% ==================== [è¯Šæ–­4] NFZ ä¸Žéšœç¢æŽ¥è¿‘åº¦å…¨è·¯å¾„æ‰«æ ====================

fprintf('\n[6/6] å…¨è·¯å¾„ NFZ / åŠ¨æ€éšœç¢æŽ¥è¿‘åº¦æ‰«æ\n');
fprintf('%s\n', repmat('â•', 1, 72));

nfzList = env.dynObstacles.tempNFZ;
obsList = env.dynObstacles.movingObs;

% NFZ æŽ¥è¿‘åº¦
fprintf('  NFZ æŽ¥è¿‘åº¦ (è·¯å¾„ç‚¹ vs æ‰€æœ‰æ´»è·ƒ NFZ):\n');
fprintf('  %4s %8s %8s  %s\n', 'idx', 't(s)', 'z(m)', 'æœ€è¿‘ NFZ ä¿¡æ¯');
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
            status = 'æŽ¥è¿‘';
            if inXY && inZ,  status = 'â˜… ç©¿è¶Š!'; end
            if inXY && ~inZ, status = 'â˜… XYåœ¨å†…(Zè§„é¿)'; end
            fprintf('  %4d %8.1f %8.1f  NFZ%d: dh=%.1fm(r=%.0f) z=[%.0f,%.0f] â†’ %s\n', ...
                k, tk, pt(3), ni, dh, nfz.radius, nfz.height(1), nfz.height(2), status);
        end
    end
end

% åŠ¨æ€éšœç¢æŽ¥è¿‘åº¦
fprintf('\n  åŠ¨æ€éšœç¢æŽ¥è¿‘åº¦ (è·¯å¾„ç‚¹ vs æ‰€æœ‰ç§»åŠ¨éšœç¢):\n');
fprintf('  %4s %8s  %s\n', 'idx', 't(s)', 'æŽ¥è¿‘éšœç¢ä¿¡æ¯');
fprintf('  %s\n', repmat('-', 1, 64));

for k = 1:N
    pt = path(k,:);
    tk = tArr(k);
    for oi = 1:length(obsList)
        obs = obsList(oi);
        op  = env.dynObstacles.getPosition(oi, tk);
        d   = norm(pt - op);
        if d < obs.radius * THRESH_OBS_MARGIN
            status = 'æŽ¥è¿‘';
            if d < obs.radius, status = 'â˜… ç¢°æ’ž!'; end
            fprintf('  %4d %8.1f  Obs%d: d=%.1fm(r=%.0f) pos=[%.0f,%.0f,%.0f] â†’ %s\n', ...
                k, tk, oi, d, obs.radius, op(1), op(2), op(3), status);
        end
    end
end

%% ==================== ç”Ÿæˆå¯è§†åŒ–å›¾è¡¨ ====================

fprintf('\nGenerating diagnostic figures...\n');

% ---- å›¾1: J ç»„æˆé¥¼å›¾ (ä¸‰é˜¶æ®µå¯¹æ¯”) ----
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
    lbls  = {'w_eÃ—E', 'w_tÃ—T/60', 'w_cÃ—C', 'w_rÃ—R', 'Î»Ã—Pen'};
    clrs  = [0.2 0.6 0.9; 0.3 0.8 0.4; 0.9 0.7 0.2; 0.9 0.4 0.4; 0.7 0.2 0.7];

    % è¿‡æ»¤æŽ‰é›¶æˆ–è´Ÿå€¼
    mask = vals > 1e-6;
    if any(mask)
        pie_h = pie(vals(mask));
        % ç€è‰²
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
sgtitle(sprintf('J ç»„æˆ (ä¸‰é˜¶æ®µå¯¹æ¯”)  city=%s  t=%.0fs', DIAG_CITY, DIAG_T_START), ...
    'FontSize', 12, 'FontWeight', 'bold');
exportPublicationFigure(fig_pie, sprintf('diag_pie_%s_t%d.png', DIAG_CITY, DIAG_T_START));

% ---- å›¾2: é€æ®µæƒ©ç½šçƒ­å›¾ ----
fig_heat = figure('Units','centimeters','Position',[1 1 30 14],'Color','w');

pen_matri×Íº¶‰žËkºwµçYÚ	Ë	Ø›Û	ÊNÂ™[™›YÙ[™
	ÓØØ][Û‰Ë	Û›ÜX\Ý	Ë	Ñ›ÛÚ^™IËÊNÂžX™[
	ÔÙYÛY[[™^	ÊNÈ[X™[
	Ô[˜[IÊNÂ]J	ú`$9«­y êyïf¹§¡9¢$	Ë	Ñ›ÛÚ^™IËL
NÈÜšYÛŽÂž[JÌ”ÙYÜÊÌWJNÂ‚œÝXœÝ
‹KŠNÂšÛÛŽÂ›Z[—Û™ž—Ù\ÝH[™Š”ÙYÜËJNÂ™›ÜˆÈHN›”ÙYÜÂˆÛZYH\œŠÊH
ÈÙYÊÊKÌŽÂˆÛZYH]
ËŠH
ÈJŠ]
ÊÌKŠK\]
ËŠJNÂˆ›ÜˆšHHN›[™Ý
™ž“\Ý
Bˆ™žˆH™ž“\Ý
šJNÂˆYˆ›™ž‹˜XÝ]™HÛZY™ž‹ÜÝ\ÛZYˆ™ž‹Ù[™ÛÛ[YNÈ[™ˆH›Ü›JÛZY
NŒŠHH™ž‹˜Ù[\ŠNÂˆZ[—Û™ž—Ù\Ý
ÊHHZ[ŠZ[—Û™ž—Ù\Ý
ÊK
NÂˆ[™™[™˜[YÛ™žˆH\Ùš[š]JZ[—Û™ž—Ù\Ý
NÂšYˆ[žJ˜[YÛ™žŠBˆÝ
š[™
˜[YÛ™žŠKZ[—Û™ž—Ù\Ý
˜[YÛ™žŠK	Û[ËIË	Ó[™UÚY	ËKK	ÓX\šÙ\”Ú^™IËJNÂ™[™‚‰H9d!‘–ˆ9cb¹o¡9càº  ùî¯Â›™ž—Ü˜YZHHÛ™ž“\Ýœ˜Y]\×NÂšYˆš\Ù[\J™ž—Ü˜YZJBˆ›ÜˆšHHN›[™Ý
™ž“\Ý
BˆYˆ[žJ˜[YÛ™žŠBˆ[[™J™ž“\Ý
šJKœ˜Y]\Ë	ËKIËÜš[Š	Ó‘–‰YIKŒ‰ËšK™ž“\Ý
šJKœ˜Y]\ÊK‹‹‚ˆ	ÐÛÛÜ‰ËÌHK	Ñ›ÛÚ^™IËÊNÂˆ[™ˆ[™™[™žX™[
	ÔÙYÛY[[™^	ÊNÈ[X™[
	ùb,9§ :/äy­.ú-àÈ‘–ˆ9æ¡:-çyé®È
JIÊNÂ]J	ú`$9«­H‘–ˆ9£©z/äyn©ˆ
:-¢¹/cº-¢¹clzfjJIË	Ñ›ÛÚ^™IËL
NÈÜšYÛŽÂž[JÌ”ÙYÜÊÌWJNÂ‚œÙÝ]JÜš[Š	ú`$9«­y êyïf¹.#“‘–¹£©z/äyn©ˆÚ]OI\ÈIKŒœÉËPQ×ÐÒUKPQ×ÕÔÕT•
K‹‹‚ˆ	Ñ›ÛÚ^™IËL‹	Ñ›ÛÙZYÚ	Ë	Ø›Û	ÊNÂ™^ÜX›XØ][Û‘šYÝ\™JšY×Ü[Œ‹Üš[Š	ÙXY×Ü[Œ—É\×Ý	Yœ™ÉËPQ×ÐÒUKPQ×ÕÔÕT•
JNÂ‚‰IHOOOOOOOOOOOOOOOOOOOH9§ 9îâ:+â¹¥«yîäú+®ˆOOOOOOOOOOOOOOOOOOOB‚™œš[Š	×¸¥e8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥e×‰ÊNÂ™œš[Š	ø¥dH:+â¹¥«yîäú+®ˆ8¥dW‰ÊNÂ™œš[Š	ø¥f¸¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥gW‰ÊNÂ‚’—Ü[—ØÛÛšX][ÛˆHL
ˆš[˜[Ù]œ[˜[WÝÝ[Â’—Û›Û—Ü[ˆHš[˜[Ù]’—Ùš[˜[H—Ü[—ØÛÛšX][ÛŽÂ‚™œš[Š	È—Ùš[˜[H	KŒÙ—‰Ëš[˜[Ù]’—Ùš[˜[
NÂ™œš[Š	È8¥%8¥ 9«hùn.9.èù.íÈ
JÕ
ÐÊÔŠHH	KŒÙˆ
9ch9«å	KŒY‰IJW‰Ë‹‹‚ˆ—Û›Û—Ü[‹L
’—Û›Û—Ü[‹Ùš[˜[Ù]’—Ùš[˜[
NÂ™œš[Š	È8¥%8¥ 9 êyïfºhnH3®ðåÔ[˜[HH	KŒÙˆ
9ch9«å	KŒY‰IJW‰Ë‹‹‚ˆ—Ü[—ØÛÛšX][Û‹L
’—Ü[—ØÛÛšX][Û‹Ùš[˜[Ù]’—Ùš[˜[
NÂ‚šYˆ—Ü[—ØÛÛšX][ÛˆˆH
ˆš[˜[Ù]’—Ùš[˜[ˆœš[Š	×ˆ8¦!H9..ú) yo ¹n.9§iy®¤ˆ[˜[H:hnych9«å:-¡z/áÈL	IW‰ÊNÂˆß‹[—ÛÜ™\—HHÛÜ
È‹‹‚ˆš[˜[Ù]œ[˜[WÚZYÚ‹‹‚ˆš[˜[Ù]œ[˜[WÜÝ]X×ØÛÛ\Ú[Û‹‹‹‚ˆš[˜[Ù]œ[˜[WÙ[˜[ZX×ØÛÛ\Ú[Û‹‹‹‚ˆš[˜[Ù]œ[˜[WØ˜]\žWK	Ù\ØÙ[™	ÊNÂˆ[—Û˜[Y\ÈHÉÜ[˜[WÚZYÚ	Ë	Ü[˜[WÜÝ]XÉË	Ü[˜[WÙ[‰Ë	Ü[˜[WØ˜]\žIßNÂˆ[—Ý˜[ÈHÙš[˜[Ù]œ[˜[WÚZYÚš[˜[Ù]œ[˜[WÜÝ]X×ØÛÛ\Ú[Û‹‹‹‚ˆš[˜[Ù]œ[˜[WÙ[˜[ZX×ØÛÛ\Ú[Û‹š[˜[Ù]œ[˜[WØ˜]\žWNÂˆœš[Š	È9§ 9i)È[˜[H:-(yã+ºhnNˆ	\ÈH	Kˆ8¡¤ˆº-(yã+ˆ	KŒ™—‰Ë‹‹‚ˆ[—Û˜[Y\ÞÜ[—ÛÜ™\ŠJ_K[—Ý˜[Ê[—ÛÜ™\ŠJJKL
œ[—Ý˜[Ê[—ÛÜ™\ŠJJJNÂ™[™‚‰H:f-¹«­y®«ù®¤š—Ü˜]ÈHÝYÙ\ÞÌ_K’—Ùš[˜[Âš—ÜÛ[ÛÝHÝYÙ\ÞÌŸK’—Ùš[˜[Âš—Ü™\Z\ˆHÝYÙ\ÞÌßK’—Ùš[˜[Â™œš[Š	×ˆ:f-¹«­y®«ù®¤—‰ÊNÂšYˆ—ÜÛ[ÛÝH—Ü˜]ÈˆLˆœš[Š	È8¡¤ˆÛ[ÛÝ]Ü[™H9o%yaiNˆ3¥IJËŒ™ˆ
9cëú ïynlù®äyîãú/áùnî¹ëdJW‰Ë—ÜÛ[ÛÝZ—Ü˜]ÊNÂ™[™šYˆ—Ü™\Z\ˆH—ÜÛ[ÛÝˆLˆœš[Š	È8¡¤ˆÜÝÛ[ÛÝ™\Z\ˆ9o%yaiNˆ3¥IJËŒ™ˆ8¡¤8¦!H9..ú) y§iy®¤‰Ë—Ü™\Z\‹Z—ÜÛ[ÛÝ
NÂˆœš[Š	È™\Z\ˆ9d#ˆZYÚš[Û9.ãˆ	Y8¡¤ˆ	Y‰Ë‹‹‚ˆÝYÙ\ÞÌŸKšZYÚš[Û][ÛœËÝYÙ\ÞÌßKšZYÚš[Û][ÛœÊNÂˆœš[Š	È9cëú ïy§.¹b-Žˆ9/ë¹i#y§ä:f§9è£y¥í¹¢¢¹à®y£ª9aiynî¹ëdzjæ9n©¹.éy."Î×‰ÊNÂˆœš[Š	È9§*ùl/¹nlù®äyl!ˆˆ9gd9¨!ú/æù. 9«iy."ù¢âW‰ÊNÂ™[™šYˆÝYÙWÙ]š[\›˜[ÜÙX\˜ÚØÛÜÝˆ—Ü™\Z\ˆ
ˆKBˆœš[Š	×ˆ9a¡z`ê9¤'9í(¹.èù.íÈ
	KŒ™ŠHˆ9§ 9îâˆ
	KŒ™ŠN—‰Ë‹‹‚ˆÝYÙWÙ]š[\›˜[ÜÙX\˜ÚØÛÜÝ—Ü™\Z\ŠNÂˆœš[Š	È8¡¤ˆ9më¹`/	KŒ™ˆH9¤'9í(¹kï9d$yïfºhny .úaã×‰Ë‹‹‚ˆÝYÙWÙ]š[\›˜[ÜÙX\˜ÚØÛÜÝH—Ü™\Z\ŠNÂˆœš[Š	È
Û[ÛÝ[˜[H
È™ž”[˜[H
ÈØœÔ[˜[H
ÈXYÚ[™[˜[JW‰ÊNÂˆœš[Š	È9¤'9í(ºf-¹«­ykîz-ëùo¡9oh¹â­¹§"z/ ùo.¹kï9d$NÈ9§ 9îâˆ9.#¹¤'9í(¹æë¹¨!ùmìº)èú )—‰ÊNÂ™[™‚šYˆ—Ø[›ÛX[Ý\Èˆˆœš[Š	×ˆ9o ¹n.9«­H
	Y9.*ŠN—‰Ë—Ø[›ÛX[Ý\ÊNÂˆ›ÜˆÈH[›ÛX[WÜÙYÜÂˆœš[Š	È9«­H	LÙˆIKŒYœÈ[IKˆ
ZYÚIK‹Ý]XÏIK‹[IKŠW‰Ë‹‹‚ˆË\œŠÊKÙY×ÙXYÊÊK”[—ÝÝ[‹‹‚ˆÙY×ÙXYÊÊK”[—ÚZYÚÙY×ÙXYÊÊK”[—ÜÝ]XËÙY×ÙXYÊÊK”[—Ù[ŠNÂˆHH]
ËŠNÈˆH]
ÊÌKŠNÂˆœš[Š	È	YVÉKŒY‹	KŒY‹	KŒY—H8¡¤ˆ	YVÉKŒY‹	KŒY‹	KŒY—W‰Ë‹‹‚ˆËJJKJŠKJÊKÊÌKŠJKŠŠKŠÊJNÂˆ[™™[™‚™œš[Š	×¹fïº(j9mì¹/çykf—‰ÊNÂ™œš[Š	ÈXY×ÜYWÉ\×Ý	Yœ™×‰ËPQ×ÐÒUKPQ×ÕÔÕT•
NÂ™œš[Š	ÈXY×ÜÙYÚX]É\×Ý	Yœ™×‰ËPQ×ÐÒUKPQ×ÕÔÕT•
NÂ™œš[Š	ÈXY×Ü]É\×Ý	Yœ™×‰ËPQ×ÐÒUKPQ×ÕÔÕT•
NÂ™œš[Š	ÈXY×Ü[Œ—É\×Ý	Yœ™×‰ËPQ×ÐÒUKPQ×ÕÔÕT•
NÂ™œš[Š	ú+â¹¥«yk£9¢$—‰ÊNÂ‚‰IHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOB‰IH:/¡ybªyaïy¥lNˆ]˜[X]T]ÜÙYÓ]™[‰IH9kîymì¹§"z-ëùo¡9`f¹. 9«(yn)º`$9«­z+é¹îáº+â¹¥«yæ¡:+á9/,‰IH9.#y/ë¹¥.H[šYšYYÛÜÝ[Ù[È9âë9êâú+¨yë¥ù«­yî©È[˜[H9b!º)èÈ
È‘–‹úf§9è£y£©z/äyn©‚‰IHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOB™[˜Ý[ÛˆÒ—ÝÝ[ÙY×ÙXY×HH]˜[X]T]ÜÙYÓ]™[
ÛÜÝ[Ù[[‹]ËÜÝ\\Ô^[ØY
B‰H]˜[X]T]ÜÙYÓ]™[8 %:`$9«­z+â¹¥«z+á9/,‰B‰H9i#yå*[šYšYYÛÜÝ[Ù[9æ¡9.èù.íùak9o#Ë:h§yi%º/¤ùaîŽ‚‰HÙY×ÙXYÊÊK”—Üš\ÚÈ9«­ya¡zhãºfjyéëùb!‚‰HÙY×ÙXYÊÊK”[—ÝÝ[9«­y .ù êyïf‚‰HÙY×ÙXYÊÊK”[—ÚZYÚ9«­zjæ9n©¹ êyïf‚‰HÙY×ÙXYÊÊK”[—ÜÝ]XÈ9«­zgfy  yè¬9¤§¹ êyïf‚‰HÙY×ÙXYÊÊK”[—Ù[ˆ9«­ybª9  yè¬9¤§¹ êyïf‚‰HÙY×ÙXYÊÊK›™ž—Ü›Þ[Z]WÙ›YÈ9¦+ùd)¹£©z/äH‘–‚‰HÙY×ÙXYÊÊK›™ž—ÛZ[—Ù\Ý9b,9§ :/äy­.ú-àÈ‘–ˆ9æ¡:-çyé®È
JB‰HÙY×ÙXYÊÊK›Øœ×Ü›Þ[Z]WÙ›YÈ9¦+ùd)¹£©z/äybª9  zf§9è£B‰HÙY×ÙXYÊÊK›Øœ×ÛZ[—Ù\Ý9b,9§ :/äzf§9è£yæ¡:-çyé®È
JB‰HÙY×ÙXYÊÊKšZYÚØ™[ÝÈ9/c¹.£¹g,9§oùi&¹l$yìlÈ
yd":)á
B‰HÙY×ÙXYÊÊKœÝ]X×Ú]9¦+ùd)¹êoú-¢¹nî¹ëdB‚ˆYˆ˜\™Ú[ˆK\Ô^[ØYHYNÈ[™ˆYˆ˜\™Ú[ˆÜÝ\HÈ[™‚ˆˆHÚ^™J]ËJNÂ‚ˆ	H9âjyä!¹n.:aãÈ
9.#ˆ[šYšYYÛÜÝ[Ù[9k£9aj9. :!í
BˆYˆ\Ô^[ØYˆWÝÝ[HÛÜÝ[Ù[›WÙœ˜[YH
ÈÛÜÝ[Ù[›WÜ^[ØYÂˆ[ÙBˆWÝÝ[HÛÜÝ[Ù[›WÙœ˜[YNÂˆ[™ˆÈHWÝÝ[
ˆKŽNÂˆ]HHÛÜÝ[Ù[™]WÛ[ÝÜˆ
ˆÛÜÝ[Ù[™]WÜ›ÜÂˆWÙ\ØÈHÛÜÝ[Ù[›—Ü›ÝÜˆ
ˆH
ˆÛÜÝ[Ù[œ—Ü›ÜŒŽÂˆ—ÚWÚÝ™\ˆHÜ\
ÈÈ
ˆ
ˆÛÜÝ[Ù[œš×ØZ\ˆ
ˆWÙ\ØÊJNÂˆÚÝ™\ˆH×ŒKHÈÜ\
Š˜ÛÜÝ[Ù[œš×ØZ\ŠWÙ\ØÊHÈ]NÂ‚ˆÛZ[ˆHÛÜÝ[Ù[’ÛZ[ŽÂˆÛX^HÛÜÝ[Ù[’ÛX^ÂˆØÛX\ˆHÛÜÝ[Ù[’ØÛX\˜[˜ÙNÂˆTÈH[‹“PTÔÒV‘NÂ‚ˆ‘–—ÓPT‘ÒSˆHKNÈ	H9£©z/äyn©º+i¹¢$¹`#y¥lˆÐ”×ÓPT‘ÒSˆH‹NÂ‚ˆ™ž“\ÝH[‹™[“ØœÝXÛ\Ë[\‘–ŽÂˆØœÓ\ÝH[‹™[“ØœÝXÛ\Ë›[Ýš[™ÓØœÎÂ‚ˆÕP—ÔÔPÒS‘ÈHLŽÂˆRS—ÔÕPˆHÎÂ‚ˆ	H:h¡9b!ºacBˆÙY×ÙXYÈHÝXÝ
‹‹‚ˆ	Ô—Üš\ÚÉË[L˜Ù[
™\›ÜÊ‹LKJJK‹‹‚ˆ	Ô[—ÝÝ[	Ë[L˜Ù[
™\›ÜÊ‹LKJJK‹‹‚ˆ	Ô[—ÚZYÚ	Ë[L˜Ù[
™\›ÜÊ‹LKJJK‹‹‚ˆ	Ô[—ÜÝ]XÉË[L˜Ù[
™\›ÜÊ‹LKJJK‹‹‚ˆ	Ô[—Ù[‰Ë[L˜Ù[
™\›ÜÊ‹LKJJK‹‹‚ˆ	Û™ž—Ü›Þ[Z]WÙ›YÉË[L˜Ù[
˜[ÙJ‹LKJJK‹‹‚ˆ	Û™ž—ÛZ[—Ù\Ý	Ë[L˜Ù[
[™Š‹LKJJK‹‹‚ˆ	ÛØœ×Ü›Þ[Z]WÙ›YÉË[L˜Ù[
˜[ÙJ‹LKJJK‹‹‚ˆ	ÛØœ×ÛZ[—Ù\Ý	Ë[L˜Ù[
[™Š‹LKJJK‹‹‚ˆ	ÚZYÚØ™[ÝÉË[L˜Ù[
™\›ÜÊ‹LKJJK‹‹‚ˆ	ÜÝ]X×Ú]	Ë[L˜Ù[
˜[ÙJ‹LKJJJNÂˆÙY×ÙXYÈHÙY×ÙXYÊŠNÂ‚ˆØÝ\œ™[HÜÝ\ÂˆWÝÝ[HÈÝÝ[HÈ[—ÝÝ[HÈ—ÝÝ[HÈ×ÝÝ[HÂ‚ˆ›ÜˆÈHN“‹LBˆHH]ÊËŠNÂˆˆH]ÊÊÌKŠNÂ‚ˆHŠJK\JJNÈHHŠŠK\JŠNÈˆHŠÊK\JÊNÂˆÚÜš^ˆHÜ\
ŒŠÙWŒŠNÂˆÌÙHÜ\
ŒŠÙWŒŠÙ—ŒŠNÂˆYˆÌÙŒKÛÛ[YNÈ[™‚ˆYˆÚÜš^ˆˆŒBˆ\—ÚHÙWKÙÚÜš^ŽÂˆ[ÙBˆ\—ÚHÌNÂˆ[™ˆØ[[XHH][ŒŠ‹ÚÜš^ŠNÂˆ—ÚÜš^ˆHÛÜÝ[Ù[—ØÜZ\ÙH
ˆÛÜÊØ[[XJNÂˆ—Ý™\HÛÜÝ[Ù[—ØÜZ\ÙH
ˆÚ[ŠØ[[XJNÂ‚ˆ”ÝXˆHX^
RS—ÔÕP‹ÙZ[
ÌÙÔÕP—ÔÔPÒS‘ÊJNÂˆÜÝXˆHÌÙÈ”ÝXŽÂ‚ˆWØXØÏLÈØXØÏLÈ—ØXØÏLÈLÈÏLÈLÈØXØÏLÂˆØ™[Ý×ÛX^HÂˆÝ]X×Ú]H˜[ÙNÂˆ™ž—ÛZ[—ÙH[™ŽÂˆØœ×ÛZ[—ÙH[™ŽÂˆÜÝXˆHØÝ\œ™[Â‚ˆ›ÜˆÈHN›”ÝX‚ˆœ˜X×ÛZYH
ËLJKÛ”ÝXŽÂˆÜÝXˆHH
Èœ˜X×ÛZY
Š‹\JNÂ‚ˆ	H:hã¹g.‚ˆÚ[™Ý™XÈHÌNÂˆYˆš\Ù[\JÛÜÝ[Ù[Ú[™šY[
BˆžBˆÚ[™Ý™XÈHÛÜÝ[Ù[Ú[™šY[™Ù]Ú[™
ÜÝXŠJKÜÝXŠŠKÜÝXŠÊKÜÝXŠNÂˆØ]ÚÈ[™ˆ[™ˆ—ÝØHHÝ
Ú[™Ý™XÊNŒŠK\—Ú
NÂˆ—ÝØÈH›Ü›JÚ[™Ý™XÊNŒŠHH—ÝØJ™\—Ú
NÂˆ—ÝÝˆHÚ[™Ý™XÊÊNÂ‚ˆ—ØZH—ÚÜš^ˆH—ÝØNÂˆ—Ø]ˆH—Ý™\H—ÝÝŽÂˆ—ØZ\ˆHÜ\
—ØZŒˆ
È—ÝØ×Œˆ
È—Ø]—ŒŠNÈ—ØZ\ˆHX^
—ØZ\‹JNÂˆ—ÙÜ›Ý[™HÜ\

—ÚÜš^ŠÝ—ÝØJWŒˆ
È—ÝØ×ŒŠNÈ—ÙÜ›Ý[™HX^
—ÙÜ›Ý[™JNÂ‚ˆ]HHX^
—ØZ
KÊ—ÚWÚÝ™\ŠÌŒJNÂˆYˆ]OŒK—ÚO]—ÚWÚÝ™\ŠŠK[]WŒ‹Í
NÂˆ[ÙK—ÚO]—ÚWÚÝ™\—Œ‹ÊŠ›X^
XœÊ—ØZ
KJJNÈ[™ˆÚ[™HÊ—ÚKÙ]NÂˆÜ\ˆHJ˜ÛÜÝ[Ù[œš×ØZ\Š˜ÛÜÝ[Ù[×Ù
˜ÛÜÝ[Ù[WØ›ÙJ—ØZ\—ŒËÙ]NÂˆÜ›ÈHŒMJ”ÚÝ™\ŽÂˆYˆ—Ø]ŒØÛUÊ—Ø]‹ØÛÜÝ[Ù[˜Û[X—ÙY™šXÚY[˜ÞKÙ]NÂˆ[ÙKØÛUÊ—Ø]Š˜ÛÜÝ[Ù[™\ØÙ[Ü™XÛÝ™\žNÈ[™ˆYˆ—ÝØÏŒBˆ[X][ŒŠ—ÝØË—ÚWÚÝ™\ŠŒÊNÈØÜTÚÝ™\ŠŠKØÛÜÊ[
KLJNÂˆ[ÙKØÜLÈ[™ˆÝÝHX^
Ú[™
ÔÜ\ŠÔÜ›ÊÔØÛ
ÔØÜ‹ÚÝ™\ŠŒŒÊNÂ‚ˆÜÝXˆHÜÝX‹Ý—ÙÜ›Ý[™ÂˆWÜÝXˆHÝÝ
™ÜÝX‹ÌÍŒÂ‚ˆ	H9bª9  zhãºfjH
È9è¬9¤§‚ˆYˆš\Ù[\JÛÜÝ[Ù[™[“ØœÝXÛ\ÊBˆÜš\ÚÈHÜÝXˆ
ÈÜÝX‹ÌŽÂˆ—ÜÝXˆHÛÜÝ[Ù[™]˜[X]Tš\ÚÐ]Ú[
ÜÝX‹Üš\ÚÊNÂˆ—ØXØÈH—ØXØÈ
È—ÜÝXŠ™ÜÝX‹ÍŒÂˆYˆÛÜÝ[Ù[˜ÚXÚÐÛÛ\Ú[ÛŠÜÝX‹Üš\ÚÊBˆ[ˆHKÛ”ÝXŽÈØXØÏTØXØÊÜ[ŽÈT
Ü[ŽÂˆ[™ˆ[™‚ˆ	H:gfy  yè¬9¤§‚ˆYˆš\Ù[\JÛÜÝ[Ù[œÝ]XÓX\
Bˆž[X^
KZ[ŠÚ^™JÛÜÝ[Ù[œÝ]XÓX\JK›Ý[™
ÜÝXŠJJJJNÂˆžO[X^
KZ[ŠÚ^™JÛÜÝ[Ù[œÝ]XÓX\ŠK›Ý[™
ÜÝXŠŠJJJNÂˆYˆÜÝXŠÊHÛÜÝ[Ù[œÝ]XÓX\
žžJJÌÂˆ[LKÛ”ÝXŽÈØXØÏTØXØÊÜ[ŽÈÏTÊÜ[ŽÂˆÝ]X×Ú]]YNÂˆ[™ˆ[™‚ˆ	H:jæ9n©¹î©¹§gÂˆYˆš\Ù[\JÛÜÝ[Ù[œÝ]XÓX\
Bˆž[X^
KZ[ŠÚ^™JÛÜÝ[Ù[œÝ]XÓX\JK›Ý[™
ÜÝXŠJJJJNÂˆžO[X^
KZ[ŠÚ^™JÛÜÝ[Ù[œÝ]XÓX\ŠK›Ý[™
ÜÝXŠŠJJJNÂˆÚXÛÜÝ[Ù[œÝ]XÓX\
žžJNÂˆ[ÙKÚLÈ[™ˆYˆHX^
ÛZ[‹Ú
ÒØÛX\ŠNÂˆYˆÜÝXŠÊOY‚ˆ[JY‹\ÜÝXŠÊJKÌLÛ”ÝXŽÈØXØÏTØXØÊÜ[ŽÈT
Ü[ŽÂˆØ™[Ý×ÛX^[X^
Ø™[Ý×ÛX^Y‹\ÜÝXŠÊJNÂˆ[™ˆYˆÜÝXŠÊO’ÛX^ˆ[JÜÝXŠÊKRÛX^
KÌLÛ”ÝXŽÈØXØÏTØXØÊÜ[ŽÈT
Ü[ŽÂˆ[™‚ˆ	H‘–ˆ9£©z/äyn©‚ˆ›ÜˆšOLN›[™Ý
™ž“\Ý
Bˆ™ž[™ž“\Ý
šJNÂˆYˆ›™ž‹˜XÝ]™_ÜÝX™ž‹ÜÝ\ÜÝX›™ž‹Ù[™ÛÛ[YNÙ[™ˆ[›Ü›JÜÝXŠNŒŠK[™ž‹˜Ù[\ŠNÂˆ™ž—ÛZ[—Ù[Z[Š™ž—ÛZ[—Ù
NÂˆ[™‚ˆ	H:f§9è£y£©z/äyn©‚ˆ›ÜˆÚOLN›[™Ý
ØœÓ\Ý
BˆÜY[‹™[“ØœÝXÛ\Ë™Ù]ÜÚ][ÛŠÚKÜÝXŠÙÜÝX‹ÌŠNÂˆÛØœÏ[›Ü›JÜÝX‹[Ü
NÂˆØœ×ÛZ[—Ù[Z[ŠØœ×ÛZ[—ÙÛØœÊNÂˆ[™‚ˆWØXØÏQWØXØÊÑWÜÝXŽÈØXØÏUØXØÊÙÜÝXŽÈÜÝX]ÜÝXŠÙÜÝXŽÂˆ[™‚ˆ	H9êëùà®zjæ9n©¹¨à9§éHH
9.#ˆ[šYšYYÛÜÝ[Ù[9/ë¹i#yd#¹. :!í
BˆYˆš\Ù[\JÛÜÝ[Ù[œÝ]XÓX\
Bˆ[X^
KZ[ŠTË›Ý[™
JJJJJNÈO[X^
KZ[ŠTË›Ý[™
JŠJJJNÂˆÚXÛÜÝ[Ù[œÝ]XÓX\
JNÂˆ[ÙKÚLÈ[™ˆY[X^
ÛZ[‹Ú
ÒØÛX\ŠNÂˆYˆJÊOY‚ˆ[JY‹\JÊJKÌLÈØXØÏTØXØÊÜ[ŽÈT
Ü[ŽÂˆØ™[Ý×ÛX^[X^
Ø™[Ý×ÛX^Y‹\JÊJNÂˆ[™ˆYˆJÊO’ÛX^ˆ[JJÊKRÛX^
KÌLÈØXØÏTØXØÊÜ[ŽÈT
Ü[ŽÂˆ[™‚ˆ×ÝÝ[H×ÝÝ[
ÈXœÊŠJŒŒNÂˆWÝÝ[HWÝÝ[
ÈWØXØÎÂˆÝÝ[HÝÝ[
ÈØXØÎÂˆ—ÝÝ[H—ÝÝ[
È—ØXØÎÂˆ[—ÝÝ[H[—ÝÝ[
ÈØXØÎÂ‚ˆ	H9a¦yaiy«­z+â¹¥«BˆÙY×ÙXYÊÊK”—Üš\ÚÈH—ØXØÎÂˆÙY×ÙXYÊÊK”[—ÝÝ[HØXØÎÂˆÙY×ÙXYÊÊK”[—ÚZYÚHÂˆÙY×ÙXYÊÊK”[—ÜÝ]XÈHÎÂˆÙY×ÙXYÊÊK”[—Ù[ˆHÂˆYˆ\Ù[\J™ž“\Ý
BˆÙY×ÙXYÊÊK›™ž—Ü›Þ[Z]WÙ›YÈH˜[ÙNÂˆ[ÙBˆÙY×ÙXYÊÊK›™ž—Ü›Þ[Z]WÙ›YÈH
\Ùš[š]J™ž—ÛZ[—Ù
H	‰ˆ‹‹‚ˆ™ž—ÛZ[—ÙZ[ŠÛ™ž“\Ýœ˜Y]\×JJ“‘–—ÓPT‘ÒSŠNÂˆ[™ˆÙY×ÙXYÊÊK›™ž—ÛZ[—Ù\ÝH™ž—ÛZ[—ÙÂˆYˆ\Ù[\JØœÓ\Ý
BˆÙY×ÙXYÊÊK›Øœ×Ü›Þ[Z]WÙ›YÈH˜[ÙNÂˆ[ÙBˆÙY×ÙXYÊÊK›Øœ×ÛZ[—Ù\ÝHØœ×ÛZ[—ÙÂˆÙY×ÙXYÊÊK›Øœ×Ü›Þ[Z]WÙ›YÈH
\Ùš[š]JØœ×ÛZ[—Ù
H	‰ˆ‹‹‚ˆØœ×ÛZ[—ÙZ[ŠÛØœÓ\Ýœ˜Y]\×JJ“Ð”×ÓPT‘ÒSŠNÂˆ[™ˆÙY×ÙXYÊÊKšZYÚØ™[ÝÈHØ™[Ý×ÛX^ÂˆÙY×ÙXYÊÊKœÝ]X×Ú]HÝ]X×Ú]Â‚ˆØÝ\œ™[HÜÝXŽÂˆ[™‚ˆ	H9§*ùl/¹¨à9§éy§ 9d#¹. 9à®BˆÛ\ÝH]Ê‹ŠNÂˆYˆš\Ù[\JÛÜÝ[Ù[œÝ]XÓX\
Bˆ[X^
KZ[ŠTË›Ý[™
Û\Ý
JJJJNÈO[X^
KZ[ŠTË›Ý[™
Û\Ý
ŠJJJNÂˆÚXÛÜÝ[Ù[œÝ]XÓX\
JNÂˆ[ÙKÚLÈ[™ˆY[X^
ÛZ[‹Ú
ÒØÛX\ŠNÂˆÛ\ÝLÂˆYˆÛ\Ý
ÊOY‹Û\ÝTÛ\Ý
ÊY‹\Û\Ý
ÊJKÌLÈ[™ˆYˆÛ\Ý
ÊO’ÛX^Û\ÝTÛ\Ý
ÊÛ\Ý
ÊKRÛX^
KÌLÈ[™ˆ[—ÝÝ[H[—ÝÝ[
ÈÛ\ÝÂ‚ˆ	H9å-y¬h9 êyïf‚ˆ[—Ø˜]HÂˆYˆWÝÝ[ˆÛÜÝ[Ù[‘WØ˜]
ŒŽBˆ[—Ø˜]H
WÝÝ[HÛÜÝ[Ù[‘WØ˜]
ŒŽJKÌLÂˆ[—ÝÝ[H[—ÝÝ[
È[—Ø˜]Âˆ[™‚ˆ—ÝÝ[HÛÜÝ[Ù[×Ù[™\™ÞJ‘WÝÝ[
È‹‹‚ˆÛÜÝ[Ù[×Ý[YJŠÝÝ[ÍŒ
H
È‹‹‚ˆÛÜÝ[Ù[×ØÛ[XŠ×ÝÝ[
È‹‹‚ˆÛÜÝ[Ù[×Üš\ÚÊ”—ÝÝ[
È‹‹‚ˆÛÜÝ[Ù[›[X™WÜ[˜[J”[—ÝÝ[Â™[™