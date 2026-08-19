function [path, cost, details, stage_details] = runRA_ALA(...
        planner, costModel, env, start, goal, t_start, hasPayload, cfg)
% =========================================================================
% runRA_ALA v3.1 — 风险感知能耗优化 ALA 三维路径规划 (诊断分解版)
%
%  输出:
%    path          [N×3]  最终路径（repair后）
%    cost          标量   最终代价（统一 evaluatePath 计算）
%    details       struct 最终路径的完整代价分解
%    stage_details struct 三阶段分解:
%                           .raw    — raw path 的 evaluatePath 结果
%                           .smooth — smooth path 的 evaluatePath 结果
%                           .repair — repair path 的 evaluatePath 结果
%
%  接口兼容: [path, cost, details] = runRA_ALA(...)  仍然有效
%
%  统一评估口径 (目标2):
%    搜索阶段适应度 evaluateRAALASearchFitness 仍用于比较候选优劣 (含 NFZ/smooth 惩罚).
%    阶段二的最终 J 统一通过 costModel.evaluatePath 重新计算,
%    不把 evaluateRAALASearchFitness 的内部值直接当最终输出.
% =========================================================================
    % >>>>> TIMING AND COUNTING INSTRUMENTATION >>>>>
    t_total = tic;
    t_initialization = tic;
    timing = struct('initialization_s', 0, 'warm_start_s', 0, ...
        'search_s', 0, 'main_optimization_s', 0, ...
        'topk_s', 0, 'topk_generation_s', 0, 'smoothing_s', 0, ...
        'topk_evaluation_s', 0, 'topk_selection_s', 0, ...
        'topk_overhead_s', 0, ...
        'rescueA_s', 0, 'rescueB_s', 0, 'rescue_total_s', 0, ...
        'other_s', 0, 'total_s', 0);

    candidate_stats = struct('topk_parent_count', 0, ...
        'raw_generated', 0, 'smooth_generated', 0, ...
        'mild_generated', 0, 'topk_candidates_evaluated', 0);

    rescueA_stats = struct('triggered', false, 'executed', false, ...
        'initial_conflict_segments', 0, 'diagnostic_evaluations', 0, ...
        'candidates_generated', 0, 'candidates_evaluated', 0, ...
        'final_evaluations', 0, 'successful_insertions', 0, ...
        'adopted', false);

    rescueB_stats = struct('triggered', false, 'executed', false, ...
        'diagnostic_evaluations', 0, 'candidate_evaluations', 0, ...
        'adopted', false);
    rescue_a_count = 0;
    rescue_b_count = 0;
    % <<<<< TIMING AND COUNTING INSTRUMENTATION END <<<<<

    start = start(:)'; goal = goal(:)';
    if length(start)<3, start=[start,60]; end
    if length(goal)<3,  goal=[goal,60];   end

    nWP     = cfg.nWaypoints;
    popSize = cfg.popSize;
    maxIter = cfg.maxIter;
    minH    = planner.minH;
    maxH    = planner.maxH;

    dirVec    = goal(1:2) - start(1:2);
    totalDist = norm(dirVec);
    dirUnit   = dirVec / max(totalDist, 1);
    perpUnit  = [-dirUnit(2), dirUnit(1)];

    dim    = nWP * 2;
    maxLat = 200;
    lb = repmat([-maxLat, minH], 1, nWP);
    ub = repmat([maxLat,  maxH], 1, nWP);

    % ─────────── v9: 环境硬度自适应缩放注入 ───────────
    % 一次估算, 在整次 runRA_ALA 期间共用. 写入 cfg.difficultyScale 后
    % evaluateRAALASearchFitness 内部的 NFZ/obs/headwind 罚项会自动按此因子缩放.
    if ~(isfield(cfg,'difficultyScale') && ~isempty(cfg.difficultyScale))
        % 仅在外部未显式指定时才自动估算 (允许调用方覆盖)
        [diffScaleAuto, diffInfo] = estimateEnvDifficulty(env, start, goal);
        cfg.difficultyScale = diffScaleAuto;
        cfg.difficultyInfo  = diffInfo;
    end

    param2path = @(x) paramToPath(x, start, goal, nWP, dirUnit, perpUnit, totalDist, env, minH);
    evalFcn    = @(x) evaluateRAALASearchFitness(x, param2path, t_start, hasPayload, costModel, env, cfg);

    % ---- 初始化种群 ----
    pop = zeros(popSize, dim);
    for j = 1:nWP
        pop(1,(j-1)*2+1) = 0;
        pop(1,(j-1)*2+2) = start(3);
    end
    try
        initPath = planner.greedyPath(start, goal, t_start);
        if size(initPath,1) >= nWP + 2
            idx = round(linspace(2, size(initPath,1)-1, nWP));
            for j = 1:nWP
                pt    = initPath(idx(j), 1:2);
                baseXY = start(1:2) + (j/(nWP+1)) * dirVec;
                latOff = dot(pt - baseXY, perpUnit);
                pop(2,(j-1)*2+1) = max(-maxLat, min(maxLat, latOff));
                pop(2,(j-1)*2+2) = max(minH, min(maxH, initPath(idx(j), 3)));
            end
        end
    catch; end
    % ★ 目标1: 增加高度感知安全候选
    % 候选3: 飞行在较高高度层 (避开大多数建筑)
    safeH3 = min(maxH, max(minH, minH + 0.6*(maxH-minH)));  % 60%高度层
    for j=1:nWP, pop(3,(j-1)*2+1)=0; pop(3,(j-1)*2+2)=safeH3; end
    % 候选4: 直线路径 + 中等高度
    safeH4 = min(maxH, max(minH, minH + 0.4*(maxH-minH)));  % 40%高度层
    for j=1:nWP, pop(min(4,popSize),(j-1)*2+1)=0; pop(min(4,popSize),(j-1)*2+2)=safeH4; end
    % 候选5: 沿途取 heightMap 最大值再加净空 (紧贴建筑顶)
    if popSize >= 5
        for j=1:nWP
            frac=j/(nWP+1);
            ptXY=start(1:2)+frac*dirVec;
            ptXY(1)=max(1,min(env.MAP_SIZE,ptXY(1))); ptXY(2)=max(1,min(env.MAP_SIZE,ptXY(2)));
            ex=max(1,min(env.MAP_SIZE,round(ptXY(1)))); ey=max(1,min(env.MAP_SIZE,round(ptXY(2))));
            localH=env.heightMap(ex,ey)+costModel.H_clearance+5;
            pop(5,(j-1)*2+1)=0;
            pop(5,(j-1)*2+2)=max(minH,min(maxH,localH));
        end
    end
    for i = max(7,3):popSize   % 从 7 开始，保留 slot 6 给 EA* 热启动种子
        for j = 1:nWP
            pop(i,(j-1)*2+1) = (rand-0.5)*120;
            pop(i,(j-1)*2+2) = minH + rand*(maxH-minH);
        end
    end
    pop = max(lb, min(ub, pop));

    % ── EA* 热启动种子（放在随机初始化之后，确保不被覆盖）──
    % 当前种群结构:
    %   C1: 直线路径（z=start_z）
    %   C2: 贪心路径投影
    %   C3: 高度层 84m（60%层）
    %   C4: 高度层 66m（40%层）
    %   C5: 紧贴建筑顶
    %   C6: ★ EA* 路径热启动（可行保障种子）← 此处写入，不会被覆盖
    %   C7~C30: 随机初始化
    %
    % 为什么能保证可行性:
    %   C6 是可行个体 → isBetter 规则下可行优先
    %   → ALA 迭代永远不会用不可行个体替换 C6
    %   → 种群始终保有至少一条可行路径
    %   → 最终输出是 ALA 从 C6 出发搜索优化的结果，不是 EA* 路径本身
    ea_seed_ok = false;
    t_warm_start = tic;
    if popSize >= 6
        try
            [path_ea_seed, ~, ~] = planner.energyAStar(start, goal, t_start, hasPayload);
            [~, det_ea_seed] = costModel.evaluatePath(path_ea_seed, t_start, hasPayload);
            if det_ea_seed.feasible && size(path_ea_seed,1) >= nWP+2
                ea_idx = round(linspace(2, size(path_ea_seed,1)-1, nWP));
                x_ea   = zeros(1, dim);
                for j = 1:nWP
                    ptXY   = path_ea_seed(ea_idx(j), 1:2);
                    frac   = j / (nWP+1);
                    baseXY = start(1:2) + frac*(goal(1:2)-start(1:2));
                    latOff = dot(ptXY - baseXY, perpUnit);
                    alt    = path_ea_seed(ea_idx(j), 3);
                    x_ea((j-1)*2+1) = max(lb(1), min(ub(1), latOff));
                    x_ea((j-1)*2+2) = max(minH,  min(maxH,  alt));
                end
                pop(6, :)  = max(lb, min(ub, x_ea));   % 写入 slot 6
                ea_seed_ok = true;
            end
        catch; end
    end
    timing.warm_start_s = toc(t_warm_start);
    if ~ea_seed_ok
        % EA* 失败时 slot 6 保持随机值（与原来相同，无退化）
        for j = 1:nWP
            pop(6,(j-1)*2+1) = (rand-0.5)*120;
            pop(6,(j-1)*2+2) = minH + rand*(maxH-minH);
        end
        pop(6,:) = max(lb, min(ub, pop(6,:)));
    end

    fitness = zeros(popSize, 1);
    for i = 1:popSize, fitness(i) = evalFcn(pop(i,:)); end
    [bestFit, bestIdx] = min(fitness);
    bestPos = pop(bestIdx, :);

    % ---- 实时显示：建立图形窗口 ----
    if exist('SHOW_LIVE','var') && SHOW_LIVE
        % 收敛曲线窗口
        if exist('SHOW_CONV_CURVE','var') && SHOW_CONV_CURVE
            fig_conv = figure('Name','RA-ALA 收敛曲线（实时）', ...
                'NumberTitle','off','Position',[50 500 520 280]);
            ax_conv = axes('Parent',fig_conv);
            title(ax_conv, sprintf('RA-ALA 搜索收敛曲线  t_0=%ds', round(t_start)), ...
                'FontSize',11,'FontWeight','bold');
            xlabel(ax_conv,'迭代次数'); ylabel(ax_conv,'最优适应度 (内部口径)');
            hold(ax_conv,'on'); grid(ax_conv,'on');
            h_line = plot(ax_conv, nan, nan, '-', 'Color','#C0392B', 'LineWidth',2);
            drawnow;
        end
        % 路径预览窗口
        if exist('SHOW_PATH_LIVE','var') && SHOW_PATH_LIVE
            fig_path = figure('Name','RA-ALA 路径实时预览', ...
                'NumberTitle','off','Position',[600 500 500 450]);
            ax_path = axes('Parent',fig_path);
            axis(ax_path,'equal'); hold(ax_path,'on'); grid(ax_path,'on');
            xlabel(ax_path,'X (m)'); ylabel(ax_path,'Y (m)');
            title(ax_path, sprintf('当前最优路径预览  t_0=%ds', round(t_start)), ...
                'FontSize',11,'FontWeight','bold');
            % 画建筑底图
            if ~isempty(env.buildings)
                for bi_ = 1:size(env.buildings,1)
                    cx_=env.buildings(bi_,1); cy_=env.buildings(bi_,2);
                    hw_=env.buildings(bi_,4); hh_=env.buildings(bi_,5);
                    bh_=env.buildings(bi_,3); gv_=max(0.4,0.88-bh_/200);
                    rectangle('Parent',ax_path,'Position',[cx_-hw_,cy_-hh_,2*hw_,2*hh_], ...
                        'FaceColor',[gv_ gv_ gv_ 0.7],'EdgeColor',[0.5 0.5 0.5],'LineWidth',0.3);
                end
            end
            plot(ax_path, start(1), start(2), 'p', 'MarkerSize',13, ...
                'MarkerFaceColor','#2ECC71','MarkerEdgeColor','k','LineWidth',1.2);
            plot(ax_path, goal(1), goal(2), 'h', 'MarkerSize',13, ...
                'MarkerFaceColor','#E74C3C','MarkerEdgeColor','k','LineWidth',1.2);
            xlim(ax_path,[0 env.MAP_SIZE]); ylim(ax_path,[0 env.MAP_SIZE]);
            h_path = plot(ax_path, nan, nan, 'b-', 'LineWidth', 2.5);
            drawnow;
        end
    end
    conv_hist = zeros(maxIter,1);  % 收敛历史

    timing.initialization_s = toc(t_initialization);

    % ---- ALA 主迭代 ----
    % >>>>> RUNTIME_ANALYSIS PATCH 2b (search timing) >>>>>
    t_search = tic;
    % <<<<<
    for iter = 1:maxIter
        theta       = 2 * atan(1 - iter/maxIter);
        sigma_decay = 1 - 0.6 * iter/maxIter;
        for i = 1:popSize
            E  = 2 * log(1/rand) * theta;
            r1 = rand;
            if r1 < 0.3
                newPos = pop(i,:) + E * (bestPos - pop(i,:));
            elseif r1 < 0.55
                noise  = randn(1,dim).*(ub-lb)*0.08*sigma_decay;
                newPos = pop(i,:) + E*noise;
            elseif r1 < 0.8
                l      = rand*2-1;
                newPos = bestPos + E*exp(l)*cos(2*pi*l)*(pop(i,:)-bestPos)*sigma_decay;
            else
                beta  = 1.5;
                sig_l = (gamma(1+beta)*sin(pi*beta/2)/(gamma((1+beta)/2)*beta*2^((beta-1)/2)))^(1/beta);
                u     = randn(1,dim)*sig_l;
                v     = randn(1,dim);
                step  = u./abs(v).^(1/beta).*(ub-lb)*0.025*sigma_decay;
                newPos = pop(i,:) + step;
            end
            newPos = max(lb, min(ub, newPos));
            newFit = evalFcn(newPos);
            if newFit < fitness(i)
                pop(i,:)   = newPos;
                fitness(i) = newFit;
                if newFit < bestFit, bestFit = newFit; bestPos = newPos; end
            end
        end

        % ── 实时显示更新 ──
        conv_hist(iter) = bestFit;
        if exist('SHOW_LIVE','var') && SHOW_LIVE

            % 命令行进度条
            if exist('SHOW_ITER_BAR','var') && SHOW_ITER_BAR
                pct = iter/maxIter;
                bar_len = 30;
                filled  = round(pct * bar_len);
                bar_str = [repmat('█',1,filled), repmat('░',1,bar_len-filled)];
                fprintf('\r  迭代 [%s] %3d/%d  bestJ=%-8.3f', ...
                    bar_str, iter, maxIter, bestFit);
                if iter == maxIter, fprintf('\n'); end
            end

            % 每 REFRESH_EVERY 轮刷新图形
            if exist('REFRESH_EVERY','var') && mod(iter, REFRESH_EVERY)==0
                % 更新收敛曲线
                if exist('SHOW_CONV_CURVE','var') && SHOW_CONV_CURVE && ishandle(fig_conv)
                    set(h_line, 'XData', 1:iter, 'YData', conv_hist(1:iter));
                    xlim(ax_conv, [1, maxIter]);
                    drawnow limitrate;
                end
                % 更新路径预览
                if exist('SHOW_PATH_LIVE','var') && SHOW_PATH_LIVE && ishandle(fig_path)
                    curPath = param2path(bestPos);
                    set(h_path, 'XData', curPath(:,1), 'YData', curPath(:,2));
                    title(ax_path, sprintf('迭代 %d/%d  当前最优 J=%.3f', ...
                        iter, maxIter, bestFit), 'FontSize',10,'FontWeight','bold');
                    drawnow limitrate;
                end
            end

            % 每 PRINT_ITER_EVERY 轮打印详情
            if exist('PRINT_ITER_EVERY','var') && PRINT_ITER_EVERY>0 && mod(iter,PRINT_ITER_EVERY)==0
                fprintf('  iter=%3d  bestFit=%.4f  sigma_decay=%.3f\n', ...
                    iter, bestFit, 1-0.6*iter/maxIter);
            end
        end
    end

    % =========================================================
    % 捕获搜索阶段内部最优适应度 (evaluateRAALASearchFitness 输出, 含 smooth/NFZ/
    % headwind 导向惩罚).  这是 ALA 优化器"看到"的最优值.
    % 它与最终 J 的差距 = 各类导向罚项叠加量 (不参与比较).
    % =========================================================
    % >>>>> RUNTIME_ANALYSIS PATCH 2b END (search timing) >>>>>
    timing.search_s = toc(t_search);
    timing.main_optimization_s = timing.search_s;
    % <<<<<
    internal_search_cost = bestFit;

    % ====================================================================
    % 阶段二: Top-K → 三路径生成 → 统一评估 → 择优选出最终路径
    %
    % 核心修改: 不再默认输出 repair 后路径.
    % 对每个候选生成 raw / smooth / repair 三条路径,
    % 每条路径均用 costModel.evaluatePath 统一评估,
    % 然后按以下规则择优:
    %   1. 优先选择 evaluatePath 严格判定为 feasible 的路径
    %   2. feasible 候选中取 final_J 最小者
    %   3. 全不 feasible 时, 取 penalty_total 最小 (同则取 J 最小) 者
    % ====================================================================
    % ── ablate_unifiedEval：跳过统一重评估，直接输出内部适应度最优的 raw 路径 ──
    % 这是消融实验 w/o Unified Eval 变体的实现：证明"统一评估驱动"的实质贡献。
    % 正常流程：evaluateRAALASearchFitness内部适应度最优个体 → 三路径生成 → evaluatePath统一重评估 → 择优。
    % 消融流程：evaluateRAALASearchFitness内部适应度最优个体 → 直接输出其raw路径（不经统一重评估）。
    % 若两条路径的J值差异显著，说明内部口径和报告口径不一致，"统一评估驱动"
    % 是真正改变路径质量的机制，而非口号。
    if isfield(cfg,'ablate_unifiedEval') && cfg.ablate_unifiedEval
        t_direct = tic;
        t_piece = tic;
        raw_direct = param2path(bestPos);
        timing.topk_generation_s = toc(t_piece);
        candidate_stats.raw_generated = 1;

        t_piece = tic;
        [j_direct, det_direct] = costModel.evaluatePath(raw_direct, t_start, hasPayload);
        timing.topk_evaluation_s = toc(t_piece);
        candidate_stats.topk_candidates_evaluated = 1;
        timing.topk_s = toc(t_direct);
        timing.topk_overhead_s = max(0, timing.topk_s - ...
            timing.topk_generation_s - timing.topk_evaluation_s);
        path = raw_direct;
        cost = j_direct;
        details = det_direct;
        details.final_unified_J   = j_direct;
        details.internal_search_J = bestFit;
        details.feasible          = det_direct.feasible;
        details.chosen_path_type  = 'raw(internal-only,no-reeval)';
        details.J_final           = j_direct;
        stage_details.raw_J       = j_direct;
        stage_details.smooth_J    = NaN;
        stage_details.repair_J    = NaN;
        % >>>>> RUNTIME_ANALYSIS PATCH 2e (early-return path) >>>>>
        timing.total_s = toc(t_total);
        timing.other_s = max(0, timing.total_s - timing.initialization_s - ...
            timing.search_s - timing.topk_s);
        details.timing          = timing;
        details.candidate_stats = candidate_stats;
        details.rescueA_stats   = rescueA_stats;
        details.rescueB_stats   = rescueB_stats;
        details.rescue_a_count  = 0;
        details.rescue_b_count  = 0;
        % <<<<<
        return;
    end

    % >>>>> RUNTIME_ANALYSIS PATCH 2c (Top-K timing) >>>>>
    t_topk = tic;
    % <<<<<
    K = min(5, popSize);
    candidate_stats.topk_parent_count = K;
    t_piece = tic;
    [~, sortIdx] = sort(fitness, 'ascend');
    topIdx       = sortIdx(1:K);
    timing.topk_generation_s = timing.topk_generation_s + toc(t_piece);

    % 全局最优 (跨 K 个候选 × 3 条路径)
    bestCost_final  = inf;
    bestPath_final  = [];
    bestDet_final   = struct();
    best_raw_det    = struct();
    best_smooth_det = struct();
    best_repair_det = struct();
    best_mild_det   = struct();   % v9: 温和平滑候选的诊断信息
    best_chosen_tag = 'none';   % 记录最终选中的路径类型

    % 辅助: 在 feasible 约束下比较两条路径
    % 返回 true 表示 (candJ, candPen, candFeas) 优于当前最优
    isBetter = @(candJ, candPen, candFeas, curJ, curPen, curFeas) ...
        (candFeas && ~curFeas) || ...                          % 可行优于不可行
        (candFeas &&  curFeas  && candJ   < curJ - 1e-9) || ...% 同为可行取 J 小
        (~candFeas && ~curFeas && candPen < curPen - 1e-9) || ...% 同不可行取 pen 小
        (~candFeas && ~curFeas && abs(candPen-curPen)<1e-9 && candJ < curJ - 1e-9);

    bestFeas = false;
    bestPen  = inf;

    % v10: mild 候选仅在 hard env (diffScale < 0.8) 启用, 避免对中等 env 的干扰
    enableMildCand = getOrDefault(cfg, 'difficultyScale', 1.0) < 0.8;

    for ci = 1:K
        t_piece = tic;
        rawPath = param2path(pop(topIdx(ci),:));
        timing.topk_generation_s = timing.topk_generation_s + toc(t_piece);
        candidate_stats.raw_generated = candidate_stats.raw_generated + 1;

        t_piece = tic;
        smoothed = smoothPathSpline(rawPath, env, minH, maxH);
        timing.smoothing_s = timing.smoothing_s + toc(t_piece);
        candidate_stats.smooth_generated = candidate_stats.smooth_generated + 1;

        % 评估 raw / smooth (固定两个候选)
        t_piece = tic;
        [raw_J,    raw_det]    = costModel.evaluatePath(rawPath,  t_start, hasPayload);
        [smooth_J, smooth_det] = costModel.evaluatePath(smoothed, t_start, hasPayload);
        timing.topk_evaluation_s = timing.topk_evaluation_s + toc(t_piece);
        candidate_stats.topk_candidates_evaluated = ...
            candidate_stats.topk_candidates_evaluated + 2;

        % v10: mild 候选 — 仅在 hard env 启用, 否则用 raw 占位避免日志中断
        if enableMildCand
            t_piece = tic;
            mildPath = mildSmoothPath(rawPath);
            timing.smoothing_s = timing.smoothing_s + toc(t_piece);
            candidate_stats.mild_generated = candidate_stats.mild_generated + 1;

            t_piece = tic;
            [mild_J, mild_det] = costModel.evaluatePath(mildPath, t_start, hasPayload);
            timing.topk_evaluation_s = timing.topk_evaluation_s + toc(t_piece);
            candidate_stats.topk_candidates_evaluated = ...
                candidate_stats.topk_candidates_evaluated + 1;
            cand_paths = {rawPath,  smoothed, mildPath};
            cand_dets  = {raw_det,  smooth_det, mild_det};
            cand_Js    = [raw_J,    smooth_J,   mild_J];
            cand_tags  = {'raw','smooth','mild'};
            nCand = 3;
        else
            mild_det = raw_det;          % 占位
            mild_det.J_final = raw_J;
            cand_paths = {rawPath,  smoothed};
            cand_dets  = {raw_det,  smooth_det};
            cand_Js    = [raw_J,    smooth_J];
            cand_tags  = {'raw','smooth'};
            nCand = 2;
        end

        t_piece = tic;
        for ci2 = 1:nCand
            cJ   = cand_Js(ci2);
            cDet = cand_dets{ci2};
            cFeas= cDet.feasible;
            cPen = cDet.penalty_total;

            if isBetter(cJ, cPen, cFeas, bestCost_final, bestPen, bestFeas)
                bestCost_final  = cJ;
                bestPath_final  = cand_paths{ci2};
                bestDet_final   = cDet;
                best_raw_det    = raw_det;
                best_smooth_det = smooth_det;
                best_repair_det = smooth_det;   % 兼容性占位
                best_mild_det   = mild_det;
                best_chosen_tag = cand_tags{ci2};
                bestFeas        = cFeas;
                bestPen         = cPen;
            end
        end
        timing.topk_selection_s = timing.topk_selection_s + toc(t_piece);
    end

    % 兜底: 所有候选均失败时回退到最优参数向量的 raw path
    if isempty(bestPath_final) || bestCost_final >= inf
        t_piece = tic;
        bestPath_final = param2path(bestPos);
        timing.topk_generation_s = timing.topk_generation_s + toc(t_piece);
        candidate_stats.raw_generated = candidate_stats.raw_generated + 1;

        t_piece = tic;
        [bestCost_final, bestDet_final] = costModel.evaluatePath(bestPath_final, t_start, hasPayload);
        timing.topk_evaluation_s = timing.topk_evaluation_s + toc(t_piece);
        candidate_stats.topk_candidates_evaluated = ...
            candidate_stats.topk_candidates_evaluated + 1;
        bestDet_final.repair_penalty = 0;
        best_raw_det    = bestDet_final;
        best_smooth_det = bestDet_final;
        best_repair_det = bestDet_final;
        best_mild_det   = bestDet_final;   % v9: 兜底同步
        best_chosen_tag = 'raw(fallback)';
    end

    % >>>>> RUNTIME_ANALYSIS PATCH 2c END / 2d START (topk + rescue timing) >>>>>
    timing.topk_s = toc(t_topk);
    timing.topk_overhead_s = max(0, timing.topk_s - ...
        timing.topk_generation_s - timing.smoothing_s - ...
        timing.topk_evaluation_s - timing.topk_selection_s);
    % <<<<<
    % ====================================================================
    % 定向二次救援 v10 — 多位置插点 + 高度变换
    %
    % Pass-A 对比 v9 的改进:
    %   v9 每轮只在碰撞段 "之前" 插点 (insert_after = k_c-1).
    %   当 v9 iter1 的绕行点与 p_k_c 之间形成一条长段 (≈156m, nSub=13),
    %   该长段自身穿越障碍轨迹产生 2 个新碰撞子点.
    %   v9 iter2 继续在 "之前" 插点, 对长段的起点没有改变效果, 因此无改善.
    %
    %   v10 每轮对最坏碰撞段尝试 3 个插点位置:
    %     Pos-A: 段之前 (k_c-1 / k_c 之间)  ← 延迟到达段起点
    %     Pos-B: 段之中 (k_c / k_c+1 之间, 在碰撞分数 frac_col 处)
    %            ← 直接在碰撞位置放安全航路点, 分割长段
    %     Pos-C: 段之后 (k_c+1 / k_c+2 之间) ← 延迟离开段终点
    %
    %   对每个位置, 尝试:
    %     · 8 方向 × 8 距离 (30/60/100/150/200/300/400/500 m)
    %     · + 高度变换: 在纯横向基础上试探飞越障碍顶部
    %       pt(3) = max(current_z, op_col_z + r_obs + 15m)
    %
    %   共 3 位置 × 8方向 × 8距离 × 3高度 = 576 候选/轮 (v11扩展)
    %   取使 P_dyn 减少最多的候选; 若无改善则退出迭代.
    %
    % MAX_INS_TOTAL = 6, Pass-B 逻辑不变.
    % ====================================================================
    if ~bestFeas
        rescueA_stats.triggered = true;
        rescueB_stats.triggered = true;
        MS_rc    = env.MAP_SIZE;
        cl_rc    = costModel.H_clearance;
        minH_rc  = minH;  maxH_rc = maxH;
        SUB_SP   = 12;    MIN_SUB_RC = 3;
        % MAX_INS_TOTAL: 根据运行上下文自适应调整
        %   主实验  = 6  （效率优先）
        %   统计/消融实验通过 ala_cfg.rescue_max_ins 传入更大值
        if isfield(cfg,'rescue_max_ins') && cfg.rescue_max_ins > 0
            MAX_INS_TOTAL = cfg.rescue_max_ins;
        else
            MAX_INS_TOTAL = 6;
        end

        % ══════════════════════════════════════════════════════════════════
        % Pass-A: 多位置迭代式时空绕行点插入
        % ══════════════════════════════════════════════════════════════════
        t_rescueA = tic;
        ins_total = 0;
        try
            rp_A  = bestPath_final;
            passA_mod = false;

            if isempty(env.dynObstacles) || ~isfield(env.dynObstacles,'movingObs')
                fprintf('    [救援A] 无移动障碍, 跳过\n');
            else
                rescueA_stats.executed = true;
                DIRS = zeros(8,2);
                for di_=1:8, ag=(di_-1)*pi/4; DIRS(di_,:)=[cos(ag),sin(ag)]; end
                % v11: 增加 400m/500m 大步长（应对大轨道半径障碍）
                DETOUR_D = [30, 60, 100, 150, 200, 300, 400, 500];
                obsList_rc = env.dynObstacles.movingObs;

                while ins_total < MAX_INS_TOTAL
                    % ── 步骤1: 重新评估, 获取最新 dyn_col_segs ──
                    [j_cur, det_cur] = costModel.evaluatePath(rp_A, t_start, hasPayload);
                    rescueA_stats.diagnostic_evaluations = ...
                        rescueA_stats.diagnostic_evaluations + 1;
                    if rescueA_stats.diagnostic_evaluations == 1 && ...
                            isfield(det_cur,'dyn_col_segs')
                        rescueA_stats.initial_conflict_segments = ...
                            nnz(det_cur.dyn_col_segs > 0);
                    end
                    tA_rc = det_cur.t_arrivals;
                    nRP_A = size(rp_A, 1);
                    if numel(tA_rc) ~= nRP_A
                        tA_rc = computeApproxTArr(rp_A, t_start);
                    end

                    if ~isfield(det_cur,'dyn_col_segs') || all(det_cur.dyn_col_segs==0)
                        break;
                    end

                    % ── 步骤2: 找当前最坏碰撞段 ──
                    [~, worst_seg_idx] = max(det_cur.dyn_col_segs);
                    k_c = worst_seg_idx;
                    if k_c < 1 || k_c >= size(rp_A,1), break; end

                    p1c = rp_A(k_c,:);   p2c = rp_A(k_c+1,:);
                    t1c = tA_rc(min(k_c,   numel(tA_rc)));
                    t2c = tA_rc(min(k_c+1, numel(tA_rc)));
                    d3c = norm(p2c-p1c);
                    if d3c < 0.01, break; end
                    nSub_c = max(MIN_SUB_RC, ceil(d3c/SUB_SP));

                    % ── 步骤3: 精确定位碰撞子点 + 障碍信息 ──
                    best_r_c=inf; t_col=(t1c+t2c)/2; frac_col=0.5;
                    op_col=zeros(1,3); obs_r_col=1;
                    for s_c=1:nSub_c
                        frac_c=(s_c-0.5)/nSub_c;
                        pt_c=p1c+frac_c*(p2c-p1c); t_c=t1c+frac_c*(t2c-t1c);
                        for oi_c=1:length(obsList_rc)
                            obs_c=obsList_rc(oi_c);
                            op_c=env.dynObstacles.getPosition(oi_c,t_c);
                            d_c=norm(pt_c-op_c);
                            if d_c<obs_c.radius && d_c/obs_c.radius<best_r_c
                                best_r_c=d_c/obs_c.radius;
                                t_col=t_c; frac_col=frac_c;
                                op_col=op_c; obs_r_col=obs_c.radius;
                            end
                        end
                    end
                    if best_r_c>=1, break; end

                    % ── 步骤4: 最多3个插点位置 × 8方向 × 8距离 × 3高度策略 ──
                    best_rnd_J    = j_cur;
                    best_rnd_pen  = det_cur.penalty_total;
                    best_rnd_feas = det_cur.feasible;
                    best_rnd_rp   = rp_A;
                    found_rnd     = false;

                    % 候选插入位置:
                    %   Pos-A: max(1,k_c-1) 到 k_c (段之前)
                    %   Pos-B: k_c 到 k_c+1 (段之中, 在 frac_col 位置)
                    %   Pos-C: k_c+1 到 k_c+2 (段之后, 若存在)
                    ins_positions = [max(1,k_c-1), k_c];
                    if k_c+1 < size(rp_A,1)
                        ins_positions = [ins_positions, k_c+1];
                    end

                    for ip = 1:length(ins_positions)
                        if found_rnd && best_rnd_feas, break; end
                        ins_after = ins_positions(ip);

                        % 插点的空间参考位置:
                        %   Pos-A/C: 使用碰撞子点位置作为推避参考中心
                        %   Pos-B: 精确使用碰撞分数位置
                        ref_pt = p1c + frac_col*(p2c-p1c);

                        for di_=1:8
                            if found_rnd && best_rnd_feas, break; end
                            ev_d = DIRS(di_,:);

                            for dist_i=1:length(DETOUR_D)
                                D_det = DETOUR_D(dist_i);

                                % --- 高度策略 1/2/3 ---
                                for hz_try = 1:3
                                    pt_det = op_col + [ev_d(1)*(obs_r_col+D_det), ...
                                                       ev_d(2)*(obs_r_col+D_det), 0];
                                    pt_det(1) = max(1,min(MS_rc,pt_det(1)));
                                    pt_det(2) = max(1,min(MS_rc,pt_det(2)));

                                    if hz_try == 1
                                        % 策略1: 纯横向, 保持当前段起点高度
                                        pt_det(3) = rp_A(min(ins_after,size(rp_A,1)),3);
                                    elseif hz_try == 2
                                        % 策略2: 飞越障碍顶部
                                        above_z = op_col(3) + obs_r_col + 15;
                                        if above_z > maxH_rc - 5, continue; end
                                        pt_det(3) = above_z;
                                    else
                                        % 策略3: 斜向抬升（横向+高度同步增加）
                                        % 在横向推避的同时额外抬高半个安全半径
                                        % 适用于障碍在斜上方或路径需要同时规避
                                        % 水平和垂直方向的场景
                                        diag_z = rp_A(min(ins_after,size(rp_A,1)),3) ...
                                                 + obs_r_col * 0.5;
                                        if diag_z > maxH_rc - 5, continue; end
                                        pt_det(3) = diag_z;
                                    end

                                    if ~isempty(env.heightMap)
                                        rx=max(1,min(MS_rc,round(pt_det(1))));
                                        ry=max(1,min(MS_rc,round(pt_det(2))));
                                        pt_det(3)=max(pt_det(3),max(minH_rc,env.heightMap(rx,ry)+cl_rc));
                                    end
                                    pt_det(3)=max(pt_det(3),minH_rc);
                                    pt_det(3)=min(pt_det(3),maxH_rc);

                                    % 边界截断检查
                                    actual_d = norm(pt_det(1:2)-op_col(1:2));
                                    if actual_d < (obs_r_col+D_det)*0.5, continue; end
                                    if ins_after >= size(rp_A,1), continue; end

                                    rp_try = [rp_A(1:ins_after,:); pt_det; rp_A(ins_after+1:end,:)];
                                    rescueA_stats.candidates_generated = ...
                                        rescueA_stats.candidates_generated + 1;
                                    [jT, detT] = costModel.evaluatePath(rp_try, t_start, hasPayload);
                                    rescueA_stats.candidates_evaluated = ...
                                        rescueA_stats.candidates_evaluated + 1;
                                    penT = detT.penalty_total;

                                    if isBetter(jT, penT, detT.feasible, ...
                                            best_rnd_J, best_rnd_pen, best_rnd_feas)
                                        best_rnd_J    = jT;
                                        best_rnd_pen  = penT;
                                        best_rnd_feas = detT.feasible;
                                        best_rnd_rp   = rp_try;
                                        found_rnd     = true;
                                        if detT.feasible, break; end
                                    end
                                end % hz_try
                                if found_rnd && best_rnd_feas, break; end
                            end % DETOUR_D
                            if found_rnd && best_rnd_feas, break; end
                        end % dirs
                    end % ins_positions

                    % ── 步骤5: 更新路径 ──
                    if found_rnd && isBetter(best_rnd_J, best_rnd_pen, ...
                            best_rnd_feas, j_cur, det_cur.penalty_total, det_cur.feasible)
                        rp_A    = best_rnd_rp;
                        ins_total = ins_total + 1;
                        passA_mod = true;
                        fprintf('    [救援A-iter%d] 插入绕行点: pen %.4f→%.4f\n', ...
                            ins_total, det_cur.penalty_total, best_rnd_pen);
                    else
                        fprintf(['    [救援A] 累计评估%d个候选均未产生进一步改善 ', ...
                            '(pen=%.4f), 退出迭代\n'], ...
                            rescueA_stats.candidates_evaluated, det_cur.penalty_total);
                        break;
                    end
                    if best_rnd_feas
                        fprintf('    [救援A] 已恢复严格可行性 (pen=%.4f)\n', best_rnd_pen);
                        break;
                    end
                end % while

                % ── 全局采用判断 ──
                if passA_mod
                    [jA, detA] = costModel.evaluatePath(rp_A, t_start, hasPayload);
                    rescueA_stats.final_evaluations = ...
                        rescueA_stats.final_evaluations + 1;
                    detA.repair_penalty = 0;
                    penA  = detA.penalty_total;  feasA = detA.feasible;
                    adoptA = isBetter(jA, penA, feasA, ...
                        bestCost_final, bestPen, bestFeas);
                    if adoptA
                        rescueA_stats.adopted = true;
                        bestPath_final=rp_A; bestCost_final=jA;
                        bestDet_final=detA; bestFeas=feasA; bestPen=penA;
                        best_chosen_tag=[best_chosen_tag,'+rescA'];
                        fprintf('    [救援A] ★ 多位置绕行成功 J=%.3f feas=%d pen=%.4f dyn=%.4f (共%d点)\n', ...
                            jA,feasA,penA,detA.penalty_dynamic_collision,ins_total);
                    else
                        fprintf('    [救援A] 绕行整体未改善 penA=%.4f vs %.4f\n',penA,bestPen);
                    end
                else
                    fprintf('    [救援A] 所有位置均无改善\n');
                end
            end
        catch me_A
            fprintf('    [救援A] 异常: %s\n', me_A.message);
        end
        rescueA_stats.successful_insertions = ins_total;
        timing.rescueA_s = toc(t_rescueA);

        % ══════════════════════════════════════════════════════════════════
        % Pass-B: 高度/静态段级抬升 (逻辑不变)
        % ══════════════════════════════════════════════════════════════════
        t_rescueB = tic;
        rescueB_stats.executed = true;
        try
            rp_B  = bestPath_final;
            nRP_B = size(rp_B,1);
            [~, det_B0] = costModel.evaluatePath(rp_B, t_start, hasPayload);
            rescueB_stats.diagnostic_evaluations = ...
                rescueB_stats.diagnostic_evaluations + 1;
            tA_rcB = det_B0.t_arrivals;
            if numel(tA_rcB)~=nRP_B, tA_rcB=computeApproxTArr(rp_B,t_start); end

            VIOL_SCALE=2.5; EXTRA_MARGIN=8; VIOL_THRESH=0.02; N_FIX_B=20;
            seg_viol_B=zeros(nRP_B-1,1); seg_lift_B=zeros(nRP_B-1,1);
            for k_B=1:nRP_B-1
                p1B=rp_B(k_B,:); p2B=rp_B(k_B+1,:);
                d3B=norm(p2B-p1B); if d3B<0.01,continue;end
                nSub_B=max(MIN_SUB_RC,ceil(d3B/SUB_SP));
                for s_B=1:nSub_B
                    frac_B=(s_B-0.5)/nSub_B; pt_B=p1B+frac_B*(p2B-p1B);
                    if ~isempty(env.heightMap)
                        rx=max(1,min(MS_rc,round(pt_B(1)))); ry=max(1,min(MS_rc,round(pt_B(2))));
                        gH_B=env.heightMap(rx,ry); minF_B=max(minH_rc,gH_B+cl_rc);
                        if pt_B(3)<minF_B
                            v=minF_B-pt_B(3); seg_viol_B(k_B)=seg_viol_B(k_B)+v;
                            seg_lift_B(k_B)=max(seg_lift_B(k_B),v*VIOL_SCALE+EXTRA_MARGIN);
                        end
                        if pt_B(3)<gH_B+3
                            v2=gH_B+3-pt_B(3); seg_viol_B(k_B)=seg_viol_B(k_B)+v2;
                            seg_lift_B(k_B)=max(seg_lift_B(k_B),v2*VIOL_SCALE+gH_B+cl_rc+EXTRA_MARGIN);
                        end
                    end
                end
            end
            [~,sortB]=sort(seg_viol_B,'descend'); passB_mod=false;
            for ki_B=1:min(nRP_B-1,N_FIX_B)
                k_B=sortB(ki_B); if seg_viol_B(k_B)<VIOL_THRESH,break;end
                lift=seg_lift_B(k_B);
                for fe_B=[k_B,k_B+1]
                    if fe_B==1||fe_B==nRP_B,continue;end
                    if ~isempty(env.heightMap)
                        rx=max(1,min(MS_rc,round(rp_B(fe_B,1)))); ry=max(1,min(MS_rc,round(rp_B(fe_B,2))));
                        gH_fe=env.heightMap(rx,ry);
                    else,gH_fe=0;end
                    targetZ=max(rp_B(fe_B,3)+lift,max(minH_rc,gH_fe+cl_rc+EXTRA_MARGIN));
                    rp_B(fe_B,3)=min(maxH_rc,targetZ); passB_mod=true;
                end
            end
            if passB_mod
                [jB,detB]=costModel.evaluatePath(rp_B,t_start,hasPayload);
                rescueB_stats.candidate_evaluations = ...
                    rescueB_stats.candidate_evaluations + 1;
                detB.repair_penalty=jB-bestCost_final;
                penB=detB.penalty_total; feasB=detB.feasible;
                adoptB=isBetter(jB,penB,feasB,bestCost_final,bestPen,bestFeas);
                if adoptB
                    rescueB_stats.adopted = true;
                    bestPath_final=rp_B; bestCost_final=jB; bestDet_final=detB;
                    bestFeas=feasB; bestPen=penB;
                    best_chosen_tag=[best_chosen_tag,'+rescB'];
                    fprintf('    [救援B] ★ 高度/静态修复成功 J=%.3f feas=%d pen=%.4f\n',jB,feasB,penB);
                else
                    fprintf('    [救援B] 未改善 penB=%.4f vs %.4f\n',penB,bestPen);
                end
            end
        catch me_B
            fprintf('    [救援B] 异常: %s\n', me_B.message);
        end
        timing.rescueB_s = toc(t_rescueB);

    end  % ~bestFeas

    % >>>>> RUNTIME_ANALYSIS PATCH 2d END (rescue timing) >>>>>
    timing.rescue_total_s = timing.rescueA_s + timing.rescueB_s;
    % <<<<<

    % ---- 诊断日志: 打印三阶段 J 和最终选择 (v9: 含 mild 候选) ----
    fprintf('    [路径选择] raw J=%.3f(feas=%d,pen=%.4f) | smooth J=%.3f(feas=%d,pen=%.4f) | mild J=%.3f(feas=%d,pen=%.4f)\n', ...
        best_raw_det.J_final,    best_raw_det.feasible,    best_raw_det.penalty_total, ...
        best_smooth_det.J_final, best_smooth_det.feasible, best_smooth_det.penalty_total, ...
        best_mild_det.J_final,   best_mild_det.feasible,   best_mild_det.penalty_total);
    fprintf('    [路径选择] ★ 最终选择: %s (J=%.3f, feasible=%d, diffScale=%.2f)\n', ...
        best_chosen_tag, bestCost_final, bestFeas, ...
        getOrDefault(cfg, 'difficultyScale', 1.0));

    path    = bestPath_final;
    cost    = bestCost_final;
    details = bestDet_final;
    details.internal_search_cost = internal_search_cost;
    details.chosen_path_type     = best_chosen_tag;   % 记录选择路径类型

    % >>>>> RUNTIME_ANALYSIS PATCH 2d/2e (rescue counts + timing into details) >>>>>
    %  计数口径: 该 run 最终采用的路径是否实际应用了 RescueA / RescueB
    %  (依据 best_chosen_tag 中的 '+rescA' / '+rescB' 标记; 每 run 取 0/1)
    %  注: 这是"应用到最终路径"的次数, 不是"尝试"次数 (尝试但失败的不计)。
    rescue_a_count = double(contains(best_chosen_tag, 'rescA'));
    rescue_b_count = double(contains(best_chosen_tag, 'rescB'));
    rescueA_stats.adopted = logical(rescue_a_count);
    rescueB_stats.adopted = logical(rescue_b_count);

    timing.total_s = toc(t_total);
    timing.other_s = max(0, timing.total_s - timing.initialization_s - ...
        timing.search_s - timing.topk_s - timing.rescueA_s - timing.rescueB_s);

    details.timing          = timing;
    details.candidate_stats = candidate_stats;
    details.rescueA_stats   = rescueA_stats;
    details.rescueB_stats   = rescueB_stats;
    details.rescue_a_count  = rescue_a_count;
    details.rescue_b_count  = rescue_b_count;
    % <<<<< RUNTIME_ANALYSIS PATCH 2d/2e END <<<<

    stage_details.raw    = best_raw_det;
    stage_details.smooth = best_smooth_det;
    % stage_details.repair 已移除（v4 设计变更：Repair 模块已删除）
    stage_details.chosen_path_type     = best_chosen_tag;
    stage_details.internal_search_cost = internal_search_cost;
    stage_details.timing               = timing;
    stage_details.candidate_stats      = candidate_stats;
    stage_details.rescueA_stats        = rescueA_stats;
    stage_details.rescueB_stats        = rescueB_stats;
end

%% ============== 参数向量 → 三维路径 (与原版完全一致) ==============

%% ============== v9 辅助: 安全字段获取 ==============
function v = getOrDefault(s, fname, dflt)
    if isfield(s, fname) && ~isempty(s.(fname))
        v = s.(fname);
    else
        v = dflt;
    end
end
