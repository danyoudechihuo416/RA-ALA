function [path, cost, details, stage_details] = runRA_ALA(...
        planner, costModel, env, start, goal, t_start, hasPayload, cfg)
% =========================================================================
% runRA_ALA v3.1 â€” é£Žé™©æ„ŸçŸ¥èƒ½è€—ä¼˜åŒ– ALA ä¸‰ç»´è·¯å¾„è§„åˆ’ (è¯Šæ–­åˆ†è§£ç‰ˆ)
%
%  è¾“å‡º:
%    path          [NÃ—3]  æœ€ç»ˆè·¯å¾„ï¼ˆrepairåŽï¼‰
%    cost          æ ‡é‡   æœ€ç»ˆä»£ä»·ï¼ˆç»Ÿä¸€ evaluatePath è®¡ç®—ï¼‰
%    details       struct æœ€ç»ˆè·¯å¾„çš„å®Œæ•´ä»£ä»·åˆ†è§£
%    stage_details struct ä¸‰é˜¶æ®µåˆ†è§£:
%                           .raw    â€” raw path çš„ evaluatePath ç»“æžœ
%                           .smooth â€” smooth path çš„ evaluatePath ç»“æžœ
%                           .repair â€” repair path çš„ evaluatePath ç»“æžœ
%
%  æŽ¥å£å…¼å®¹: [path, cost, details] = runRA_ALA(...)  ä»ç„¶æœ‰æ•ˆ
%
%  ç»Ÿä¸€è¯„ä¼°å£å¾„ (ç›®æ ‡2):
%    æœç´¢é˜¶æ®µé€‚åº”åº¦ evalRA_v2 ä»ç”¨äºŽæ¯”è¾ƒå€™é€‰ä¼˜åŠ£ (å« NFZ/smooth æƒ©ç½š).
%    é˜¶æ®µäºŒçš„æœ€ç»ˆ J ç»Ÿä¸€é€šè¿‡ costModel.evaluatePath é‡æ–°è®¡ç®—,
%    ä¸æŠŠ evalRA_v2 çš„å†…éƒ¨å€¼ç›´æŽ¥å½“æœ€ç»ˆè¾“å‡º.
% =========================================================================

    % noWindBias comparison variant: keep the wind field, propulsion model,
    % and headwind-risk evaluation unchanged, but disable the wind-biased
    % ALA walk operator for every caller of this copied project.
    cfg.ablate_windBias = true;

    % >>>>> RUNTIME/COUNTING INSTRUMENTATION >>>>>
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
    % <<<<< RUNTIME/COUNTING INSTRUMENTATION END <<<<<

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

    % â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ v9: çŽ¯å¢ƒç¡¬åº¦è‡ªé€‚åº”ç¼©æ”¾æ³¨å…¥ â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    % ä¸€æ¬¡ä¼°ç®—, åœ¨æ•´æ¬¡ runRA_ALA æœŸé—´å…±ç”¨. å†™å…¥ cfg.difficultyScale åŽ
    % evalRA_v2 å†…éƒ¨çš„ NFZ/obs/headwind ç½šé¡¹ä¼šè‡ªåŠ¨æŒ‰æ­¤å› å­ç¼©æ”¾.
    % åŒæ—¶ walk operator çš„ windBias ä¹Ÿä¼šæ ¹æ® difficultyScale æ•´ä½“å¼±åŒ–.
    if ~(isfield(cfg,'difficultyScale') && ~isempty(cfg.difficultyScale))
        % ä»…åœ¨å¤–éƒ¨æœªæ˜¾å¼æŒ‡å®šæ—¶æ‰è‡ªåŠ¨ä¼°ç®— (å…è®¸è°ƒç”¨æ–¹è¦†ç›–)
        [diffScaleAuto, diffInfo] = estimateEnvDifficulty(env, start, goal);
        cfg.difficultyScale = diffScaleAuto;
        cfg.difficultyInfo  = diffInfo;
    end

    param2path = @(x) paramToPath(x, start, goal, nWP, dirUnit, perpUnit, totalDist, env, minH);
    evalFcn    = @(x) evalRA_v2(x, param2path, t_start, hasPayload, costModel, env, cfg);

    % ---- åˆå§‹åŒ–ç§ç¾¤ ----
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
    % â˜… ç›®æ ‡1: å¢žåŠ é«˜åº¦æ„ŸçŸ¥å®‰å…¨å€™é€‰
    % å€™é€‰3: é£žè¡Œåœ¨è¾ƒé«˜é«˜åº¦å±‚ (é¿å¼€å¤§å¤šæ•°å»ºç­‘)
    safeH3 = min(maxH, max(minH, minH + 0.6*(maxH-minH)));  % 60%é«˜åº¦å±‚
    for j=1:nWP, pop(3,(j-1)*2+1)=0; pop(3,(j-1)*2+2)=safeH3; end
    % å€™é€‰4: ç›´çº¿è·¯å¾„ + ä¸­ç­‰é«˜åº¦
    safeH4 = min(maxH, max(minH, minH + 0.4*(maxH-minH)));  % 40%é«˜åº¦å±‚
    for j=1:nWP, pop(min(4,popSize),(j-1)*2+1)=0; pop(min(4,popSize),(j-1)*2+2)=safeH4; end
    % å€™é€‰5: æ²¿é€”å– heightMap æœ€å¤§å€¼å†åŠ å‡€ç©º (ç´§è´´å»ºç­‘é¡¶)
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
    for i = max(7,3):popSize   % ä»Ž 7 å¼€å§‹ï¼Œä¿ç•™ slot 6 ç»™ EA* çƒ­å¯åŠ¨ç§å­
        for j = 1:nWP
            pop(i,(j-1)*2+1) = (rand-0.5)*120;
            pop(i,(j-1)*2+2) = minH + rand*(maxH-minH);
        end
    end
    pop = max(lb, min(ub, pop));

    % â”€â”€ EA* çƒ­å¯åŠ¨ç§å­ï¼ˆæ”¾åœ¨éšæœºåˆå§‹åŒ–ä¹‹åŽï¼Œç¡®ä¿ä¸è¢«è¦†ç›–ï¼‰â”€â”€
    % å½“å‰ç§ç¾¤ç»“æž„:
    %   C1: ç›´çº¿è·¯å¾„ï¼ˆz=start_zï¼‰
    %   C2: è´ªå¿ƒè·¯å¾„æŠ•å½±
    %   C3: é«˜åº¦å±‚ 84mï¼ˆ60%å±‚ï¼‰
    %   C4: é«˜åº¦å±‚ 66mï¼ˆ40%å±‚ï¼‰
    %   C5: ç´§è´´å»ºç­‘é¡¶
    %   C6: â˜… EA* è·¯å¾„çƒ­å¯åŠ¨ï¼ˆå¯è¡Œä¿éšœç§å­ï¼‰â† æ­¤å¤„å†™å…¥ï¼Œä¸ä¼šè¢«è¦†ç›–
    %   C7~C30: éšæœºåˆå§‹åŒ–
    %
    % ä¸ºä»€ä¹ˆèƒ½ä¿è¯å¯è¡Œæ€§:
    %   C6 æ˜¯å¯è¡Œä¸ªä½“ â†’ isBetter è§„åˆ™ä¸‹å¯è¡Œä¼˜å…ˆ
    %   â†’ ALA è¿­ä»£æ°¸è¿œä¸ä¼šç”¨ä¸å¯è¡Œä¸ªä½“æ›¿æ¢ C6
    %   â†’ ç§ç¾¤å§‹ç»ˆä¿æœ‰è‡³å°‘ä¸€æ¡å¯è¡Œè·¯å¾„
    %   â†’ æœ€ç»ˆè¾“å‡ºæ˜¯ ALA ä»Ž C6 å‡ºå‘æœç´¢ä¼˜åŒ–çš„ç»“æžœï¼Œä¸æ˜¯ EA* è·¯å¾„æœ¬èº«
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
                pop(6, :)  = max(lb, min(ub, x_ea));   % å†™å…¥ slot 6
                ea_seed_ok = true;
            end
        catch; end
    end
    timing.warm_start_s = toc(t_warm_start);
    if ~ea_seed_ok
        % EA* å¤±è´¥æ—¶ slot 6 ä¿æŒéšæœºå€¼ï¼ˆä¸ŽåŽŸæ¥ç›¸åŒï¼Œæ— é€€åŒ–ï¼‰
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

    % ---- å®žæ—¶æ˜¾ç¤ºï¼šå»ºç«‹å›¾å½¢çª—å£ ----
    if exist('SHOW_LIVE','var') && SHOW_LIVE
        % æ”¶æ•›æ›²çº¿çª—å£
        if exist('SHOW_CONV_CURVE','var') && SHOW_CONV_CURVE
            fig_conv = figure('Name','RA-ALA æ”¶æ•›æ›²çº¿ï¼ˆå®žæ—¶ï¼‰', ...
                'NumberTitle','off','Position',[50 500 520 280]);
            ax_conv = axes('Parent',fig_conv);
            title(ax_conv, sprintf('RA-ALA æœç´¢æ”¶æ•›æ›²çº¿  t_0=%ds', round(t_start)), ...
                'FontSize',11,'FontWeight','bold');
            xlabel(ax_conv,'è¿­ä»£æ¬¡æ•°'); ylabel(ax_conv,'æœ€ä¼˜é€‚åº”åº¦ (å†…éƒ¨å£å¾„)');
            hold(ax_conv,'on'); grid(ax_conv,'on');
            h_line = plot(ax_conv, nan, nan, '-', 'Color','#C0392B', 'LineWidth',2);
            drawnow;
        end
        % è·¯å¾„é¢„è§ˆçª—å£
        if exist('SHOW_PATH_LIVE','var') && SHOW_PATH_LIVE
            fig_path = figure('Name','RA-ALA è·¯å¾„å®žæ—¶é¢„è§ˆ', ...
                'NumberTitle','off','Position',[600 500 500 450]);
            ax_path = axes('Parent',fig_path);
            axis(ax_path,'equal'); hold(ax_path,'on'); grid(ax_path,'on');
            xlabel(ax_path,'X (m)'); ylabel(ax_path,'Y (m)');
            title(ax_path, sprintf('å½“å‰æœ€ä¼˜è·¯å¾„é¢„è§ˆ  t_0=%ds', round(t_start)), ...
                'FontSize',11,'FontWeight','bold');
            % ç”»å»ºç­‘åº•å›¾
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
    conv_hist = zeros(maxIter,1);  % æ”¶æ•›åŽ†å²

    timing.initialization_s = toc(t_initialization);

    % ---- ALA ä¸»è¿­ä»£ ----
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
                if isfield(cfg,'ablate_windBias') && cfg.ablate_windBias
                    noise  = randn(1,dim).*(ub-lb)*0.08*sigma_decay;
                    newPos = pop(i,:) + E * noise;
                else
                    midPt   = (start+goal)/2;
                    w       = env.windField.getWind(midPt(1),midPt(2),midPt(3),t_start);
                    windLat = dot(w(1:2), perpUnit);
                    windBias = zeros(1,dim);
                    for j=1:nWP, windBias((j-1)*2+1) = -windLat*3*sigma_decay; end
                    noise  = randn(1,dim).*(ub-lb)*0.06*sigma_decay;
                    % v9: windBias åœ¨ hard env ä¸­è‡ªé€‚åº”å¼±åŒ–, é¿å…æŠŠæœç´¢æŽ¨å‘ NFZ é›†ç¾¤
                    %   diffScale=1.0 (ä¸­ç­‰çŽ¯å¢ƒ) â†’ windBias å¼ºåº¦ 0.30 (åŽŸå€¼ä¸å˜)
                    %   diffScale=0.3 (æžç¡¬çŽ¯å¢ƒ) â†’ windBias å¼ºåº¦ 0.09 (å‰Šå¼± 70%)
                    %   diffScale=1.5 (ç©ºæ—·çŽ¯å¢ƒ) â†’ windBias å¼ºåº¦ 0.45 (ç•¥å¢žå¼º)
                    if isfield(cfg,'difficultyScale')
                        wb_strength = 0.30 * cfg.difficultyScale;
                    else
                        wb_strength = 0.30;
                    end
                    newPos = pop(i,:) + E*noise + windBias*wb_strength;
                end
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

        % â”€â”€ å®žæ—¶æ˜¾ç¤ºæ›´æ–° â”€â”€
        conv_hist(iter) = bestFit;
        if exist('SHOW_LIVE','var') && SHOW_LIVE

            % å‘½ä»¤è¡Œè¿›åº¦æ¡
            if exist('SHOW_ITER_BAR','var') && SHOW_ITER_BAR
                pct = iter/maxIter;
                bar_len = 30;
                filled  = round(pct * bar_len);
                bar_str = [repmat('â–ˆ',1,filled), repmat('â–‘',1,bar_len-filled)];
                fprintf('\r  è¿­ä»£ [%s] %3d/%d  bestJ=%-8.3f', ...
                    bar_str, iter, maxIter, bestFit);
                if iter == maxIter, fprintf('\n'); end
            end

            % æ¯ REFRESH_EVERY è½®åˆ·æ–°å›¾å½¢
            if exist('REFRESH_EVERY','var') && mod(iter, REFRESH_EVERY)==0
                % æ›´æ–°æ”¶æ•›æ›²çº¿
                if exist('SHOW_CONV_CURVE','var') && SHOW_CONV_CURVE && ishandle(fig_conv)
                    set(h_line, 'XData', 1:iter, 'YData', conv_hist(1:iter));
                    xlim(ax_conv, [1, maxIter]);
                    drawnow limitrate;
                end
                % æ›´æ–°è·¯å¾„é¢„è§ˆ
                if exist('SHOW_PATH_LIVE','var') && SHOW_PATH_LIVE && ishandle(fig_path)
                    curPath = param2path(bestPos);
                    set(h_path, 'XData', curPath(:,1), 'YData', curPath(:,2));
                    title(ax_path, sprintf('è¿­ä»£ %d/%d  å½“å‰æœ€ä¼˜ J=%.3f', ...
                        iter, maxIter, bestFit), 'FontSize',10,'FontWeight','bold');
                    drawnow limitrate;
                end
            end

            % æ¯ PRINT_çN}¶‰žËkºwµçH[™ˆ[™ˆYˆ™\ÝÜ—ØÏLKœ™XZÎÈ[™‚ˆ	H8¥ 8¥ 9«izj©ˆ9§ 9i&Œù.*¹£ä¹à®y/cyïkˆ0åÈ9¥®yd$H0åÈ:-çyé®È0åÈújæ9n©¹ëe¹åiH8¥ 8¥ ˆ™\ÝÜ›™Ü[ˆH]ØÝ\‹œ[˜[WÝÝ[Âˆ™\ÝÜ›™ÜœHœÐNÂˆ›Ý[™Ü›™H˜[ÙNÂ‚ˆ	H9`&z`"y£ä¹aiy/cyïkŽ‚ˆ	HÜËPNˆX^
K×ØËLJH9b,×ØÈ
9«­y.bùbcJBˆ	HÜËPŽˆ×ØÈ9b,×ØÊÌH
9«­y.bù.+K9g*œ˜X×ØÛÛ9/cyïkŠBˆ	HÜËPÎˆ×ØÊÌH9b,×ØÊÌˆ
9«­y.bùd#‹:"éykf9g*
Bˆ[œ×ÜÜÚ][ÛœÈHÛX^
K×ØËLJK×Ø×NÂˆYˆ×ØÊÌHÚ^™JœÐKJBˆ[œ×ÜÜÚ][ÛœÈHÚ[œ×ÜÜÚ][ÛœË×ØÊÌWNÂˆ[™‚ˆ›Üˆ\HN›[™Ý
[œ×ÜÜÚ][ÛœÊBˆYˆ›Ý[™Ü›™	‰ˆ™\ÝÜ›™Ü[ˆYKM‹œ™XZÎÈ[™ˆ[œ×ØY\ˆH[œ×ÜÜÚ][ÛœÊ\
NÂ‚ˆ	H9£ä¹à®yæ¡9ênºeí9càº  ù/cyïkŽ‚ˆ	HÜËPKÐÎˆ9/oùå*9è¬9¤§¹kd9à®y/cyïk¹/g9..¹£ª:`oùcàº  ù.+yoàÂˆ	HÜËPŽˆ9ì¯¹èk¹/oùå*9è¬9¤§¹b!¹¥l9/cyïk‚ˆ™Y—ÜHXÈ
Èœ˜X×ØÛÛ
Š˜Ë\XÊNÂ‚ˆ›ÜˆWÏLNŽˆYˆ›Ý[™Ü›™	‰ˆ™\ÝÜ›™Ü[ˆYKM‹œ™XZÎÈ[™ˆ]—ÙHT”ÊWËŠNÂ‚ˆ›Üˆ\ÝÚOLN›[™Ý
UÕT—Ñ
BˆÙ]HUÕT—Ñ
\ÝÚJNÂ‚ˆ	HKKH:jæ9n©¹ëe¹åiHKÌ‹ÌÈKKBˆ›Üˆ—ÝžHHNŒÂˆÙ]HÜØÛÛ
ÈÙ]—Ù
JJŠØœ×Ü—ØÛÛ
ÑÙ]
K‹‹‚ˆ]—Ù
ŠJŠØœ×Ü—ØÛÛ
ÑÙ]
KNÂˆÙ]
JHHX^
KZ[ŠT×Ü˜ËÙ]
JJJNÂˆÙ]
ŠHHX^
KZ[ŠT×Ü˜ËÙ]
ŠJJNÂ‚ˆYˆ—ÝžHOHBˆ	H9ëe¹åiLNˆ9î«ùª*¹d$K9/çy£ yodùbcy«­z-mùà®zjæ9n©‚ˆÙ]
ÊHHœÐJZ[Š[œ×ØY\‹Ú^™JœÐKJJKÊNÂˆ[ÙZYˆ—ÝžHOH‚ˆ	H9ëe¹åiLŽˆ:hçº-¢ºf§9è£zhmº`êˆX›Ý™WÞˆHÜØÛÛ
ÊH
ÈØœ×Ü—ØÛÛ
ÈMNÂˆYˆX›Ý™WÞˆˆX^Ü˜ÈHKÛÛ[YNÈ[™ˆÙ]
ÊHHX›Ý™WÞŽÂˆ[ÙBˆ	H9ëe¹åiLÎˆ9¥§9d$y¢«9caûï"9ª*¹d$Jújæ9n©¹d#9«iyh§¹b¨;ï"Bˆ	H9g*9ª*¹d$y£ª:`oùæ¡9d#9¥íºh§yi%¹¢«:jæ9cb¹.*¹k¢yaj9cb¹o¡ˆ	H:` ¹å*9.£ºf§9è£yg*9¥§9."¹¥®y¢%º-ëùo¡:g :) yd#9¥íº)á:`oÂˆ	H9¬-9nlùd£9g ¹æí9¥®yd$yæ¡9g.¹¦kÂˆXY×ÞˆHœÐJZ[Š[œ×ØY\‹Ú^™JœÐKJJKÊH‹‹‚ˆ
ÈØœ×Ü—ØÛÛ
ˆNÂˆYˆXY×ÞˆˆX^Ü˜ÈHKÛÛ[YNÈ[™ˆÙ]
ÊHHXY×ÞŽÂˆ[™‚ˆYˆš\Ù[\J[‹šZYÚX\
Bˆž[X^
KZ[ŠT×Ü˜Ë›Ý[™
Ù]
JJJJNÂˆžO[X^
KZ[ŠT×Ü˜Ë›Ý[™
Ù]
ŠJJJNÂˆÙ]
ÊO[X^
Ù]
ÊKX^
Z[’Ü˜Ë[‹šZYÚX\
žžJJØÛÜ˜ÊJNÂˆ[™ˆÙ]
ÊO[X^
Ù]
ÊKZ[’Ü˜ÊNÂˆÙ]
ÊO[Z[ŠÙ]
ÊKX^Ü˜ÊNÂ‚ˆ	H:/®yåc9¢*¹¥«y¨à9§éBˆXÝX[ÙH›Ü›JÙ]
NŒŠK[ÜØÛÛ
NŒŠJNÂˆYˆXÝX[Ù
Øœ×Ü—ØÛÛ
ÑÙ]
JŒKÛÛ[YNÈ[™ˆYˆ[œ×ØY\ˆHÚ^™JœÐKJKÛÛ[YNÈ[™‚ˆœÝžHHÜœÐJNš[œ×ØY\‹ŠNÈÙ]ÈœÐJ[œ×ØY\ŠÌN™[™ŠWNÂˆ™\ØÝYPWÜÝ]Ë˜Ø[™Y]\×ÙÙ[™\˜]YH‹‹‚ˆ™\ØÝYPWÜÝ]Ë˜Ø[™Y]\×ÙÙ[™\˜]Y
ÈNÂˆÚ•]HHÛÜÝ[Ù[™]˜[X]T]
œÝžKÜÝ\YJNÂˆ™\ØÝYPWÜÝ]Ë˜Ø[™Y]\×Ù]˜[X]YH‹‹‚ˆ™\ØÝYPWÜÝ]Ë˜Ø[™Y]\×Ù]˜[X]Y
ÈNÂˆ[•H]œ[˜[WÝÝ[Â‚ˆYˆ[•™\ÝÜ›™Ü[ˆHYKM‚ˆ™\ÝÜ›™Ü[ˆH[•Âˆ™\ÝÜ›™ÜœHœÝžNÂˆ›Ý[™Ü›™HYNÂˆYˆ]™™X\ÚX›Kœ™XZÎÈ[™ˆ[™ˆ[™	H—ÝžBˆYˆ›Ý[™Ü›™	‰ˆ™\ÝÜ›™Ü[ˆYKM‹œ™XZÎÈ[™ˆ[™	HUÕT—ÑˆYˆ›Ý[™Ü›™	‰ˆ™\ÝÜ›™Ü[ˆYKM‹œ™XZÎÈ[™ˆ[™	H\œÂˆ[™	H[œ×ÜÜÚ][ÛœÂ‚ˆ	H8¥ 8¥ 9«izj©Nˆ9¦í9¥¬:-ëùo¡8¥ 8¥ ˆYˆ›Ý[™Ü›™	‰ˆ™\ÝÜ›™Ü[ˆ™]—Ü[ˆHYKM‚ˆœÐHH™\ÝÜ›™ÜœÂˆ™]—Ü[H™\ÝÜ›™Ü[ŽÂˆ[œ×ÝÝ[H[œ×ÝÝ[
ÈNÂˆ\ÜÐWÛ[ÙHYNÂˆœš[Š	Èù¥dy£íKZ]\‰YH9£ä¹aiyîåz(c9à®Nˆ[ˆ	K¸¡¤‰K—‰Ë‹‹‚ˆ[œ×ÝÝ[]ØÝ\‹œ[˜[WÝÝ[™\ÝÜ›™Ü[ŠNÂˆ[ÙBˆœš[ŠÉÈù¥dy£íWH9í+ú+¨z+á9/,	Y9.*¹`&z`"ygaù§*¹.©ùå'ú/æù. 9«iy¥.ye¡	Ë‹‹‚ˆ	Ê[IKŠK:` 9aîº/ëy.è×‰×K‹‹‚ˆ™\ØÝYPWÜÝ]Ë˜Ø[™Y]\×Ù]˜[X]Y]ØÝ\‹œ[˜[WÝÝ[
NÂˆœ™XZÎÂˆ[™ˆYˆ™\ÝÜ›™Ü[ˆŒBˆœš[Š	Èù¥dy£íWH9bª9  yè¬9¤§¹mì¹­¢:fi
[IKˆŒJW‰Ë™\ÝÜ›™Ü[ŠNÂˆœ™XZÎÂˆ[™ˆ[™	HÚ[B‚ˆ	H8¥ 8¥ 9aj9l`:aáùå*9b)9¥«H8¥ 8¥ ˆYˆ\ÜÐWÛ[ÙˆÚK]WHHÛÜÝ[Ù[™]˜[X]T]
œÐKÜÝ\YJNÂˆ™\ØÝYPWÜÝ]Ë™š[˜[Ù]˜[X][ÛœÈH‹‹‚ˆ™\ØÝYPWÜÝ]Ë™š[˜[Ù]˜[X][ÛœÈ
ÈNÂˆ]Kœ™\Z\—Ü[˜[HHÂˆ[HH]Kœ[˜[WÝÝ[È™X\ÐHH]K™™X\ÚX›NÂˆYÜHH
™X\ÐH	‰ˆ˜™\Ý™X\ÊH‹‹‚ˆ
[H™\Ý[‹LYKMŠH‹‹‚ˆ
XœÊ[KX™\Ý[ŠOYKMˆ	‰ˆH™\ÝÛÜÝÙš[˜[LYKMŠNÂˆYˆYÜBˆ™\ØÝYPWÜÝ]Ë˜YÜYHYNÂˆ™\Ý]Ùš[˜[\œÐNÈ™\ÝÛÜÝÙš[˜[ZNÂˆ™\Ý]Ùš[˜[Y]NÈ™\Ý™X\ÏY™X\ÐNÈ™\Ý[\[NÂˆ™\ÝØÚÜÙ[—ÝYÏVØ™\ÝØÚÜÙ[—ÝYË	ÊÜ™\ØÐI×NÂˆœš[Š	Èù¥dy£íWH8¦!H9i&¹/cyïk¹îåz(c9¢$9b§ÈIKŒÙˆ™X\ÏIY[IKˆ[IKˆ
9alIY9à®JW‰Ë‹‹‚ˆK™X\ÐK[K]Kœ[˜[WÙ[˜[ZX×ØÛÛ\Ú[Û‹[œ×ÝÝ[
NÂˆ[ÙBˆœš[Š	Èù¥dy£íWH9îåz(c9¥m9/dù§*¹¥.ye¡[OIKˆœÈ	K—‰Ë[K™\Ý[ŠNÂˆ[™ˆ[ÙBˆœš[Š	Èù¥dy£íWH9¢`9§"y/cyïk¹gaù¥è9¥.ye¡‰ÊNÂˆ[™ˆ[™ˆØ]ÚYWÐBˆœš[Š	Èù¥dy£íWH9o ¹n.ˆ	\×‰ËYWÐK›Y\ÜØYÙJNÂˆ[™ˆ™\ØÝYPWÜÝ]ËœÝXØÙ\ÜÙ[Ú[œÙ\[ÛœÈH[œ×ÝÝ[Âˆ[Z[™Ëœ™\ØÝYPWÜÈHØÊÜ™\ØÝYPJNÂ‚ˆ	H8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥dˆ	H\ÜËPŽˆ:jæ9n©‹úgfy  y«­yî©ù¢«9caÈ
:`.ú/¤y.#ycæ
Bˆ	H8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥d8¥dˆÜ™\ØÝYPˆHXÎÂˆ™\ØÝYP—ÜÝ]Ë™^XÝ]YHYNÂˆžBˆœÐˆH™\Ý]Ùš[˜[Âˆ””ÐˆHÚ^™JœÐ‹JNÂˆß‹]ÐŒHHÛÜÝ[Ù[™]˜[X]T]
œÐ‹ÜÝ\YJNÂˆ™\ØÝYP—ÜÝ]Ë™XYÛ›ÜÝX×Ù]˜[X][ÛœÈH‹‹‚ˆ™\ØÝYP—ÜÝ]Ë™XYÛ›ÜÝX×Ù]˜[X][ÛœÈ
ÈNÂˆWÜ˜ÐˆH]ÐŒØ\œš]˜[ÎÂˆYˆ[Y[
WÜ˜ÐŠ_[””Ð‹WÜ˜ÐXÛÛ\]P\›Þ\œŠœÐ‹ÜÝ\
NÈ[™‚ˆ’SÓÔÐÐSOL‹NÈVWÓPT‘ÒSNÈ’SÓÕ‘TÒLŒŽÈ—Ñ’VÐLŒÂˆÙY×Ýš[ÛÐ^™\›ÜÊ””Ð‹LKJNÈÙY×ÛYÐ^™\›ÜÊ””Ð‹LKJNÂˆ›Üˆ×ÐLN›””Ð‹LBˆP\œÐŠ×Ð‹ŠNÈ\œÐŠ×ÐŠÌKŠNÂˆÐ[›Ü›J‹\PŠNÈYˆÐŒKÛÛ[YNÙ[™ˆ”ÝX—Ð[X^
RS—ÔÕP—ÔËÙZ[
Ð‹ÔÕP—ÔÔ
JNÂˆ›Üˆ×ÐLN›”ÝX—Ð‚ˆœ˜X×ÐJ×Ð‹LJKÛ”ÝX—ÐŽÈÐ\PŠÙœ˜X×ÐŠŠ‹\PŠNÂˆYˆš\Ù[\J[‹šZYÚX\
Bˆž[X^
KZ[ŠT×Ü˜Ë›Ý[™
ÐŠJJJJNÈžO[X^
KZ[ŠT×Ü˜Ë›Ý[™
ÐŠŠJJJNÂˆÒÐY[‹šZYÚX\
žžJNÈZ[‘—Ð[X^
Z[’Ü˜ËÒÐŠØÛÜ˜ÊNÂˆYˆÐŠÊOZ[‘—Ð‚ˆ[Z[‘—Ð‹\ÐŠÊNÈÙY×Ýš[ÛÐŠ×ÐŠO\ÙY×Ýš[ÛÐŠ×ÐŠJÝŽÂˆÙY×ÛYÐŠ×ÐŠO[X^
ÙY×ÛYÐŠ×ÐŠKŠ•’SÓÔÐÐSJÑVWÓPT‘ÒSŠNÂˆ[™ˆYˆÐŠÊOÒÐŠÌÂˆŒYÒÐŠÌË\ÐŠÊNÈÙY×Ýš[ÛÐŠ×ÐŠO\ÙY×Ýš[ÛÐŠ×ÐŠJÝŒŽÂˆÙY×ÛYÐŠ×ÐŠO[X^
ÙY×ÛYÐŠ×ÐŠKŒŠ•’SÓÔÐÐSJÙÒÐŠØÛÜ˜ÊÑVWÓPT‘ÒSŠNÂˆ[™ˆ[™ˆ[™ˆ[™ˆß‹ÛÜ—O\ÛÜ
ÙY×Ýš[ÛÐ‹	Ù\ØÙ[™	ÊNÈ\ÜÐ—Û[ÙY˜[ÙNÂˆ›ÜˆÚWÐLN›Z[Š””Ð‹LK—Ñ’VÐŠBˆ×Ð\ÛÜŠÚWÐŠNÈYˆÙY×Ýš[ÛÐŠ×ÐŠO’SÓÕ‘TÒœ™XZÎÙ[™ˆY\ÙY×ÛYÐŠ×ÐŠNÂˆ›Üˆ™WÐVÚ×Ð‹×ÐŠÌWBˆYˆ™WÐOL_™WÐO[””Ð‹ÛÛ[YNÙ[™ˆYˆš\Ù[\J[‹šZYÚX\
Bˆž[X^
KZ[ŠT×Ü˜Ë›Ý[™
œÐŠ™WÐ‹JJJJNÈžO[X^
KZ[ŠT×Ü˜Ë›Ý[™
œÐŠ™WÐ‹ŠJJJNÂˆÒÙ™OY[‹šZYÚX\
žžJNÂˆ[ÙKÒÙ™OLÙ[™ˆ\™Ù][X^
œÐŠ™WÐ‹ÊJÛYX^
Z[’Ü˜ËÒÙ™JØÛÜ˜ÊÑVWÓPT‘ÒSŠJNÂˆœÐŠ™WÐ‹ÊO[Z[ŠX^Ü˜Ë\™Ù]ŠNÈ\ÜÐ—Û[Ù]YNÂˆ[™ˆ[™ˆYˆ\ÜÐ—Û[ÙˆÚ‹]—OXÛÜÝ[Ù[™]˜[X]T]
œÐ‹ÜÝ\YJNÂˆ™\ØÝYP—ÜÝ]Ë˜Ø[™Y]WÙ]˜[X][ÛœÈH‹‹‚ˆ™\ØÝYP—ÜÝ]Ë˜Ø[™Y]WÙ]˜[X][ÛœÈ
ÈNÂˆ]‹œ™\Z\—Ü[˜[OZ‹X™\ÝÛÜÝÙš[˜[Âˆ[Y]‹œ[˜[WÝÝ[È™X\ÐY]‹™™X\ÚX›NÂˆYÜJ™X\Ð‰‰Ÿ˜™\Ý™X\Ê_
[™\Ý[‹LYKMŠ_
XœÊ[‹X™\Ý[ŠOYKM‰‰š™\ÝÛÜÝÙš[˜[LYKMŠNÂˆYˆYÜ‚ˆ™\ØÝYP—ÜÝ]Ë˜YÜYHYNÂˆ™\Ý]Ùš[˜[\œÐŽÈ™\ÝÛÜÝÙš[˜[ZŽÈ™\Ý]Ùš[˜[Y]ŽÂˆ™\Ý™X\ÏY™X\ÐŽÈ™\Ý[\[ŽÂˆ™\ÝØÚÜÙ[—ÝYÏVØ™\ÝØÚÜÙ[—ÝYË	ÊÜ™\ØÐ‰×NÂˆœš[Š	Èù¥dy£í—H8¦!H:jæ9n©‹úgfy  y/ë¹i#y¢$9b§ÈIKŒÙˆ™X\ÏIY[IK—‰Ë‹™X\Ð‹[ŠNÂˆ[ÙBˆœš[Š	Èù¥dy£í—H9§*¹¥.ye¡[IKˆœÈ	K—‰Ë[‹™\Ý[ŠNÂˆ[™ˆ[™ˆØ]ÚYWÐ‚ˆœš[Š	Èù¥dy£í—H9o ¹n.ˆ	\×‰ËYWÐ‹›Y\ÜØYÙJNÂˆ[™ˆ[Z[™Ëœ™\ØÝYP—ÜÈHØÊÜ™\ØÝYPŠNÂ‚ˆ[™	H˜™\Ý™X\Â‚ˆ	Hˆ•S•SQWÐSSTÒTÈUÒ™S‘
™\ØÝYH[Z[™ÊH‚ˆ[Z[™Ëœ™\ØÝYWÝÝ[ÜÈH[Z[™Ëœ™\ØÝYPWÜÈ
È[Z[™Ëœ™\ØÝYP—ÜÎÂˆ	H‚ˆ	HKKKH:+â¹¥«y¥éyoåÎˆ9¢dùcl9."zf-¹«­Hˆ9d£9§ 9îâ:`"y¢êH
ŽNˆ9d*ÈZ[9`&z`"JHKKKBˆœš[Š	Èú-ëùo¡:`"y¢êWH˜]ÈIKŒÙŠ™X\ÏIY[IKŠHÛ[ÛÝIKŒÙŠ™X\ÏIY[IKŠHZ[IKŒÙŠ™X\ÏIY[IKŠW‰Ë‹‹‚ˆ™\ÝÜ˜]×Ù]’—Ùš[˜[™\ÝÜ˜]×Ù]™™X\ÚX›K™\ÝÜ˜]×Ù]œ[˜[WÝÝ[‹‹‚ˆ™\ÝÜÛ[ÛÝÙ]’—Ùš[˜[™\ÝÜÛ[ÛÝÙ]™™X\ÚX›K™\ÝÜÛ[ÛÝÙ]œ[˜[WÝÝ[‹‹‚ˆ™\ÝÛZ[Ù]’—Ùš[˜[™\ÝÛZ[Ù]™™X\ÚX›K™\ÝÛZ[Ù]œ[˜[WÝÝ[
NÂˆœš[Š	Èú-ëùo¡:`"y¢êWH8¦!H9§ 9îâ:`"y¢êNˆ	\È
IKŒÙ‹™X\ÚX›OIYY™”ØØ[OIKŒ™ŠW‰Ë‹‹‚ˆ™\ÝØÚÜÙ[—ÝYË™\ÝÛÜÝÙš[˜[™\Ý™X\Ë‹‹‚ˆÙ]Ü‘Y˜][
Ù™Ë	ÙY™šXÝ[TØØ[IËKŒ
JNÂ‚ˆ]H™\Ý]Ùš[˜[ÂˆÛÜÝH™\ÝÛÜÝÙš[˜[Âˆ]Z[ÈH™\Ý]Ùš[˜[Âˆ]Z[Ëš[\›˜[ÜÙX\˜ÚØÛÜÝH[\›˜[ÜÙX\˜ÚØÛÜÝÂˆ]Z[Ë˜ÚÜÙ[—Ü]Ý\HH™\ÝØÚÜÙ[—ÝYÎÈ	H:+¬9oez`"y¢êz-ëùo¡9ìnùg¢Â‚ˆ	Hˆ•S•SQWÐSSTÒTÈUÒ™Ì™H
™\ØÝYHÛÝ[È
È[Z[™È[È]Z[ÊH‚ˆ	H:+¨y¥l9cèùo¡ˆ:+éH[ˆ9§ 9îâ:aáùå*9æ¡:-ëùo¡9¦+ùd)¹k§ºfayn¥9å*9.¡ˆ™\ØÝYPHÈ™\ØÝYP‚ˆ	H
9/§y£kˆ™\ÝØÚÜÙ[—ÝYÈ9.+yæ¡	ÊÜ™\ØÐIÈÈ	ÊÜ™\ØÐ‰È9¨!ú+¬È9«ãÈ[ˆ9cåˆÌJBˆ	H9¬êˆ:/æy¦+È¹n¥9å*9b,9§ 9îâ:-ëùo¡¹æ¡9«(y¥l9.#y¦+È¹l'z+åH¹«(y¥l
9l'z+åy/a¹i,z-)yæ¡9.#z+¨Jxà ‚ˆ™\ØÝYWØWØÛÝ[HÝX›JÛÛZ[œÊ™\ÝØÚÜÙ[—ÝYË	Ü™\ØÐIÊJNÂˆ™\ØÝYWØ—ØÛÝ[HÝX›JÛÛZ[œÊ™\ÝØÚÜÙ[—ÝYË	Ü™\ØÐ‰ÊJNÂˆ™\ØÝYPWÜÝ]Ë˜YÜYHÙÚXØ[
™\ØÝYWØWØÛÝ[
NÂˆ™\ØÝYP—ÜÝ]Ë˜YÜYHÙÚXØ[
™\ØÝYWØ—ØÛÝ[
NÂ‚ˆ[Z[™ËÝ[ÜÈHØÊÝÝ[
NÂˆ[Z[™Ë›Ý\—ÜÈHX^
[Z[™ËÝ[ÜÈH[Z[™Ëš[š]X[^˜][Û—ÜÈH‹‹‚ˆ[Z[™ËœÙX\˜ÚÜÈH[Z[™ËÜ×ÜÈH[Z[™Ëœ™\ØÝYPWÜÈH[Z[™Ëœ™\ØÝYP—ÜÊNÂ‚ˆ]Z[Ë[Z[™ÈH[Z[™ÎÂˆ]Z[Ë˜Ø[™Y]WÜÝ]ÈHØ[™Y]WÜÝ]ÎÂˆ]Z[Ëœ™\ØÝYPWÜÝ]ÈH™\ØÝYPWÜÝ]ÎÂˆ]Z[Ëœ™\ØÝYP—ÜÝ]ÈH™\ØÝYP—ÜÝ]ÎÂˆ]Z[Ëœ™\ØÝYWØWØÛÝ[H™\ØÝYWØWØÛÝ[Âˆ]Z[Ëœ™\ØÝYWØ—ØÛÝ[H™\ØÝYWØ—ØÛÝ[Âˆ	H•S•SQWÐSSTÒTÈUÒ™Ì™HS‘‚ˆÝYÙWÙ]Z[Ëœ˜]ÈH™\ÝÜ˜]×Ù]ÂˆÝYÙWÙ]Z[ËœÛ[ÛÝH™\ÝÜÛ[ÛÝÙ]Âˆ	HÝYÙWÙ]Z[Ëœ™\Z\ˆ9mì¹éîúfi;ï":+¯º+¨ycæ9¦í;ï&”™\Z\ˆ9ª(ygeùmì¹b(:fi;ï"BˆÝYÙWÙ]Z[Ë˜ÚÜÙ[—Ü]Ý\HH™\ÝØÚÜÙ[—ÝYÎÂˆÝYÙWÙ]Z[Ëš[\›˜[ÜÙX\˜ÚØÛÜÝH[\›˜[ÜÙX\˜ÚØÛÜÝÂˆÝYÙWÙ]Z[Ë[Z[™ÈH[Z[™ÎÂˆÝYÙWÙ]Z[Ë˜Ø[™Y]WÜÝ]ÈHØ[™Y]WÜÝ]ÎÂˆÝYÙWÙ]Z[Ëœ™\ØÝYPWÜÝ]ÈH™\ØÝYPWÜÝ]ÎÂˆÝYÙWÙ]Z[Ëœ™\ØÝYP—ÜÝ]ÈH™\ØÝYP—ÜÝ]ÎÂ™[™‚‰IHOOOOOOOOOOOOOH9cà¹¥l9d$zaãÈ8¡¤ˆ9."yîí:-ëùo¡
9.#¹c§ùâb9k£9aj9. :!í
HOOOOOOOOOOOOOB‚‰IHOOOOOOOOOOOOOHŽH:/¡ybªNˆ9k¢yaj9keù«­z#­ùcåˆOOOOOOOOOOOOOB™[˜Ý[ÛˆˆHÙ]Ü‘Y˜][
Ë›˜[YK›
BˆYˆ\ÙšY[
Ë›˜[YJH	‰ˆš\Ù[\JËŠ›˜[YJJBˆˆHËŠ›˜[YJNÂˆ[ÙBˆˆH›Âˆ[™™[™