classdef PathPlanners < handle
% PathPlanners - 路径规划算法集合
%   包含多种基线算法用于对比实验:
%   1. Energy-A* (能耗加权A*)
%   2. Informed RRT* (采样优化RRT*)
%   3. ALA-based (人工旅鼠算法)
%   4. Greedy Heuristic (贪心启发式)

    properties
        env;            % 环境引用
        costModel;      % 代价模型引用
        maxH = 120;     % 最大飞行高度 (m)
        minH = 30;      % 最小飞行高度 (m)
        
        % ===== 统一计算预算 =====
        % 参数选择依据:
        %   - A*: 典型城市图200-400节点，3000预算绑绑有余
        %   - RRT*: 1000次迭代可找到较优路径，渐进最优特性保证质量
        %   - ALA: 1000次迭代通常已收敛
        %   - 时间预算: 5秒足够完成上述迭代
        timeBudget = 5;         % 时间预算 (秒/路径)
        nodeBudget = 3000;      % A* 节点扩展预算
        iterationBudget = 1000; % RRT*/ALA 迭代预算
        sampleBudget = 1000;    % RRT* 采样预算
    end
    
    methods
        function obj = PathPlanners(env, costModel)
            obj.env = env;
            obj.costModel = costModel;
            if ~isempty(env)
                obj.minH = max(obj.env.heightMap(:)) * 0.1 + 20;
            end
            % 从代价模型获取高度限制
            if ~isempty(costModel)
                if isprop(costModel, 'H_max') || isfield(costModel, 'H_max')
                    obj.maxH = costModel.H_max;
                end
                if isprop(costModel, 'H_min') || isfield(costModel, 'H_min')
                    obj.minH = max(obj.minH, costModel.H_min);
                end
            end
        end
        
        function setBudget(obj, timeBudget, nodeBudget, iterBudget)
            % 设置统一计算预算
            if nargin >= 2, obj.timeBudget = timeBudget; end
            if nargin >= 3, obj.nodeBudget = nodeBudget; end
            if nargin >= 4
                obj.iterationBudget = iterBudget;
                obj.sampleBudget = iterBudget;
            end
        end
        
        %% ==================== Energy-A* ====================
        function [path, cost, info] = energyAStar(obj, start, goal, t_start, hasPayload)
            % Energy-A*: 使用能耗作为代价的A*算法
            % 支持节点预算和时间预算限制
            if nargin < 5, hasPayload = true; end
            if nargin < 4, t_start = 0; end
            
            tic;
            startTime = tic;
            
            % 确保start和goal是行向量
            start = start(:)';
            goal = goal(:)';
            if length(start) < 3, start = [start, 60]; end
            if length(goal) < 3, goal = [goal, 60]; end
            
            nodes = obj.env.nodes;
            edges = obj.env.edges;
            nNodes = size(nodes, 1);
            
            % 找到最近的起点和终点节点
            startXY = [start(1), start(2)];
            goalXY = [goal(1), goal(2)];
            [~, startIdx] = min(pdist2(startXY, nodes(:,1:2)));
            [~, goalIdx] = min(pdist2(goalXY, nodes(:,1:2)));
            
            % 初始化
            g = inf(nNodes, 1);  % 实际代价
            f = inf(nNodes, 1);  % 估计总代价
            parent = zeros(nNodes, 1);
            heights = zeros(nNodes, 1);  % 记录到达每个节点的高度
            
            g(startIdx) = 0;
            heights(startIdx) = start(3);
            f(startIdx) = obj.heuristicEnergy(startIdx, goalIdx, nodes, hasPayload);
            
            openSet = startIdx;
            closedSet = false(nNodes, 1);
            nodesExpanded = 0;
            
            while ~isempty(openSet)
                % 检查预算限制
                nodesExpanded = nodesExpanded + 1;
                if nodesExpanded > obj.nodeBudget
                    break;  % 超出节点预算
                end
                if toc(startTime) > obj.timeBudget
                    break;  % 超出时间预算
                end
                
                % 选择f值最小的节点
                [~, minIdx] = min(f(openSet));
                current = openSet(minIdx);
                
                if current == goalIdx
                    break;
                end
                
                openSet(minIdx) = [];
                closedSet(current) = true;
                
                % 获取邻居
                neighbors = edges(edges(:,1) == current, 2);
                
                for i = 1:length(neighbors)
                    neighbor = neighbors(i);
                    
                    % 确保neighbor是标量索引
                    if ~isscalar(neighbor)
                        neighbor = neighbor(1);
                    end
                    
                    if closedSet(neighbor)
                        continue;
                    end
                    
                    % ★ NFZ/动态障碍碰撞检测: 跳过危险节点
                    neighX = double(nodes(neighbor, 1));
                    neighY = double(nodes(neighbor, 2));
                    neighborZ = double(nodes(neighbor, 3));
                    targetH_pre = max(neighborZ + 5, obj.minH);
                    targetH_pre = min(targetH_pre, obj.maxH);
                    
                    if ~isempty(obj.env.dynObstacles)
                        % NFZ 检测: 保守策略 — 忽略时间窗, 视所有 NFZ 为永久激活
                        %  原因: A* 规划时用 t_start 检查, 但无人机到达该节点
                        %        的真实时刻远大于 t_start, NFZ 可能已经激活
                        skipNode = false;
                        for nfi = 1:length(obj.env.dynObstacles.tempNFZ)
                            nfz = obj.env.dynObstacles.tempNFZ(nfi);
                            if ~nfz.active, continue; end
                            % ★ 不检查时间窗, 只检查空间
                            dh = sqrt((neighX-nfz.center(1))^2 + (neighY-nfz.center(2))^2);
                            if dh < nfz.radius * 1.15 && targetH_pre >= nfz.height(1) && targetH_pre <= nfz.height(2)
                                skipNode = true; break;
                            end
                        end
                        if skipNode, continue; end
                        
                        % 动态障碍检测 (这些确实依赖时刻, 用 t_start 粗估)
                        if obj.env.dynObstacles.checkCollision(...
                                neighX, neighY, targetH_pre, t_start)
                            continue;
                        end
                    end
                    
                    % 计算到达邻居的高度
                    currentH = heights(current);
                    if currentH == 0
                        currentH = start(3);
                    end
                    
                    targetH = max(neighborZ + 5, obj.minH);
                    targetH = min(targetH, obj.maxH);
                    
                    % 高度变化约束 - 分别获取坐标确保是标量
                    currX = double(nodes(current, 1)); 
                    currY = double(nodes(current, 2));
                    dist2D = sqrt((neighX - currX)^2 + (neighY - currY)^2);
                    
                    maxClimb = dist2D * tan(pi/6);  % 最大爬升角30度
                    if targetH > currentH + maxClimb
                        targetH = currentH + maxClimb;
                    elseif targetH < currentH - maxClimb
                        targetH = currentH - maxClimb;
                    end
                    
                    % 确保targetH是标量
                    targetH = double(targetH(1));
                    
                    % 计算能耗代价
                    p1 = [currX, currY, currentH];
                    p2 = [neighX, neighY, targetH];
                    edgeCost = obj.computeEdgeEnergy(p1, p2, t_start, hasPayload);
                    
                    tentative_g = g(current) + edgeCost;
                    
                    if tentative_g < g(neighbor)
                        parent(neighbor) = current;
                        heights(neighbor) = targetH;
                        g(neighbor) = tentative_g;
                        f(neighbor) = g(neighbor) + obj.heuristicEnergy(neighbor, goalIdx, nodes, hasPayload);
                        
                        if ~ismember(neighbor, openSet)
                            openSet = [openSet; neighbor];
                        end
                    end
                end
            end
            
            % 重建路径
            if parent(goalIdx) == 0 && startIdx ~= goalIdx
                path = [start(1:3); goal(1:3)];
                cost = inf;
                info.success = false;
                info.reachedGoal = false;
                info.details = struct();
            else
                pathIdx = goalIdx;
                pathNodes = goalIdx;
                while parent(pathIdx) ~= 0
                    pathIdx = parent(pathIdx);
                    pathNodes = [pathIdx; pathNodes];
                end
                
                path = zeros(length(pathNodes), 3);
                for i = 1:length(pathNodes)
                    idx = pathNodes(i);
                    px = double(nodes(idx, 1));
                    py = double(nodes(idx, 2));
                    pz = heights(idx);
                    path(i,:) = [px, py, pz];
                end
                path(1,:) = start(1:3);
                path(end,:) = goal(1:3);
                
                [cost, pathDetails] = obj.costModel.evaluatePath(path, t_start, hasPayload);
                info.success = true;
                info.reachedGoal = true;
                info.details = pathDetails;
            end
            
            info.time = toc;
            info.nodesExpanded = nodesExpanded;
            info.nodeBudget = obj.nodeBudget;
            info.timeBudget = obj.timeBudget;
            info.budgetExhausted = (nodesExpanded >= obj.nodeBudget) || (info.time >= obj.timeBudget);
            info.algorithm = 'Energy-A*';
        end
        
        function h = heuristicEnergy(obj, nodeIdx, goalIdx, nodes, hasPayload)
            % 能耗启发式: 基于直线距离的最小能耗估计
            dist = norm(nodes(nodeIdx,1:2) - nodes(goalIdx,1:2));
            P_hover = obj.costModel.getHoverPower(hasPayload);
            v = obj.costModel.v_cruise;
            h = P_hover * 0.8 * dist / v / 3600;  % 乐观估计
        end
        
        function energy = computeEdgeEnergy(obj, p1, p2, t_start, hasPayload)
            % 计算边的能耗
            % 确保p1和p2都是1×3的行向量
            p1 = p1(:)';
            p2 = p2(:)';
            if length(p1) < 3, p1 = [p1, 60]; end
            if length(p2) < 3, p2 = [p2, 60]; end
            p1 = p1(1:3);
            p2 = p2(1:3);
            
            [~, det] = obj.costModel.evaluatePath([p1; p2], t_start, hasPayload);
            % 风险权重从 0.1 提到 5.0, 让 A* 边代价对 NFZ/障碍敏感
            energy = det.E_total + det.C_climb + det.R_dynamic * 5.0;
            % 如果该段路径不可行 (碰撞/高度违规), 大幅增加代价
            if ~det.feasible
                energy = energy + 100;
            end
        end
        
        %% ==================== Space-Time Energy-A* ====================
        function [path, cost, info] = timeExpandedEnergyAStar(obj, start, goal, ...
                t_start, hasPayload, timeStep, timeHorizon)
            % Time-expanded Energy-A*: the search state is (graph node,time bin).
            % Every edge is evaluated at its propagated arrival time by the
            % unified evaluator, including all time-dependent constraints.
            if nargin < 7 || isempty(timeHorizon), timeHorizon = 300; end
            if nargin < 6 || isempty(timeStep), timeStep = 2; end
            if nargin < 5, hasPayload = true; end
            if nargin < 4, t_start = 0; end
            validateattributes(timeStep,{'numeric'},{'scalar','positive','finite'});
            validateattributes(timeHorizon,{'numeric'},{'scalar','positive','finite'});

            startTimer = tic;
            start = start(:)'; goal = goal(:)';
            if numel(start) < 3, start = [start,60]; end
            if numel(goal) < 3, goal = [goal,60]; end

            nodes = obj.env.nodes;
            edges = obj.env.edges;
            nNodes = size(nodes,1);
            [~,startIdx] = min(pdist2(start(1:2),nodes(:,1:2)));
            [~,goalIdx] = min(pdist2(goal(1:2),nodes(:,1:2)));

            nBins = floor(timeHorizon/timeStep)+1;
            nStates = nNodes*nBins;
            stateId = @(nodeIdx,binIdx) nodeIdx+(binIdx-1)*nNodes;
            adjacency = sparse(double(edges(:,1)),double(edges(:,2)),true,nNodes,nNodes);

            g = inf(nStates,1);
            f = inf(nStates,1);
            arrival = inf(nStates,1);
            altitude = nan(nStates,1);
            parent = zeros(nStates,1,'uint32');
            closed = false(nStates,1);

            startState = stateId(startIdx,1);
            g(startState) = 0;
            arrival(startState) = t_start;
            altitude(startState) = start(3);
            f(startState) = obj.heuristicUnified(startIdx,goalIdx,nodes,hasPayload);
            openSet = startState;

            staw�M�����k�w��`      if pt_sub(3) > obj.H_max
                        pen = (pt_sub(3) - obj.H_max) / 10 / nSub;
                        P_seg_acc = P_seg_acc + pen;
                        Ph_acc    = Ph_acc    + pen;
                    end

                    E_seg_acc = E_seg_acc + E_sub;
                    T_seg_acc = T_seg_acc + dt_sub;
                    t_sub     = t_sub + dt_sub;
                end  % 子采样循环

                % ==============================================================
                % 【修复】只检查段起点 p1 的高度约束 (不再同时检查 p2)
                % 原因: 检查 p2 会导致中间路径点被相邻两段各计一次，形成重复收费。
                %       原版对 N=20 的平滑路径, 18 个中间点各多收一次 (min_h-z)/10,
                %       lambda=100 可将 J 虚增数百乃至逾千。
                %       修复: 每段仅检查 p1；最后路径点 pathPts(N) 在段循环后单独检查。
                % ==============================================================
                pt_ep = p1;   % 只检查 p1
                if ~isempty(obj.staticMap)
                    px = max(1, min(size(obj.staticMap,1), round(pt_ep(1))));
                    py = max(1, min(size(obj.staticMap,2), round(pt_ep(2))));
                    ground_h_ep = obj.staticMap(px, py);
                else
                    ground_h_ep = 0;
                end
                min_ep = max(obj.H_min, ground_h_ep + obj.H_clearance);
                if pt_ep(3) < min_ep
                    pen = (min_ep - pt_ep(3)) / 10;
                    P_seg_acc = P_seg_acc + pen;
                    Ph_acc    = Ph_acc    + pen;
                end
                if pt_ep(3) > obj.H_max
                    pen = (pt_ep(3) - obj.H_max) / 10;
                    P_seg_acc = P_seg_acc + pen;
                    Ph_acc    = Ph_acc    + pen;
                end

                % ---- 爬升代价 ----
                C_climb_seg(k) = abs(dz) * 0.01;

                % ---- 写入段数组 ----
                E_segments(k)  = E_seg_acc;
                T_segments(k)  = T_seg_acc;
                R_risk_seg(k)  = R_seg_acc;
                Penalty_seg(k) = P_seg_acc;
                Ph_seg(k)      = Ph_acc;
                Ps_seg(k)      = Ps_acc;
                Pd_seg(k)      = Pd_acc;

                t_current = t_sub;
                t_arrivals(k+1) = t_current;
            end  % 段循环

            % ==============================================================
            % 【修复】单独检查最后一个路径点 pathPts(N) (修复端点遗漏)
            % ==============================================================
            pt_last = pathPts(N, :);
            Ph_last = 0; P_last = 0;
            if ~isempty(obj.staticMap)
                px = max(1, min(size(obj.staticMap,1), round(pt_last(1))));
                py = max(1, min(size(obj.staticMap,2), round(pt_last(2))));
                ground_h_last = obj.staticMap(px, py);
            else
                ground_h_last = 0;
            end
            min_last = max(obj.H_min, ground_h_last + obj.H_clearance);
            if pt_last(3) < min_last
                pen = (min_last - pt_last(3)) / 10;
                P_last = P_last + pen;
                Ph_last = Ph_last + pen;
            end
            if pt_last(3) > obj.H_max
                pen = (pt_last(3) - obj.H_max) / 10;
                P_last = P_last + pen;
                Ph_last = Ph_last + pen;
            end

            % ==============================================================
            %  汇总
            % ==============================================================
            E_total   = sum(E_segments);
            T_total   = sum(T_segments);
            C_climb   = sum(C_climb_seg);
            R_dynamic = sum(R_risk_seg);
            Penalty   = sum(Penalty_seg) + P_last;

            pen_height  = sum(Ph_seg) + Ph_last;
            pen_static  = sum(Ps_seg);
            pen_dyn     = sum(Pd_seg);

            % 电池惩罚
            pen_battery = 0;
            if E_total > obj.E_batt * 0.9
                pen_battery = (E_total - obj.E_batt * 0.9) / 10;
                Penalty     = Penalty + pen_battery;
            end

            % ---- NFZ hard-constraint penalty ----
            % Uses the same distance-based interval as the static and moving-
            % obstacle checks, with interpolation between recursive arrivals.
            pen_nfz = 0;
            total_nfz_subsamples = 0;
            if ~isempty(obj.dynObstacles) && isfield(obj.dynObstacles,'tempNFZ')
                for k = 1:N-1
                    p1n = pathPts(k,:);   t1n = t_arrivals(k);
                    p2n = pathPts(k+1,:); t2n = t_arrivals(k+1);
                    d_nfz = norm(p2n-p1n);
                    if d_nfz < 0.01, continue; end
                    NFZ_NSUB = max(MIN_SUB,ceil(d_nfz/SUB_SPACING));
                    total_nfz_subsamples = total_nfz_subsamples + NFZ_NSUB;
                    for s = 1:NFZ_NSUB
                        frac = (s-0.5)/NFZ_NSUB;
                        ptn = p1n + frac*(p2n-p1n);
                        ttn = t1n + frac*(t2n-t1n);
                        for ni = 1:length(obj.dynObstacles.tempNFZ)
                            nfz = obj.dynObstacles.tempNFZ(ni);
                            if ~nfz.active || ttn < nfz.t_start || ttn > nfz.t_end
                                continue;
                            end
                            dh = norm(ptn(1:2)-nfz.center);
                            if dh < nfz.radius && ...
                                    ptn(3) >= nfz.height(1) && ptn(3) <= nfz.height(2)
                                pen_nfz = pen_nfz + (1-dh/nfz.radius)/NFZ_NSUB;
                            end
                        end
                    end
                end
            end
            Penalty = Penalty + pen_nfz;

            % 高度违规计数 (仅统计, 不加入 Penalty)
            heightViolations = 0;
            for k = 1:N
                pt = pathPts(k,:);
                if ~isempty(obj.staticMap)
                    px = max(1, min(size(obj.staticMap,1), round(pt(1))));
                    py = max(1, min(size(obj.staticMap,2), round(pt(2))));
                    ground_h = obj.staticMap(px, py);
                else
                    ground_h = 0;
                end
                min_allowed = max(obj.H_min, ground_h + obj.H_clearance);
                if pt(3) < min_allowed || pt(3) > obj.H_max
                    heightViolations = heightViolations + 1;
                end
            end

            % ---- 统一代价公式 (与原版完全一致) ----
            J = obj.w_energy * E_total + ...
                obj.w_time * (T_total / 60) + ...
                obj.w_climb * C_climb + ...
                obj.w_risk * R_dynamic + ...
                obj.lambda_penalty * Penalty;

            % ==============================================================
            %  NFZ 穿越惩罚 (v12: 已在 J 公式前并入 Penalty/penalty_total,
            %   见上方 pen_nfz; 此处不再单独诊断计算)
            % ==============================================================

            % ==============================================================
            %  输出 details (目标1: 完整代价分解)
            % ==============================================================
            details.J_final                   = J;
            details.E_total                   = E_total;
            details.T_total                   = T_total;
            details.C_climb                   = C_climb;
            details.R_dynamic                 = R_dynamic;
            details.penalty_total             = Penalty;
            details.penalty_height            = pen_height;
            details.penalty_static_collision  = pen_static;
            details.penalty_dynamic_collision = pen_dyn;
            details.penalty_battery           = pen_battery;
            details.penalty_nfz               = pen_nfz;   % v12: NFZ 硬约束, 已计入 penalty_total/J
            % 兼容旧字段名: 现等于进入 J 的 NFZ 硬罚 (不再是"仅诊断")
            details.NFZ_penalty               = pen_nfz;
            details.turn_or_smoothness_penalty= 0;   % 仅 evalRA_v2 中
            details.wind_lookahead_penalty    = 0;   % 仅 evalRA_v2 中
            details.repair_penalty            = 0;   % 通过阶段对比诊断
            % 可行性与统计
            details.feasible = ~(pen_height>0 || pen_static>0 || pen_dyn>0 || ...
                pen_battery>0 || pen_nfz>0);
            details.heightViolations          = heightViolations;
            details.E_segments                = E_segments;
            details.T_segments                = T_segments;
            details.t_end                     = t_current;
            details.SoC_end                   = 1 - E_total / obj.E_batt;
            details.t_arrivals                = t_arrivals;
            details.collision_sample_spacing_m= SUB_SPACING;
            details.minimum_collision_samples = MIN_SUB;
            details.total_collision_subsamples= total_subsamples;
            details.total_nfz_subsamples      = total_nfz_subsamples;
            % ★ 段级动态碰撞定位 (供 RescueA 直接复用, 避免重新扫描)
            %   dyn_col_segs(k) = Pd_seg(k): 段 k 的动态碰撞惩罚 (>0 表示有碰撞)
            %   结合 t_arrivals 可精确重建碰撞子点时刻和位置
            details.dyn_col_segs              = Pd_seg;
        end

        %% ================================================================
        %%  evaluateMultiTask (与原版完全一致)
        %% ================================================================
        function [J_total, task_details] = evaluateMultiTask(obj, paths, t_starts, payloads)
            nTasks = length(paths);
            J_total = 0;
            task_details = cell(nTasks, 1);
            for i = 1:nTasks
                [J_i, det_i] = obj.evaluatePath(paths{i}, t_starts(i), payloads(i));
                J_total = J_total + J_i;
                task_details{i} = det_i;
            end
        end

        %% ================================================================
        %%  evaluateRiskAtPoint (与原版完全一致)
        %% ================================================================
        function risk = evaluateRiskAtPoint(obj, pt, t)
            risk = 0;
            if isempty(obj.dynObstacles), return; end
            try
                allObs = obj.dynObstacles.getAllPositions(t);
                for i = 1:size(allObs, 1)
                    d = norm(pt - allObs(i, 1:3));
                    r_safe = allObs(i, 4) * 2;
                    if d < r_safe
                        risk = risk + (1 - d/r_safe)^2;
                    end
                end
                for i = 1:length(obj.dynObstacles.tempNFZ)
                    nfz = obj.dynObstacles.tempNFZ(i);
                    if ~nfz.active || t < nfz.t_start || t > nfz.t_end
                        continue;
                    end
                    dh = sqrt((pt(1)-nfz.center(1))^2 + (pt(2)-nfz.center(2))^2);
                    if dh < nfz.radius * 1.5 && ...
                            pt(3) >= nfz.height(1) && pt(3) <= nfz.height(2)
                        risk = risk + 5 * max(0, 1 - dh/nfz.radius/1.5);
                    end
                end
            catch; end
        end

        %% ================================================================
        %%  checkCollision (与原版完全一致)
        %% ================================================================
        function collision = checkCollision(obj, pt, t)
            collision = false;
            if isempty(obj.dynObstacles), return; end
            try
                collision = obj.dynObstacles.checkCollision(pt(1), pt(2), pt(3), t);
            catch; end
        end

        %% ================================================================
        %%  checkStaticCollision (legacy sampled helper)
        %% ================================================================
        function collision = checkStaticCollision(obj, p1, p2)
            collision = false;
            if isempty(obj.staticMap), return; end
            nCheck   = max(10, round(norm(p2(1:2)-p1(1:2)) / 10));
            MAP_SIZE = size(obj.staticMap, 1);
            for i = 0:nCheck
                t_i = i / nCheck;
                pt = p1 + t_i * (p2 - p1);
                rx = max(1, min(MAP_SIZE, round(pt(1))));
                ry = max(1, min(MAP_SIZE, round(pt(2))));
                if pt(3) < obj.staticMap(rx, ry) + 3
                    collision = true;
                    return;
                end
            end
        end

        %% ================================================================
        %%  getHoverPower (与原版完全一致)
        %% ================================================================
        function P_hover = getHoverPower(obj, hasPayload)
            if hasPayload
                m = obj.m_frame + obj.m_payload;
            else
                m = obj.m_frame;
            end
            W_val      = m * 9.81;
            eta_val    = obj.eta_motor * obj.eta_prop;
            A_disc_val = obj.n_rotor * pi * obj.r_prop^2;
            P_hover    = W_val^1.5 / sqrt(2 * obj.rho_air * A_disc_val) / eta_val;
        end
    end

    methods (Access = private)
        function d = emptyDetails(obj, t_start) %#ok<INUSL>
            d.J_final                   = inf;
            d.E_total                   = inf;
            d.T_total                   = inf;
            d.C_climb                   = inf;
            d.R_dynamic                 = inf;
            d.penalty_total             = inf;
            d.penalty_height            = inf;
            d.penalty_static_collision  = inf;
            d.penalty_dynamic_collision = inf;
            d.penalty_battery           = 0;
            d.penalty_nfz               = 0;
            d.NFZ_penalty               = 0;
            d.turn_or_smoothness_penalty= 0;
            d.wind_lookahead_penalty    = 0;
            d.repair_penalty            = 0;
            d.feasible                  = false;
            d.heightViolations          = 0;
            d.E_segments                = inf;
            d.T_segments                = inf;
            d.t_end                     = t_start;
            d.SoC_end                   = 0;
            d.t_arrivals                = t_start;
            d.collision_sample_spacing_m= obj.collision_sample_spacing;
            d.minimum_collision_samples = obj.min_collision_samples;
            d.total_collision_subsamples= 0;
            d.total_nfz_subsamples      = 0;
        end
    end
end
