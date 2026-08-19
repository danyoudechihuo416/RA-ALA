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

            statesExpanded = 0;
            reachedState = 0;
            stoppedBy = 'open_set_empty';

            while ~isempty(openSet)
                if statesExpanded >= obj.nodeBudget
                    stoppedBy = 'state_budget'; break;
                end
                if toc(startTimer) >= obj.timeBudget
                    stoppedBy = 'wall_clock_timeout'; break;
                end

                if any(~isfinite(openSet) | openSet < 1 | openSet > nStates | openSet ~= floor(openSet))
                    error('PathPlanners:InvalidSpaceTimeState', ...
                        'Invalid state ID in open set: %s',mat2str(openSet));
                end
                [~,k] = min(f(openSet));
                currentState = openSet(k);
                openSet(k) = [];
                openSet = openSet(:);
                if closed(currentState), continue; end
                closed(currentState) = true;
                statesExpanded = statesExpanded+1;

                currentNode = mod(currentState-1,nNodes)+1;
                if currentNode == goalIdx
                    reachedState = currentState;
                    stoppedBy = 'goal_reached'; break;
                end

                currentTime = arrival(currentState);
                currentH = altitude(currentState);
                neighbors = find(adjacency(currentNode,:));
                for i = 1:numel(neighbors)
                    neighbor = neighbors(i);
                    currX = double(nodes(currentNode,1));
                    currY = double(nodes(currentNode,2));
                    neighX = double(nodes(neighbor,1));
                    neighY = double(nodes(neighbor,2));

                    targetH = max(double(nodes(neighbor,3))+5,obj.minH);
                    targetH = min(targetH,obj.maxH);
                    dist2D = hypot(neighX-currX,neighY-currY);
                    maxClimb = dist2D*tan(pi/6);
                    targetH = min(max(targetH,currentH-maxClimb),currentH+maxClimb);

                    p1 = [currX,currY,currentH];
                    p2 = [neighX,neighY,targetH];
                    [edgeCost,edgeDetails] = obj.costModel.evaluatePath( ...
                        [p1;p2],currentTime,hasPayload);
                    if ~isfinite(edgeCost) || ~edgeDetails.feasible || ...
                            ~isfinite(edgeDetails.T_total) || edgeDetails.T_total <= 0
                        continue;
                    end

                    nextTime = currentTime+edgeDetails.T_total;
                    nextBin = floor((nextTime-t_start)/timeStep)+1;
                    if nextBin < 1 || nextBin > nBins, continue; end
                    nextState = stateId(neighbor,nextBin);
                    if closed(nextState), continue; end

                    tentativeG = g(currentState)+edgeCost;
                    if tentativeG < g(nextState)
                        g(nextState) = tentativeG;
                        arrival(nextState) = nextTime;
                        altitude(nextState) = targetH;
                        parent(nextState) = uint32(currentState);
                        f(nextState) = tentativeG+obj.heuristicUnified( ...
                            neighbor,goalIdx,nodes,hasPayload);
                        if ~ismember(nextState,openSet)
                            if isempty(openSet)
                                openSet = nextState;
                            else
                                openSet(end+1,1) = nextState; %#ok<AGROW>
                            end
                        end
                    end
                end
            end

            if reachedState == 0
                path = [start(1:3);goal(1:3)];
                cost = inf;
                details = struct();
                pathArrivalTimes = [];
                success = false;
            else
                statePath = reachedState;
                s = reachedState;
                while parent(s) ~= 0
                    s = double(parent(s));
                    statePath = [s;statePath]; %#ok<AGROW>
                end
                path = zeros(numel(statePath),3);
                pathArrivalTimes = zeros(numel(statePath),1);
                for i = 1:numel(statePath)
                    nodeIdx = mod(statePath(i)-1,nNodes)+1;
                    path(i,:) = [double(nodes(nodeIdx,1:2)),altitude(statePath(i))];
                    pathArrivalTimes(i) = arrival(statePath(i));
                end
                path(1,:) = start(1:3);
                path(end,:) = goal(1:3);
                [cost,details] = obj.costModel.evaluatePath(path,t_start,hasPayload);
                success = isfinite(cost) && details.feasible;
            end

            elapsed = toc(startTimer);
            info = struct();
            info.success = success;
            info.reachedGoal = reachedState ~= 0;
            info.details = details;
            info.time = elapsed;
            info.nodesExpanded = statesExpanded;
            info.statesExpanded = statesExpanded;
            info.nodeBudget = obj.nodeBudget;
            info.timeBudget = obj.timeBudget;
            info.budgetExhausted = strcmp(stoppedBy,'state_budget') || ...
                strcmp(stoppedBy,'wall_clock_timeout');
            info.algorithm = 'ST-EA*';
            info.temporalState = true;
            info.timeStep = timeStep;
            info.timeHorizon = timeHorizon;
            info.timeBins = nBins;
            info.arrivalTimesSearch = pathArrivalTimes;
            info.stopReason = stoppedBy;
        end

        function h = heuristicUnified(obj,nodeIdx,goalIdx,nodes,hasPayload)
            % Optimistic lower bound in the units of the reported composite J.
            dist = norm(double(nodes(nodeIdx,1:2))-double(nodes(goalIdx,1:2)));
            P_hover = obj.costModel.getHoverPower(hasPayload);
            v = obj.costModel.v_cruise;
            energyLB = P_hover*0.8*dist/v/3600;
            timeLBMin = (dist/v)/60;
            h = obj.costModel.w_energy*energyLB+obj.costModel.w_time*timeLBMin;
        end
        %% ==================== Informed RRT* ====================
        function [path, cost, info] = informedRRTStar(obj, start, goal, t_start, hasPayload, maxIter)
            % Informed RRT*: 采样优化的RRT*算法
            % 使用统一迭代预算和时间预算
            if nargin < 6 || isempty(maxIter), maxIter = obj.iterationBudget; end
            if nargin < 5, hasPayload = true; end
            if nargin < 4, t_start = 0; end

            tic;
            startTime = tic;

            % 参数
            stepSize = 30;
            goalBias = 0.1;
            rewireRadius = 60;

            % 初始化树
            tree.nodes = start;
            tree.parents = 0;
            tree.costs = 0;

            bestCost = inf;
            bestPath = [];
            actualIter = 0;

            for iter = 1:maxIter
                actualIter = iter;

                % 检查时间预算
                if toc(startTime) > obj.timeBudget
                    break;
                end

                % 采样
                if rand < goalBias || (bestCost < inf && rand < 0.3)
                    if bestCost < inf
                        % Informed sampling within ellipse
                        sample = obj.sampleInformedEllipse(start, goal, bestCost);
                    else
                        sample = goal;
                    end
                else
                    sample = obj.randomSample();
                end

                % 找最近节点
                [nearestIdx, nearestNode] = obj.nearest(tree.nodes, sample);

                % 扩展
                direction = sample - nearestNode;
                dist = norm(direction);
                if dist > stepSize
                    direction = direction / dist * stepSize;
                end
                newNode = nearestNode + direction;

                % 高度约束
                newNode(3) = max(obj.minH, min(obj.maxH, newNode(3)));
                ex = max(1, min(obj.env.MAP_SIZE, round(newNode(1))));
                ey = max(1, min(obj.env.MAP_SIZE, round(newNode(2))));
                minZ = obj.env.heightMap(ex, ey) + 5;
                newNode(3) = max(newNode(3), minZ);

                % 碰撞检测
                if obj.checkCollisionFree(nearestNode, newNode, t_start)
                    % 找邻近节点
                    nearNodes = obj.findNear(tree.nodes, newNode, rewireRadius);

                    % 选择最优父节点
                    minCost = tree.costs(nearestIdx) + obj.computeEdgeEnergy(nearestNode, newNode, t_start, hasPayload);
                    minParent = nearestIdx;

                    for i = 1:length(nearNodes)
                        nIdx = nearNodes(i);
                        if obj.checkCollisionFree(tree.nodes(nIdx,:), newNode, t_start)
                            newCost = tree.costs(nIdx) + obj.computeEdgeEnergy(tree.nodes(nIdx,:), newNode, t_start, hasPayload);
                            if newCost < minCost
                                minCost = newCost;
                                minParent = nIdx;
                            end
                        end
                    end

                    % 添加节点
                    tree.nodes = [tree.nodes; newNode];
                    tree.parents = [tree.parents; minParent];
                    tree.costs = [tree.costs; minCost];
                    newIdx = size(tree.nodes, 1);

                    % 重连
                    for i = 1:length(nearNodes)
                        nIdx = nearNodes(i);
                        if nIdx ~= minParent
                            newCost = minCost + obj.computeEdgeEnergy(newNode, tree.nodes(nIdx,:), t_start, hasPayload);
                            if newCost < tree.costs(nIdx) && obj.checkCollisionFree(newNode, tree.nodes(nIdx,:), t_start)
                                tree.parents(nIdx) = newIdx;
                                tree.costs(nIdx) = newCost;
                            end
                        end
                    end

                    % 检查是否到达目标
                    if norm(newNode - goal) < stepSize
                        if obj.checkCollisionFree(newNode, goal, t_start)
                            goalCost = minCost + obj.computeEdgeEnergy(newNode, goal, t_start, hasPayload);
                            if goalCost < bestCost
                                bestCost = goalCost;
                                % 重建路径
                                pathNodes = [goal];
                                idx = newIdx;
                                while idx ~= 0
                                    pathNodes = [tree.nodes(idx,:); pathNodes];
                                    idx = tree.parents(idx);
                                end
                                bestPath = pathNodes;
                            end
                        end
                    end
                end
            end

            if isempty(bestPath)
                path = [start; goal];
                cost = inf;
                info.success = false;
                info.reachedGoal = false;
                info.details = struct();
            else
                path = bestPath;
                [cost, pathDetails] = obj.costModel.evaluatePath(path, t_start, hasPayload);
                info.success = true;
                info.reachedGoal = true;
                info.details = pathDetails;
            end

            info.time = toc;
            info.nodesCreated = size(tree.nodes, 1);
            info.iterations = actualIter;
            info.iterationBudget = maxIter;
            info.timeBudget = obj.timeBudget;
            info.budgetExhausted = (actualIter >= maxIter) || (info.time >= obj.timeBudget);
            info.algorithm = 'Informed-RRT*';
        end

        function sample = randomSample(obj)
            x = rand * obj.env.MAP_SIZE;
            y = rand * obj.env.MAP_SIZE;
            z = obj.minH + rand * (obj.maxH - obj.minH);
            sample = [x, y, z];
        end

        function sample = sampleInformedEllipse(obj, start, goal, cBest)
            % 在椭圆内采样
            cMin = norm(goal - start);
            if cBest <= cMin
                sample = obj.randomSample();
                return;
            end

            center = (start + goal) / 2;
            a = cBest / 2;
            c = cMin / 2;
            b = sqrt(a^2 - c^2);

            % 随机采样椭圆内的点
            theta = rand * 2 * pi;
            r = sqrt(rand);

            % 椭圆坐标系
            direction = (goal - start) / cMin;
            perpXY = [-direction(2), direction(1), 0];
            perpXY = perpXY / (norm(perpXY) + 0.001);

            sample = center + a * r * cos(theta) * direction + b * r * sin(theta) * perpXY;
            sample(3) = obj.minH + rand * (obj.maxH - obj.minH);

            % 边界约束
            sample(1) = max(1, min(obj.env.MAP_SIZE, sample(1)));
            sample(2) = max(1, min(obj.env.MAP_SIZE, sample(2)));
        end

        function [idx, node] = nearest(obj, nodes, sample)
            dists = sqrt(sum((nodes - sample).^2, 2));
            [~, idx] = min(dists);
            node = nodes(idx, :);
        end

        function nearIdx = findNear(obj, nodes, sample, radius)
            dists = sqrt(sum((nodes - sample).^2, 2));
            nearIdx = find(dists < radius);
        end

        function free = checkCollisionFree(obj, p1, p2, t)
            free = true;
            nCheck = max(5, round(norm(p2-p1) / 10));
            for i = 0:nCheck
                pt = p1 + (i/nCheck) * (p2 - p1);
                ex = max(1, min(obj.env.MAP_SIZE, round(pt(1))));
                ey = max(1, min(obj.env.MAP_SIZE, round(pt(2))));
                if pt(3) < obj.env.heightMap(ex, ey) + 3
                    free = false;
                    return;
                end
                if ~isempty(obj.env.dynObstacles)
                    % 动态障碍 (依赖时刻)
                    if obj.env.dynObstacles.checkCollision(pt(1), pt(2), pt(3), t)
                        free = false;
                        return;
                    end
                    % ★ NFZ 保守检测: 忽略时间窗, 只检查空间
                    for nfi = 1:length(obj.env.dynObstacles.tempNFZ)
                        nfz = obj.env.dynObstacles.tempNFZ(nfi);
                        if ~nfz.active, continue; end
                        dh = sqrt((pt(1)-nfz.center(1))^2 + (pt(2)-nfz.center(2))^2);
                        if dh < nfz.radius * 1.15 && pt(3) >= nfz.height(1) && pt(3) <= nfz.height(2)
                            free = false;
                            return;
                        end
                    end
                end
            end
        end

        %% ==================== ALA-based Planner ====================
        function [path, cost, info] = ALAPlanner(obj, start, goal, t_start, hasPayload, maxIter)
            % ALA-based: 使用人工旅鼠算法优化路径
            % 使用统一迭代预算和时间预算
            if nargin < 6 || isempty(maxIter)
                % 将迭代预算转换为ALA的代数 (每代约100次评估)
                maxIter = max(30, round(obj.iterationBudget / 100));
            end
            if nargin < 5, hasPayload = true; end
            if nargin < 4, t_start = 0; end

            tic;
            startTime = tic;

            % 初始路径: 通过贪心搜索生成
            initPath = obj.greedyPath(start, goal, t_start);

            % 路径参数化: 中间航路点坐标
            nWaypoints = min(10, max(3, round(norm(goal(1:2)-start(1:2)) / 100)));

            % 初始化种群
            popSize = 15;
            dim = nWaypoints * 3;

            % 从初始路径采样生成初始种群
            pop = zeros(popSize, dim);
            for i = 1:popSize
                if i == 1 && size(initPath,1) >= nWaypoints+2
                    % 第一个个体使用初始路径
                    indices = round(linspace(2, size(initPath,1)-1, nWaypoints));
                    for j = 1:nWaypoints
                        pop(i, (j-1)*3+1:j*3) = initPath(indices(j), :);
                    end
                else
                    % 随机扰动
                    for j = 1:nWaypoints
                        t_ratio = j / (nWaypoints + 1);
                        basePos = start + t_ratio * (goal - start);
                        pop(i, (j-1)*3+1) = basePos(1) + (rand-0.5) * 200;
                        pop(i, (j-1)*3+2) = basePos(2) + (rand-0.5) * 200;
                        pop(i, (j-1)*3+3) = obj.minH + rand * (obj.maxH - obj.minH);
                    end
                end
            end

            % 边界
            lb = repmat([1, 1, obj.minH], 1, nWaypoints);
            ub = repmat([obj.env.MAP_SIZE, obj.env.MAP_SIZE, obj.maxH], 1, nWaypoints);

            % 评估函数
            evalFcn = @(x) obj.evaluatePathFromVector(x, start, goal, nWaypoints, t_start, hasPayload);

            % ALA优化
            fitness = zeros(popSize, 1);
            for i = 1:popSize
                fitness(i) = evalFcn(pop(i,:));
            end

            [bestFit, bestIdx] = min(fitness);
            bestPos = pop(bestIdx, :);
            actualIter = 0;
            totalEvals = popSize;  % 初始评估

            for iter = 1:maxIter
                actualIter = iter;

                % 检查时间预算
                if toc(startTime) > obj.timeBudget
                    break;
                end

                theta = 2 * atan(1 - iter/maxIter);

                for i = 1:popSize
                    E = 2 * log(1/rand) * theta;
                    r1 = rand;

                    if r1 < 0.25
                        % 跟随迁徙
                        newPos = pop(i,:) + E * (bestPos - pop(i,:));
                    elseif r1 < 0.5
                        % 随机游走
                        newPos = pop(i,:) + E * randn(1, dim) .* (ub - lb) * 0.1;
                    elseif r1 < 0.75
                        % 螺旋搜索
                        l = rand * 2 - 1;
                        newPos = bestPos + E * exp(l) * cos(2*pi*l) * (pop(i,:) - bestPos);
                    else
                        % Levy飞行
                        step = obj.levyFlight(dim) .* (ub - lb) * 0.05;
                        newPos = pop(i,:) + step;
                    end

                    % 边界约束
                    newPos = max(lb, min(ub, newPos));

                    % 评估
                    newFit = evalFcn(newPos);
                    totalEvals = totalEvals + 1;
                    if newFit < fitness(i)
                        pop(i,:) = newPos;
                        fitness(i) = newFit;
                        if newFit < bestFit
                            bestFit = newFit;
                            bestPos = newPos;
                        end
                    end
                end
            end

            % 重建最优路径
            path = obj.vectorToPath(bestPos, start, goal, nWaypoints);
            [cost, det] = obj.costModel.evaluatePath(path, t_start, hasPayload);

            info.time = toc;
            info.iterations = actualIter;
            info.totalEvaluations = totalEvals;
            info.iterationBudget = maxIter;
            info.timeBudget = obj.timeBudget;
            info.budgetExhausted = (actualIter >= maxIter) || (info.time >= obj.timeBudget);
            info.algorithm = 'ALA-Planner';
            info.success = (cost < inf) && det.feasible;
            info.reachedGoal = info.success;
        end

        function cost = evaluatePathFromVector(obj, x, start, goal, nWaypoints, t_start, hasPayload)
            path = obj.vectorToPath(x, start, goal, nWaypoints);
            [cost, det] = obj.costModel.evaluatePath(path, t_start, hasPayload);
            if ~det.feasible
                cost = cost + 1000;
            end
        end

        function path = vectorToPath(obj, x, start, goal, nWaypoints)
            path = zeros(nWaypoints + 2, 3);
            path(1,:) = start;
            path(end,:) = goal;
            for i = 1:nWaypoints
                path(i+1,:) = x((i-1)*3+1:i*3);
                % 高度安全检查
                ex = max(1, min(obj.env.MAP_SIZE, round(path(i+1,1))));
                ey = max(1, min(obj.env.MAP_SIZE, round(path(i+1,2))));
                path(i+1,3) = max(path(i+1,3), obj.env.heightMap(ex,ey) + 5);
            end
        end

        function step = levyFlight(obj, dim)
            beta = 1.5;
            sigma = (gamma(1+beta)*sin(pi*beta/2)/(gamma((1+beta)/2)*beta*2^((beta-1)/2)))^(1/beta);
            u = randn(1, dim) * sigma;
            v = randn(1, dim);
            step = u ./ abs(v).^(1/beta);
        end

        %% ==================== Greedy Heuristic ====================
        function [path, cost, info] = greedyPlanner(obj, start, goal, t_start, hasPayload)
            % 简单贪心路径规划
            if nargin < 5, hasPayload = true; end
            if nargin < 4, t_start = 0; end

            tic;
            path = obj.greedyPath(start, goal, t_start);
            [cost, pathDetails] = obj.costModel.evaluatePath(path, t_start, hasPayload);
            info.time = toc;
            info.algorithm = 'Greedy';
            info.success = (cost < inf);
            info.reachedGoal = true;
            info.details = pathDetails;
            info.iterations = 1;
            info.iterationBudget = 1;
            info.timeBudget = inf;
            info.budgetExhausted = false;
        end

        function path = greedyPath(obj, start, goal, t_start)
            % 贪心路径搜索
            % 确保start和goal是行向量
            start = start(:)';
            goal = goal(:)';
            if length(start) < 3, start = [start, 60]; end
            if length(goal) < 3, goal = [goal, 60]; end

            nodes = obj.env.nodes;
            edges = obj.env.edges;

            startXY = [start(1), start(2)];
            goalXY = [goal(1), goal(2)];
            [~, startIdx] = min(pdist2(startXY, nodes(:,1:2)));
            [~, goalIdx] = min(pdist2(goalXY, nodes(:,1:2)));

            current = startIdx;
            visited = false(size(nodes,1), 1);
            visited(current) = true;

            pathIdx = current;
            currentH = start(3);

            maxSteps = size(nodes,1) * 2;
            step = 0;

            while current ~= goalIdx && step < maxSteps
                step = step + 1;
                neighbors = edges(edges(:,1) == current, 2);
                neighbors = neighbors(~visited(neighbors));

                if isempty(neighbors)
                    if length(pathIdx) > 1
                        pathIdx(end) = [];
                        current = pathIdx(end);
                        currentH = double(nodes(current, 3)) + 10;
                    else
                        break;
                    end
                    continue;
                end

                % 选择最接近目标的邻居 (排除 NFZ/障碍内的节点)
                gx = double(nodes(goalIdx, 1));
                gy = double(nodes(goalIdx, 2));
                dists = inf(length(neighbors), 1);  % 默认 inf → 被排除
                for i = 1:length(neighbors)
                    neighIdx = neighbors(i);
                    nx = double(nodes(neighIdx, 1));
                    ny = double(nodes(neighIdx, 2));
                    nz = max(double(nodes(neighIdx, 3)) + 5, obj.minH);

                    % ★ 检查该节点是否在 NFZ/动态障碍内
                    safeNode = true;
                    if ~isempty(obj.env.dynObstacles)
                        % NFZ: 保守策略 — 忽略时间窗, 视所有 NFZ 为永久激活
                        for nfi = 1:length(obj.env.dynObstacles.tempNFZ)
                            nfz = obj.env.dynObstacles.tempNFZ(nfi);
                            if ~nfz.active, continue; end
                            dh = sqrt((nx-nfz.center(1))^2+(ny-nfz.center(2))^2);
                            if dh < nfz.radius*1.15 && nz >= nfz.height(1) && nz <= nfz.height(2)
                                safeNode = false; break;
                            end
                        end
                        % 动态障碍 (用 t_start 粗估)
                        if safeNode
                            if obj.env.dynObstacles.checkCollision(nx, ny, nz, t_start)
                                safeNode = false;
                            end
                        end
                    end

                    if safeNode
                        dists(i) = sqrt((nx - gx)^2 + (ny - gy)^2);
                    end
                end

                % 如果所有邻居都不安全, 回退到纯距离选择 (允许穿越, 避免死锁)
                if all(isinf(dists))
                    for i = 1:length(neighbors)
                        neighIdx = neighbors(i);
                        nx = double(nodes(neighIdx, 1));
                        ny = double(nodes(neighIdx, 2));
                        dists(i) = sqrt((nx - gx)^2 + (ny - gy)^2);
                    end
                end
                [~, minIdx] = min(dists);
                nextNode = neighbors(minIdx);

                visited(nextNode) = true;
                pathIdx = [pathIdx; nextNode];
                currentH = max(double(nodes(nextNode, 3)) + 5, obj.minH);
                current = nextNode;
            end

            % 构建路径
            path = zeros(length(pathIdx), 3);
            path(1,:) = start(1:3);
            for i = 2:length(pathIdx)
                idx = pathIdx(i);
                nx = double(nodes(idx, 1));
                ny = double(nodes(idx, 2));
                nz = double(nodes(idx, 3));
                z = max(nz + 5, obj.minH);
                path(i,:) = [nx, ny, z];
            end
            if norm(path(end,1:2) - goal(1:2)) > 1
                path = [path; goal(1:3)];
            else
                path(end,:) = goal(1:3);
            end
        end
    end
end
