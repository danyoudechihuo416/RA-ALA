classdef CityEnvironment < handle
% CityEnvironment - 可配置复杂度的城市环境生成器
%   支持低/中/高三级城市复杂度
%   支持弱/中/强/变风四级风场强度
%   支持少/中/多三级动态风险密度

    properties
        MAP_SIZE = 1000;
        GRID_STEP = 10;

        % 地形数据
        heightMap;
        buildings;

        % 环境模型
        windField;
        dynObstacles;

        % 配置参数
        cityComplexity;     % 'low', 'medium', 'high'
        windLevel;          % 'weak', 'medium', 'strong', 'variable'
        riskDensity;        % 'sparse', 'medium', 'dense'

        % 任务点
        depot;
        deliveryPoints;

        % 规划图
        nodes;
        edges;
        D_matrix;
    end

    methods
        function obj = CityEnvironment(mapSize, gridStep)
            if nargin >= 1, obj.MAP_SIZE = mapSize; end
            if nargin >= 2, obj.GRID_STEP = gridStep; end
        end

        function generate(obj, cityComplexity, windLevel, riskDensity, seed)
            % 生成完整城市环境
            if nargin >= 5 && ~isempty(seed)
                rng(seed);
            end

            obj.cityComplexity = cityComplexity;
            obj.windLevel = windLevel;
            obj.riskDensity = riskDensity;

            % 1. 生成地形和建筑
            obj.generateTerrain();
            obj.generateBuildings(cityComplexity);
            obj.buildHeightMap();

            % 2. 生成风场
            obj.generateWindField(windLevel);

            % 3. 生成动态障碍
            obj.generateDynamicObstacles(riskDensity);

            % 4. 构建规划图
            obj.buildPlanningGraph();
        end

        function generateTerrain(obj)
            % 生成基础地形 (轻微起伏)
            t = 1:obj.MAP_SIZE;
            [xg, yg] = meshgrid(t);
            h_ground = 1 + 0.3*sin(xg/80) + 0.3*cos(yg/80) + 0.1*randn(obj.MAP_SIZE);
            h_ground = imgaussfilt(h_ground, 5);
            obj.heightMap = max(h_ground, 0.5);
        end

        function generateBuildings(obj, complexity)
            % 根据复杂度生成建筑物
            switch complexity
                case 'low'
                    % 低复杂度: 15栋建筑, 高度30-60m
                    nBuildings = 15;
                    hRange = [30, 60];
                    sizeRange = [20, 40];
                case 'medium'
                    % 中复杂度: 35栋建筑, 高度40-120m
                    nBuildings = 35;
                    hRange = [40, 120];
                    sizeRange = [15, 35];
                case 'high'
                    % 高复杂度: 60栋建筑, 高度50-180m
                    nBuildings = 60;
                    hRange = [50, 180];
                    sizeRange = [10, 30];
                otherwise
                    nBuildings = 35;
                    hRange = [40, 120];
                    sizeRange = [15, 35];
            end

            obj.buildings = zeros(nBuildings, 5);
            margin = 50;

            for i = 1:nBuildings
                placed = false;
                attempts = 0;
                while ~placed && attempts < 100
                    cx = margin + rand * (obj.MAP_SIZE - 2*margin);
                    cy = margin + rand * (obj.MAP_SIZE - 2*margin);
                    hw = sizeRange(1) + rand * (sizeRange(2) - sizeRange(1));
                    hh = sizeRange(1) + rand * (sizeRange(2) - sizeRange(1));
                    bh = hRange(1) + rand * (hRange(2) - hRange(1));

                    % 检查重叠
                    overlap = false;
                    for j = 1:i-1
                        if abs(cx - obj.buildings(j,1)) < (hw + obj.buildings(j,4)) * 1.5 && ...
                           abs(cy - obj.buildings(j,2)) < (hh + obj.buildings(j,5)) * 1.5
                            overlap = true;
                            break;
                        end
                    end

                    if ~overlap
                        obj.buildings(i, :) = [cx, cy, bh, hw, hh];
                        placed = true;
                    end
                    attempts = attempts + 1;
                end

                if ~placed
                    % 强制放置
                    obj.buildings(i, :) = [cx, cy, bh, hw, hh];
                end
            end
        end

        function buildHeightMap(obj)
            % 将建筑物叠加到高程图
            h_buildings = zeros(obj.MAP_SIZE);
            for i = 1:size(obj.buildings, 1)
                cx = obj.buildings(i,1); cy = obj.buildings(i,2);
                bh = obj.buildings(i,3);
                hw = obj.buildings(i,4); hh = obj.buildings(i,5);
                rx1 = max(1, round(cx-hw)); rx2 = min(obj.MAP_SIZE, round(cx+hw));
                ry1 = max(1, round(cy-hh)); ry2 = min(obj.MAP_SIZE, round(cy+hh));
                h_buildings(rx1:rx2, ry1:ry2) = max(h_buildings(rx1:rx2, ry1:ry2), bh);
            end
            obj.heightMap = max(obj.heightMap, h_buildings);
        end

        function generateWindField(obj, level)
            % 生成时变风场 (所有等级都启用时变特性, 用于验证时变感知)
            switch level
                case 'weak'
                    params.v_base = 3.0;
                    params.turbulence = 0.15;
                    params.gust_prob = 0.03;
                case 'medium'
                    params.v_base = 6.0;
                    params.turbulence = 0.25;
                    params.gust_prob = 0.08;
                case 'strong'
                    params.v_base = 11.0;
                    params.turbulence = 0.40;
                    params.gust_prob = 0.12;
                case 'variable'
                    params.v_base = 8.0;
                    params.turbulence = 0.45;
                    params.gust_prob = 0.15;
                otherwise
                    params.v_base = 6.0;
                    params.turbulence = 0.25;
                    params.gust_prob = 0.08;
            end

            params.dir_base = rand * 2 * pi;
            params.gust_factor = 2.8;
            params.canyon_accel = 1.8;
            % 核心: 所有风场都启用时变, 确保不同出发时刻环境不同
            params.time_varying = true;

            obj.windField = obj.createWindFieldStruct(params);
        end

        function wf = createWindFieldStruct(obj, params)
            % 创建风场结构体
            gs = 10;
            gN = ceil(obj.MAP_SIZE / gs);

            wx_base = params.v_base * cos(params.dir_base);
            wy_base = params.v_base * sin(params.dir_base);

            windMapX = ones(gN) * wx_base;
            windMapY = ones(gN) * wy_base;

            % 建筑物影响
            for b = 1:size(obj.buildings, 1)
                cx = obj.buildings(b,1); cy = obj.buildings(b,2);
                bh = obj.buildings(b,3);
                hw = obj.buildings(b,4); hh = obj.buildings(b,5);

                for gi = 1:gN
                    for gj = 1:gN
                        gx = gi * gs; gy = gj * gs;
                        dx = gx - cx; dy = gy - cy;
                        along = dx * cos(params.dir_base) + dy * sin(params.dir_base);
                        cross = -dx * sin(params.dir_base) + dy * cos(params.dir_base);
                        hw_eff = abs(hw*cos(params.dir_base)) + abs(hh*sin(params.dir_base));
                        hh_eff = abs(hw*sin(params.dir_base)) + abs(hh*cos(params.dir_base));

                        if along > 0 && along < 5*bh && abs(cross) < hh_eff*2
                            wake_decay = exp(-along / (2*bh));
                            cross_decay = exp(-(cross/max(hh_eff,1))^2);
                            wake_factor = 1 - 0.6 * wake_decay * cross_decay;
                            windMapX(gi,gj) = windMapX(gi,gj) * wake_factor;
                            windMapY(gi,gj) = windMapY(gi,gj) * wake_factor;
                        end

                        if abs(along) < hw_eff*1.5 && abs(cross) > hh_eff && abs(cross) < hh_eff*3
                            canyon_factor = 1 + (params.canyon_accel - 1) * ...
                                exp(-(abs(cross)-hh_eff)^2 / hh_eff^2);
                            windMapX(gi,gj) = windMapX(gi,gj) * canyon_factor;
                            windMapY(gi,gj) = windMapY(gi,gj) * canyon_factor;
                        end
                    end
                end
            end

            wf.windMap.X = windMapX;
            wf.windMap.Y = windMapY;
            wf.windMap.gridStep = gs;
            wf.windMap.gridN = gN;
            wf.params = params;
            wf.getWind = @(x,y,z,t) obj.getWindAt(x,y,z,t,wf);
        end

        function w = getWindAt(obj, x, y, z, t, wf)
            gs = wf.windMap.gridStep;
            gN = wf.windMap.gridN;
            p = wf.params;

            gi = max(1, min(gN, round(x / gs)));
            gj = max(1, min(gN, round(y / gs)));

            wx0 = wf.windMap.X(gi, gj);
            wy0 = wf.windMap.Y(gi, gj);

            % 高度剖面 (对数风速律)
            z0 = 1.0; z_ref = 80;
            z_eff = max(z, z0 + 0.1);
            height_factor = log(z_eff / z0) / log(z_ref / z0);
            height_factor = max(0.3, min(2.0, height_factor));

            wx = wx0 * height_factor;
            wy = wy0 * height_factor;
            wz = 0;

            % 湍流 (空间+时间相关)
            turb_intensity = p.turbulence * (1 + 2 * exp(-z / 50));
            seed_val = x*137.3 + y*271.7 + z*419.3 + t*53.7;
            wx = wx + turb_intensity * p.v_base * sin(seed_val * 0.013 + 1.7);
            wy = wy + turb_intensity * p.v_base * cos(seed_val * 0.017 + 2.3);
            wz = turb_intensity * p.v_base * 0.3 * sin(seed_val * 0.011 + 3.1);

            % ===== 强时变特性 (核心修改: 让不同出发时刻体验到截然不同的风) =====
            if isfield(p, 'time_varying') && p.time_varying
                % (1) 风向随时间旋转: 周期约180秒转一圈
                dir_shift = 0.8 * sin(2*pi*t / 180);         % 方向偏移角 (rad)
                cos_s = cos(dir_shift); sin_s = sin(dir_shift);
                wx_new = wx * cos_s - wy * sin_s;            % 旋转矩阵
                wy_new = wx * sin_s + wy * cos_s;
                wx = wx_new;
                wy = wy_new;

                % (2) 风速幅值随时间波动: ±40%, 周期约120秒
                speed_factor = 1 + 0.4 * sin(2*pi*t / 120 + x/500);
                wx = wx * speed_factor;
                wy = wy * speed_factor;

                % (3) 垂直阵风 (低空上升气流, 高空下沉)
                wz = wz + p.v_base * 0.15 * sin(2*pi*t/90 + y/300);
            end

            % 阵风 (突发性, 依赖时间窗)
            gust_hash = mod(floor(x/50)*31 + floor(y/50)*37 + floor(t/10)*41, 100);
            if gust_hash < p.gust_prob * 100
                gust_dir = mod(gust_hash * 0.0628 + t*0.01, 2*pi); % 阵风方向也随时间变
                gust_speed = p.v_base * p.gust_factor;
                wx = wx + gust_speed * cos(gust_dir);
                wy = wy + gust_speed * sin(gust_dir);
            end

            w = [wx, wy, wz];
        end

        function generateDynamicObstacles(obj, density)
            % 根据密度生成动态障碍物
            switch density
                case 'sparse'
                    nMoving = 3;
                    nNFZ = 1;
                case 'medium'
                    nMoving = 6;
                    nNFZ = 3;
                case 'dense'
                    nMoving = 12;
                    nNFZ = 5;
                otherwise
                    nMoving = 6;
                    nNFZ = 3;
            end

            obj.dynObstacles = obj.createDynObsStruct(nMoving, nNFZ);
        end

        function dyn = createDynObsStruct(obj, nMoving, nNFZ)
            % 创建动态障碍物结构
            obsTemplate = struct('type','','pos0',[0 0 0],'velocity',[0 0 0],...
                'radius',0,'speed',0,'z_range',[0 0],'pattern','',...
                'center',[0 0],'orbit_r',0,'period',0,'phase',0);

            movObs = repmat(obsTemplate, 1, nMoving);

            for i = 1:nMoving
                pattern_type = mod(i-1, 3);
                if pattern_type == 0
                    % 圆形巡逻
                    movObs(i).type = 'patrol';
                    movObs(i).pos0 = [100+rand*800, 100+rand*800, 60+rand*60];
                    movObs(i).radius = 12 + rand*8;
                    movObs(i).speed = 6 + rand*6;
                    movObs(i).pattern = 'circular';
                    movObs(i).center = movObs(i).pos0(1:2);
                    movObs(i).orbit_r = 60 + rand*80;
                    movObs(i).period = 2*pi*movObs(i).orbit_r / movObs(i).speed;
                    movObs(i).phase = rand*2*pi;
                    movObs(i).z_range = [50, 130];
                elseif pattern_type == 1
                    % 直线穿越
                    movObs(i).type = 'linear';
                    movObs(i).pos0 = [50+rand*900, 50+rand*900, 70+rand*40];
                    movObs(i).velocity = (rand(1,3)-0.5) .* [12,12,3];
                    movObs(i).radius = 10 + rand*5;
                    movObs(i).speed = norm(movObs(i).velocity);
                    movObs(i).pattern = 'linear_bounce';
                    movObs(i).z_range = [40, 150];
                else
                    % 八字形
                    movObs(i).type = 'flock';
                    movObs(i).pos0 = [300+rand*400, 300+rand*400, 80+rand*30];
                    movObs(i).radius = 20 + rand*15;
                    movObs(i).pattern = 'figure8';
                    movObs(i).center = movObs(i).pos0(1:2);
                    movObs(i).orbit_r = 100 + rand*60;
                    movObs(i).period = 120 + rand*120;
                    movObs(i).phase = rand*2*pi;
                    movObs(i).z_range = [60, 140];
                end
            end

            % 禁飞区
            tempNFZ = struct('center',{},'radius',{},'height',{},...
                't_start',{},'t_end',{},'active',{});
            for i = 1:nNFZ
                tempNFZ(i).center = [100+rand*800, 100+rand*800];
                tempNFZ(i).radius = 50 + rand*50;
                tempNFZ(i).height = [0, 80 + rand*70];
                tempNFZ(i).t_start = rand * 200;
                tempNFZ(i).t_end = tempNFZ(i).t_start + 200 + rand*400;
                tempNFZ(i).active = true;
            end

            dyn.movingObs = movObs;
            dyn.tempNFZ = tempNFZ;
            dyn.MAP_SIZE = obj.MAP_SIZE;
            dyn.getPosition = @(idx, t) obj.getObsPos(movObs(idx), t);
            dyn.getAllPositions = @(t) obj.getAllObsPos(movObs, t);
            dyn.checkCollision = @(x,y,z,t) obj.checkDynCollision(x,y,z,t,movObs,tempNFZ);
        end

        function pos = getObsPos(obj, obs, t)
            switch obs.pattern
                case 'circular'
                    angle = obs.phase + 2*pi*t/obs.period;
                    pos = [obs.center(1) + obs.orbit_r*cos(angle), ...
                           obs.center(2) + obs.orbit_r*sin(angle), ...
                           obs.pos0(3) + 5*sin(t/20)];
                case 'figure8'
                    angle = obs.phase + 2*pi*t/obs.period;
                    pos = [obs.center(1) + obs.orbit_r*sin(angle), ...
                           obs.center(2) + obs.orbit_r*sin(2*angle)/2, ...
                           obs.pos0(3) + 8*sin(t/25)];
                case 'linear_bounce'
                    raw = obs.pos0 + obs.velocity * t;
                    pos = raw;
                    for d = 1:2
                        pos(d) = mod(pos(d), 2*obj.MAP_SIZE);
                        if pos(d) > obj.MAP_SIZE
                            pos(d) = 2*obj.MAP_SIZE - pos(d);
                        end
                    end
                    pos(3) = max(obs.z_range(1), min(obs.z_range(2), raw(3)));
                otherwise
                    pos = obs.pos0;
            end
            pos = max([1,1,obs.z_range(1)], min([obj.MAP_SIZE,obj.MAP_SIZE,obs.z_range(2)], pos));
        end

        function allPos = getAllObsPos(obj, movObs, t)
            n = length(movObs);
            allPos = zeros(n, 4);
            for i = 1:n
                p = obj.getObsPos(movObs(i), t);
                allPos(i,:) = [p, movObs(i).radius];
            end
        end

        function col = checkDynCollision(obj, x, y, z, t, movObs, tempNFZ)
            col = false;
            for i = 1:length(movObs)
                pos = obj.getObsPos(movObs(i), t);
                if norm([x,y,z] - pos) < movObs(i).radius
                    col = true; return;
                end
            end
            for i = 1:length(tempNFZ)
                nfz = tempNFZ(i);
                if ~nfz.active || t < nfz.t_start || t > nfz.t_end, continue; end
                dh = sqrt((x-nfz.center(1))^2 + (y-nfz.center(2))^2);
                if dh < nfz.radius && z >= nfz.height(1) && z <= nfz.height(2)
                    col = true; return;
                end
            end
        end

        function buildPlanningGraph(obj)
            % 构建下采样规划图
            planGrid = obj.GRID_STEP:obj.GRID_STEP:obj.MAP_SIZE;
            [pg1, pg2] = meshgrid(planGrid, planGrid);
            p1 = pg1(:); p2 = pg2(:);

            z_nodes = zeros(length(p1), 1);
            for i = 1:length(p1)
                xi = p1(i); yi = p2(i);
                rx1 = max(1, round(xi-obj.GRID_STEP/2));
                rx2 = min(obj.MAP_SIZE, round(xi+obj.GRID_STEP/2));
                ry1 = max(1, round(yi-obj.GRID_STEP/2));
                ry2 = min(obj.MAP_SIZE, round(yi+obj.GRID_STEP/2));
                z_nodes(i) = max(max(obj.heightMap(rx1:rx2, ry1:ry2)));
            end

            obj.nodes = [p1, p2, z_nodes];

            D = pdist2(obj.nodes(:,1:2), obj.nodes(:,1:2));
            maxND = obj.GRID_STEP * 2 * sqrt(2) + 0.1;
            [np1, np2] = find(D <= maxND & D > 0);
            obj.edges = [np1, np2];
            obj.D_matrix = D;
        end

        function setTaskPoints(obj, depot, deliveryPoints)
            % 设置任务点
            obj.depot = depot;
            if depot(3) < obj.heightMap(max(1,min(obj.MAP_SIZE,round(depot(1)))), ...
                                         max(1,min(obj.MAP_SIZE,round(depot(2))))) + 5
                obj.depot(3) = obj.heightMap(max(1,min(obj.MAP_SIZE,round(depot(1)))), ...
                                              max(1,min(obj.MAP_SIZE,round(depot(2))))) + 10;
            end

            obj.deliveryPoints = deliveryPoints;
            for i = 1:size(deliveryPoints, 1)
                ex = max(1, min(obj.MAP_SIZE, round(deliveryPoints(i,1))));
                ey = max(1, min(obj.MAP_SIZE, round(deliveryPoints(i,2))));
                if deliveryPoints(i,3) < obj.heightMap(ex, ey) + 5
                    obj.deliveryPoints(i,3) = obj.heightMap(ex, ey) + 10;
                end
            end
        end

        function pts = generateRandomDeliveryPoints(obj, nPoints, seed)
            % 随机生成配送点
            if nargin >= 3
                rng(seed);
            end

            margin = 80;
            pts = zeros(nPoints, 3);
            for i = 1:nPoints
                placed = false;
                while ~placed
                    x = margin + rand * (obj.MAP_SIZE - 2*margin);
                    y = margin + rand * (obj.MAP_SIZE - 2*margin);
                    z = 60 + rand * 40;

                    % 避开建筑物中心
                    tooClose = false;
                    for b = 1:size(obj.buildings, 1)
                        if norm([x,y] - obj.buildings(b,1:2)) < 50
                            tooClose = true;
                            break;
                        end
                    end

                    if ~tooClose
                        pts(i,:) = [x, y, z];
                        placed = true;
                    end
                end
            end

            % 确保高度安全
            for i = 1:nPoints
                ex = max(1, min(obj.MAP_SIZE, round(pts(i,1))));
                ey = max(1, min(obj.MAP_SIZE, round(pts(i,2))));
                pts(i,3) = max(pts(i,3), obj.heightMap(ex, ey) + 10);
            end
        end
    end
end
