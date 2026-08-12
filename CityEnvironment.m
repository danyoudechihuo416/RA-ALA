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
                    movO�6��$z{-���jםk = 0;
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
