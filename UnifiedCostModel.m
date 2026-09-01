classdef UnifiedCostModel < handle
% =========================================================================
% UnifiedCostModel - 城市低空无人机配送统一代价模型 (时变一致性 + 诊断分解版)
% =========================================================================
%
% 代价函数:
%   J = w_e*E + w_t*T/60 + w_c*C_climb + w_r*R_dynamic + lambda*Penalty
%
% ======================================================================
% 【相对原版的修复与新增 (诊断版)】
%
%  BUG 修复 — 端点高度惩罚重复计费:
%    原版在每段循环内同时检查 p1 和 p2 两个端点:
%      for pt_idx = 1:2  → p1, p2 都计一次 (min_allowed-z)/10
%    这导致中间航路点(索引 2..N-1)被相邻两段各检查一次, 重复收费.
%    以 20 点平滑路径为例, 18 个中间点各被收 2 次端点费,
%    lambda=100 时仅此一项就能使 J 虚增 ~1800.
%    修复方案: 每段只检查 p1; 最后路径点在段循环后单独检查一次.
%
%  新增字段 (目标1: 完整代价分解输出):
%    details.J_final                   最终总代价 (= J)
%    details.penalty_total             总惩罚 (= Penalty, 进入 J 计算)
%    details.penalty_height            高度约束违反 (H_min/H_max)
%    details.penalty_static_collision  静态建筑物碰撞
%    details.penalty_scene_entry       moving-obstacle or active-NFZ scene entry
%    details.penalty_dynamic_collision legacy alias of penalty_scene_entry
%    details.penalty_battery           电池超限
%    details.penalty_nfz               NFZ 穿越硬罚 (v12: 计入 penalty_total/J)
%    details.NFZ_penalty               [兼容旧名] = penalty_nfz (现已加入 J)
%    details.turn_or_smoothness_penalty 0 (仅在 evaluateRAALASearchFitness 搜索阶段存在)
%    details.wind_lookahead_penalty     0 (仅在 evaluateRAALASearchFitness 搜索阶段存在)
%    details.repair_penalty             0 (通过阶段对比诊断, 见 runRA_ALA.m)
%    details.feasible                   可行标志 (无任何硬约束违规)
%
%  接口兼容性:
%    函数签名、属性、其他方法与原版完全一致; 新增字段不影响已有调用.
% ======================================================================

    properties
        % ==================== 无人机物理参数 ====================
        m_frame = 3.5;
        m_payload = 1.0;
        v_cruise = 15.0;
        eta_motor = 0.85;
        eta_prop = 0.75;
        n_rotor = 4;
        r_prop = 0.15;
        rho_air = 1.225;
        C_d = 0.25;
        A_body = 0.04;
        E_batt = 200;

        % ==================== 高度限制参数 ====================
        H_min = 30;
        H_max = 120;
        H_clearance = 5;

        % ==================== 代价函数权重 ====================
        w_energy = 1.0;
        w_time = 0.5;
        w_climb = 2.0;
        w_risk = 10.0;
        w_height = 50.0;
        lambda_penalty = 100;

        % ==================== 环境模型引用 ====================
        windField;
        dynObstacles;
        staticMap;

        % ==================== 爬升代价参数 ====================
        climb_efficiency = 0.7;
        descent_recovery = 0.3;

        % ==================== 风险评估参数 ====================
        risk_horizon = 60;
        collision_radius = 15;

        % ==================== Collision-sampling resolution ====================
        % Shared by building, moving-obstacle, and active-NFZ checks.
        collision_sample_spacing = 1.5; % m; planning/search-stage resolution
        min_collision_samples = 3;
        wait_sample_interval = 1.0; % s; sampling for explicit loiter edges
        ground_speed_floor = 0.5;   % m/s; signed-progress acceptance threshold and diagnostic floor
        time_iteration_tolerance = 1e-6; % s
        time_iteration_max = 8;
        time_root_max_evaluations = 64;
        time_max_subdivision_depth = 6;
        time_max_evaluations_per_subsegment = 512;
    end

    methods
        function obj = UnifiedCostModel(uavParams, weights)
            if nargin >= 1 && ~isempty(uavParams)
                fn = fieldnames(uavParams);
                for i = 1:length(fn)
                    if isprop(obj, fn{i})
                        obj.(fn{i}) = uavParams.(fn{i});
                    end
                end
            end
            if nargin >= 2 && ~isempty(weights)
                if isfield(weights,'w_energy'), obj.w_energy = weights.w_energy; end
                if isfield(weights,'w_time'),   obj.w_time   = weights.w_time;   end
                if isfield(weights,'w_climb'),  obj.w_climb  = weights.w_climb;  end
                if isfield(weights,'w_risk'),   obj.w_risk   = weights.w_risk;   end
            end
        end

        function setEnvironment(obj, windField, dynObs, staticMap)
            obj.windField = windField;
            obj.dynObstacles = dynObs;
            obj.staticMap = staticMap;
        end

        function setCollisionSampling(obj, spacing_m, min_samples)
            validateattributes(spacing_m,{'numeric'}, ...
                {'scalar','positive','finite'});
            if nargin < 3 || isempty(min_samples)
                min_samples = obj.min_collision_samples;
            end
            validateattributes(min_samples,{'numeric'}, ...
                {'scalar','integer','positive','finite'});
            obj.collision_sample_spacing = double(spacing_m);
            obj.min_collision_samples = double(min_samples);
        end

        %% ================================================================
        %%  evaluatePath — 时变一致性 + 惩罚分解版
        %%  主要修改: 修复端点重复计费; 新增 penalty_* 分解字段
        %% ================================================================
        function [J, details] = evaluatePath(obj, pathPts, t_start, hasPayload, waitBeforeSegmentS)
            % >>>>> RUNTIME_ANALYSIS PATCH 1 (eval_count) >>>>>
            global EVAL_COUNTER;
            if ~isempty(EVAL_COUNTER), EVAL_COUNTER = EVAL_COUNTER + 1; end
            % <<<<< RUNTIME_ANALYSIS PATCH 1 END <<<<<
        % 评估单条路径的统一代价 (完整分解输出)
        %
        % 输入:
        %   pathPts    [N x 3]   路径点 [x, y, z]
        %   t_start    (s)       起飞时刻
        %   hasPayload (logical) 是否携带载荷
        %
        % 输出:
        %   J       标量总代价
        %   details 完整代价分解结构体 (见文件头注释)

            if nargin < 4, hasPayload = true; end
            if nargin < 3, t_start = 0; end

            N = size(pathPts, 1);
            if N < 2
                J = inf;
                details = obj.emptyDetails(t_start);
                return;
            end
            if nargin < 5 || isempty(waitBeforeSegmentS)
                waitBeforeSegmentS = zeros(N-1,1);
            end
            waitBeforeSegmentS = double(waitBeforeSegmentS(:));
            if numel(waitBeforeSegmentS) ~= N-1 || ...
                    any(~isfinite(waitBeforeSegmentS)) || any(waitBeforeSegmentS < 0)
                error('UnifiedCostModel:InvalidWaitSchedule', ...
                    'waitBeforeSegmentS must contain N-1 finite nonnegative values.');
            end

            % ---- 质量相关常量 ----
            if hasPayload
                m_total = obj.m_frame + obj.m_payload;
            else
                m_total = obj.m_frame;
            end
            W         = m_total * 9.81;
            eta       = obj.eta_motor * obj.eta_prop;
            A_disc    = obj.n_rotor * pi * obj.r_prop^2;
            v_i_hover = sqrt(W / (2 * obj.rho_air * A_disc));
            P_hover   = W^1.5 / sqrt(2*obj.rho_air*A_disc) / eta;

            % ---- 段级累加器 ----
            E_segments  = zeros(N-1, 1);
            T_segments  = zeros(N-1, 1);
            C_climb_seg = zeros(N-1, 1);
            R_risk_seg  = zeros(N-1, 1);
            Penalty_seg = zeros(N-1, 1);

            % ---- 分解惩罚段累加器 ----
            Ph_seg = zeros(N-1, 1);   % 高度
            Ps_seg = zeros(N-1, 1);   % 静态碰撞
            Pd_seg = zeros(N-1, 1);
            Pk_seg = zeros(N-1, 1);   % track-holding violation
            Pn_wait_seg = zeros(N-1,1); % active-NFZ violation during waits   % 动态碰撞

            % ---- 到达时刻 ----
            t_arrivals = zeros(N, 1);
            t_arrivals(1) = t_start;
            t_current = t_start;

            SUB_SPACING = obj.collision_sample_spacing;
            MIN_SUB     = obj.min_collision_samples;
            total_subsamples = 0;
            max_time_iterations_used = 0;
            timeOptions = struct('tolerance',obj.time_iteration_tolerance, ...
                'max_iterations',obj.time_iteration_max, ...
                'max_root_evaluations',obj.time_root_max_evaluations, ...
                'max_depth',obj.time_max_subdivision_depth, ...
                'max_evaluations',obj.time_max_evaluations_per_subsegment);
            timeStats = struct('fixed_point_failures',0,'root_attempts',0, ...
                'root_successes',0,'subdivisions',0,'wind_evaluations',0, ...
                'max_depth_used',0,'max_accepted_residual_s',0);
            pen_nfz_motion = 0;
            total_nfz_subsamples = 0;

            % ==============================================================
            %  逐段循环
            % ==============================================================
            for k = 1:N-1
                p1 = pathPts(k, :);
                p2 = pathPts(k+1, :);

                dx = p2(1)-p1(1); dy = p2(2)-p1(2); dz = p2(3)-p1(3);
                d_horiz = hypot(dx,dy);
                d_3d = norm(p2-p1);

                E_seg_acc = 0; T_seg_acc = 0; R_seg_acc = 0;
                P_seg_acc = 0; Ph_acc = 0; Ps_acc = 0; Pd_acc = 0;
                Pk_acc = 0; Pn_wait_acc = 0;

                % An explicit wait is part of the time-parameterized path.
                wait_s = waitBeforeSegmentS(k);
                if wait_s > 0
                    nWait = max(2,ceil(wait_s/obj.wait_sample_interval));
                    dt_wait = wait_s/nWait;
                    for ws = 1:nWait
                        t_wait = t_current+(ws-0.5)*dt_wait;
                        E_seg_acc = E_seg_acc+P_hover*dt_wait/3600;
                        T_seg_acc = T_seg_acc+dt_wait;
                        if ~isempty(obj.dynObstacles)
                            R_seg_acc = R_seg_acc+ ...
                                obj.evaluateRiskAtPoint(p1,t_wait)*dt_wait/60;
                            if obj.checkCollision(p1,t_wait)
                                pen = 1/nWait;
                                P_seg_acc = P_seg_acc+pen;
                                Pd_acc = Pd_acc+pen;
                            end
                            try
                                for ni = 1:length(obj.dynObstacles.tempNFZ)
                                    nfz = obj.dynObstacles.tempNFZ(ni);
                                    if ~nfz.active || t_wait < nfz.t_start || t_wait > nfz.t_end
                                        continue;
                                    end
                                    dh = norm(p1(1:2)-nfz.center);
                                    if dh < nfz.radius && p1(3) >= nfz.height(1) && p1(3) <= nfz.height(2)
                                        Pn_wait_acc = Pn_wait_acc+(1-dh/nfz.radius)/nWait;
                                    end
                                end
                            catch
                            end
                        end
                    end
                    t_current = t_current+wait_s;
                end

                if d_3d < 0.01
                    E_segments(k)=E_seg_acc; T_segments(k)=T_seg_acc;
                    R_risk_seg(k)=R_seg_acc; Penalty_seg(k)=P_seg_acc;
                    Pd_seg(k)=Pd_acc; Pn_wait_seg(k)=Pn_wait_acc;
                    t_arrivals(k+1)=t_current;
                    continue;
                end

                if d_horiz > 0.01, dir_h=[dx,dy]/d_horiz; else, dir_h=[0,0]; end
                nSub = max(MIN_SUB,ceil(d_3d/SUB_SPACING));
                t_sub = t_current;

                % ---- 子采样循环 ----
                for s = 1:nSub
                    a_sub = p1+(s-1)/nSub*(p2-p1);
                    b_sub = p1+s/nSub*(p2-p1);
                    [timeSamples,stepInfo] = solveArrivalTimeStep(a_sub,b_sub,t_sub, ...
                        obj.windField,obj.v_cruise,obj.ground_speed_floor,timeOptions);
                    max_time_iterations_used = max(max_time_iterations_used, ...
                        stepInfo.max_fixed_point_iterations);
                    for field = {'fixed_point_failures','root_attempts','root_successes', ...
                            'subdivisions','wind_evaluations'}
                        name = field{1};
                        timeStats.(name) = timeStats.(name)+stepInfo.(name);
                    end
                    timeStats.max_depth_used = max(timeStats.max_depth_used,stepInfo.max_depth_used);
                    timeStats.max_accepted_residual_s = max(timeStats.max_accepted_residual_s, ...
                        stepInfo.max_accepted_residual_s);
                    if ~stepInfo.converged
                        J = Inf;
                        details = obj.failedTimeDetails(t_start,N,k,s,stepInfo, ...
                            t_arrivals,waitBeforeSegmentS);
                        details.time_solver = timeStats;
                        details.max_time_iterations_used = max_time_iterations_used;
                        details.total_collision_subsamples = total_subsamples;
                        return;
                    end
                    total_subsamples = total_subsamples+numel(timeSamples);
                    for leaf = 1:numel(timeSamples)
                    sample = timeSamples(leaf);
                    pt_sub = sample.point;
                    t_sub = sample.start_time;
                    dt_sub = sample.duration;
                    kin = sample.kinematics;
                    sampleWeight = sample.length/d_3d;
                    if ~kin.feasible
                        P_seg_acc = P_seg_acc+sampleWeight;
                        Pk_acc = Pk_acc+sampleWeight;
                    end
                    air_velocity = kin.airVelocity;
                    v_air_horiz = norm(air_velocity(1:2));
                    v_air_vert = air_velocity(3);
                    v_air = norm(air_velocity);
                    v_ground = kin.groundSpeed;
                    if d_horiz > 0.01
                        v_wind_cross = abs(dot(air_velocity(1:2),[-dir_h(2),dir_h(1)]));
                    else
                        v_wind_cross = norm(air_velocity(1:2));
                    end

                    % (d) 功率 (动量理论)
                    mu = max(v_air_horiz, 0) / (v_i_hover + 0.01);
                    if mu < 0.1
                        v_i = v_i_hover * (1 - mu^2/4);
                    else
                        v_i = v_i_hover^2 / (2 * max(abs(v_air_horiz), 1));
                    end
                    P_induced  = W * v_i / eta;
                    P_parasite = 0.5*obj.rho_air*obj.C_d*obj.A_body*v_air^3/eta;
                    P_profile  = 0.15 * P_hover;
                    if v_air_vert > 0
                        P_climb_raw = W * v_air_vert / obj.climb_efficiency / eta;
                    else
                        P_climb_raw = W * v_air_vert * obj.descent_recovery;
                    end
                    if v_wind_cross > 0.5
                        tilt    = atan2(v_wind_cross, v_i_hover * 3);
                        P_cross = P_hover * (1/cos(tilt) - 1);
                    else
                        P_cross = 0;
                    end
                    P_total = max(P_induced + P_parasite + P_profile + ...
                                  P_climb_raw + P_cross, P_hover * 0.3);

                    % (e) 子段时间与能耗
                    E_sub  = P_total * dt_sub / 3600;

                    % (f) 动态风险 + 碰撞检测
                    if ~isempty(obj.dynObstacles)
                        t_risk   = t_sub + dt_sub / 2;
                        risk_sub = obj.evaluateRiskAtPoint(pt_sub, t_risk);
                        R_seg_acc = R_seg_acc + risk_sub * dt_sub / 60;
                        if obj.checkCollision(pt_sub, t_risk)
                            pen = sampleWeight;
                            P_seg_acc = P_seg_acc + pen;
                            Pd_acc    = Pd_acc    + pen;
                        end
                    end

                    % (g) 静态碰撞 (建筑物高度+3m 缓冲)
                    if ~isempty(obj.staticMap)
                        rx = max(1, min(size(obj.staticMap,1), round(pt_sub(1))));
                        ry = max(1, min(size(obj.staticMap,2), round(pt_sub(2))));
                        if pt_sub(3) < obj.staticMap(rx, ry) + 3
                            pen = sampleWeight;
                            P_seg_acc = P_seg_acc + pen;
                            Ps_acc    = Ps_acc    + pen;
                        end
                    end

                    % (h) 高度约束 (H_min / H_max)
                    if ~isempty(obj.staticMap)
                        rx = max(1, min(size(obj.staticMap,1), round(pt_sub(1))));
                        ry = max(1, min(size(obj.staticMap,2), round(pt_sub(2))));
                        ground_h = obj.staticMap(rx, ry);
                    else
                        ground_h = 0;
                    end
                    min_allowed = max(obj.H_min, ground_h + obj.H_clearance);
                    if pt_sub(3) < min_allowed
                        pen = (min_allowed - pt_sub(3)) / 10 * sampleWeight;
                        P_seg_acc = P_seg_acc + pen;
                        Ph_acc    = Ph_acc    + pen;
                    end
                    if pt_sub(3) > obj.H_max
                        pen = (pt_sub(3) - obj.H_max) / 10 * sampleWeight;
                        P_seg_acc = P_seg_acc + pen;
                        Ph_acc    = Ph_acc    + pen;
                    end

                    % NFZ activation uses the accepted local timeline, not
                    % waypoint interpolation that would smear explicit waits.
                    if ~isempty(obj.dynObstacles) && isfield(obj.dynObstacles,'tempNFZ')
                        total_nfz_subsamples = total_nfz_subsamples+1;
                        for ni = 1:numel(obj.dynObstacles.tempNFZ)
                            nfz = obj.dynObstacles.tempNFZ(ni);
                            if ~nfz.active || sample.mid_time < nfz.t_start || sample.mid_time > nfz.t_end
                                continue;
                            end
                            dh = norm(pt_sub(1:2)-nfz.center);
                            if dh < nfz.radius && pt_sub(3) >= nfz.height(1) && pt_sub(3) <= nfz.height(2)
                                pen_nfz_motion = pen_nfz_motion+(1-dh/nfz.radius)*sampleWeight;
                            end
                        end
                    end
                    E_seg_acc = E_seg_acc + E_sub;
                    T_seg_acc = T_seg_acc + dt_sub;
                    t_sub     = t_sub + dt_sub;
                    end % accepted time-solver leaves
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
                Pk_seg(k)      = Pk_acc;
                Pn_wait_seg(k) = Pn_wait_acc;

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

            % NFZ checks share each accepted subsegment's space-time sample.
            pen_nfz = sum(Pn_wait_seg)+pen_nfz_motion;
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
            details.numerically_valid = true;
            details.time_converged = true;
            details.evaluation_status = 'evaluated';
            details.time_failure = [];
            details.time_solver = timeStats;
            details.time_root_max_evaluations = obj.time_root_max_evaluations;
            details.time_max_subdivision_depth = obj.time_max_subdivision_depth;
            details.time_max_evaluations_per_subsegment = obj.time_max_evaluations_per_subsegment;
            details.J_final                   = J;
            details.E_total                   = E_total;
            details.T_total                   = T_total;
            details.C_climb                   = C_climb;
            details.R_dynamic                 = R_dynamic;
            details.penalty_total             = Penalty;
            details.penalty_height            = pen_height;
            details.penalty_static_collision  = pen_static;
            details.penalty_scene_entry       = pen_dyn;
            details.penalty_dynamic_collision = pen_dyn; % legacy field retained for archive compatibility
            details.penalty_kinematic         = sum(Pk_seg);
            details.penalty_battery           = pen_battery;
            details.penalty_nfz               = pen_nfz;   % v12: NFZ 硬约束, 已计入 penalty_total/J
            % 兼容旧字段名: 现等于进入 J 的 NFZ 硬罚 (不再是"仅诊断")
            details.NFZ_penalty               = pen_nfz;
            details.turn_or_smoothness_penalty= 0;   % 仅 evaluateRAALASearchFitness 中
            details.wind_lookahead_penalty    = 0;   % 仅 evaluateRAALASearchFitness 中
            details.repair_penalty            = 0;   % 通过阶段对比诊断
            % 可行性与统计
            details.feasible = ~(pen_height>0 || pen_static>0 || pen_dyn>0 || ...
                pen_battery>0 || pen_nfz>0 || sum(Pk_seg)>0);
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
            details.wait_before_segment_s     = waitBeforeSegmentS;
            details.total_wait_time_s          = sum(waitBeforeSegmentS);
            details.time_iteration_tolerance_s = obj.time_iteration_tolerance;
            details.time_iteration_max        = obj.time_iteration_max;
            details.max_time_iterations_used  = max_time_iterations_used;
            details.velocity_convention       = 'prescribed-airspeed 3-D track holding';
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
        function d = failedTimeDetails(obj,tStart,nPoints,segment,sample,info,arrivals,waits)
            d = obj.emptyDetails(tStart);
            d.evaluation_status = 'time_solver_failure';
            d.time_failure = struct('segment',segment,'nominal_sample',sample, ...
                'point',info.failure_point,'reason',info.reason, ...
                'last_residual_s',info.last_residual_s);
            % Inf is a rejection sentinel, not a measured physical penalty.
            % Individual physical constraints are unknown after propagation stops.
            for field = {'penalty_height','penalty_static_collision', ...
                    'penalty_scene_entry','penalty_dynamic_collision','penalty_kinematic', ...
                    'penalty_battery','penalty_nfz','NFZ_penalty'}
                d.(field{1}) = NaN;
            end
            d.E_segments = NaN(nPoints-1,1);
            d.T_segments = NaN(nPoints-1,1);
            d.dyn_col_segs = NaN(nPoints-1,1);
            d.t_arrivals = NaN(nPoints,1);
            d.t_arrivals(1:segment) = arrivals(1:segment);
            d.t_end = NaN;
            d.SoC_end = NaN;
            d.wait_before_segment_s = waits;
            d.total_wait_time_s = sum(waits);
        end

        function d = emptyDetails(obj, t_start) %#ok<INUSL>
            d.numerically_valid = false;
            d.time_converged = false;
            d.evaluation_status = 'empty_path';
            d.time_failure = [];
            d.time_solver = struct('fixed_point_failures',0,'root_attempts',0, ...
                'root_successes',0,'subdivisions',0,'wind_evaluations',0, ...
                'max_depth_used',0,'max_accepted_residual_s',0);
            d.time_root_max_evaluations = obj.time_root_max_evaluations;
            d.time_max_subdivision_depth = obj.time_max_subdivision_depth;
            d.time_max_evaluations_per_subsegment = obj.time_max_evaluations_per_subsegment;
            d.J_final                   = inf;
            d.E_total                   = inf;
            d.T_total                   = inf;
            d.C_climb                   = inf;
            d.R_dynamic                 = inf;
            d.penalty_total             = inf;
            d.penalty_height            = inf;
            d.penalty_static_collision  = inf;
            d.penalty_scene_entry       = inf;
            d.penalty_dynamic_collision = inf;
            d.penalty_kinematic         = inf;
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
            d.wait_before_segment_s     = [];
            d.total_wait_time_s          = 0;
            d.time_iteration_tolerance_s = obj.time_iteration_tolerance;
            d.time_iteration_max        = obj.time_iteration_max;
            d.max_time_iterations_used  = 0;
            d.velocity_convention       = 'prescribed-airspeed 3-D track holding';
        end
    end
end
