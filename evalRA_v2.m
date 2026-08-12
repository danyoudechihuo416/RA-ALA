function J = evalRA_v2(x, param2path, t_start, hasPayload, costModel, env, cfg)
% evalRA_v2 — 搜索阶段内部适应度函数
%
% 设计原则:
%   所有导向惩罚项均保留 (不删除), 但通过以下三层控制与 final_unified_J 对齐:
%
%   (1) 系数大幅缩减: NFZ 500→3, obs 300→2, smooth coeff 8→0.3
%       使每个约束违反的单次惩罚量纲与 final_J (~15~25) 相近
%
%   (2) 各项设置独立软饱和上界 (soft cap):
%       smooth ≤ 15, NFZ ≤ 25, obs ≤ 20, headwind ≤ 8
%       防止单一大违反将整个搜索景观淹没为平坦区
%
%   (3) 不可行惩罚从 5000 降至 150:
%       保持不可行路径劣于可行路径的排序关系,
%       但不会使 internal_J 比 final_J 高出 2~3 个数量级
%
%   这样 internal_J ≈ final_J + guidance_bonus, 其中 guidance_bonus ≤ ~80,
%   而不是原来的 guidance_bonus ~ 5000+.
%
% 导向项仍然有效: 比较两候选时, 哪个更平滑/远离 NFZ/避免逆风
% 的候选仍会获得更低的 internal_J, 优化器仍能正确排序.

    path = param2path(x);
    % v3 设计变更（2026-06）：Repair 已从搜索阶段移除。
    % 原因：在搜索内部调用 repair 会让 ALA 优化一个"假设 repair 完美"的目标，
    % 由于 repair 自身含随机性且非全能，搜索找到的"最优"解在 Top-K 阶段
    % 重做 repair 时可能不可行，导致 Full 反而比 w/o Repair 表现更差。
    % Repair 模块现仅保留为 Top-K 后处理的可选候选（且只在 ablate_repair=false 时启用）。
    nPts = size(path, 1);
    [J, det] = costModel.evaluatePath(path, t_start, hasPayload);

    % ---- 风险权重增强 (量级已在 final_J 中, 仅放大差异) ----
    J = J + (cfg.riskWeight - 10) * det.R_dynamic;

    % ====================================================================
    % (1) smoothPenalty — 平滑度导向
    %     原系数 8 → 0.3; 总量上界 15
    %     目的: 让更平滑的路径得到更低 J, 但不让一条弯折路径的惩罚
    %           超过 final_J 本身
    % ====================================================================
    if ~(isfield(cfg,'ablate_smoothPenalty') && cfg.ablate_smoothPenalty)
        sp = 0;
        SMOOTH_COEFF = 0.3;   % 原 8 → 0.3 (缩减 ~27×)
        SMOOTH_CAP   = 15.0;  % 总量软上界
        for k = 2:nPts-1
            v1=path(k,:)-path(k-1,:); v2=path(k+1,:)-path(k,:);
            n1=norm(v1); n2=norm(v2);
            if n1>1 && n2>1
                ca = max(-1, min(1, dot(v1,v2)/(n1*n2)));
                sp = sp + acos(ca)^2 * SMOOTH_COEFF;
            end
        end
        J = J + min(sp, SMOOTH_CAP);
    end

    % ====================================================================
    % (2) 不可行惩罚  (150 = 保持排序, 不过度膨胀)
    % ====================================================================
    if ~det.feasible
        J = J + 150;
    end

    % ====================================================================
    % (2b) 静态高度违规 proxy  ★ 目标2: 感知 final penalty_height / penalty_static
    %
    % 问题: UnifiedCostModel.evaluatePath 的 penalty_static 和 penalty_height
    %       对段内子采样违规才累加, 但 evalRA_v2 的路径点只有 ~10 个,
    %       点级查询可能漏掉段内高度违规, 导致 internal 低而 final 高.
    % 修复: 对 raw path 的每条段做 4 个子采样, 统计接近建筑或低于 H_min 的程度,
    %       加入与 final lambda*Penalty 同量级的 soft proxy.
    %       上界 = 50 (约等于 0.5 个单位 final penalty × lambda=100).
    % ====================================================================
    PROX_NSUB  = 4;
    PROX_CAP   = 50.0;
    PROX_HCOEF = 2.0;   % per-meter height violation proxy coefficient
    heightProxy = 0;
    if ~isempty(env.heightMap)
        MS_p = env.MAP_SIZE;
        cl_p = costModel.H_clearance;
        for k = 1:nPts-1
            p1p = path(k,:); p2p = path(k+1,:);
            for s = 1:PROX_NSUB
                frac = s/(PROX_NSUB+1);
                pt_p = p1p + frac*(p2p-p1p);
                rx=max(1,min(MS_p,round(pt_p(1)))); ry=max(1,min(MS_p,round(pt_p(2))));
                gH_p = env.heightMap(rx,ry);
                minFloor_p = max(costModel.H_min, gH_p + cl_p);
                if pt_p(3) < minFloor_p
                    heightProxy = heightProxy + PROX_HCOEF*(minFloor_p-pt_p(3))/PROX_NSUB;
                end
                if pt_p(3) > costModel.H_max
                    heightProxy = heightProxy + PROX_HCOEF*(pt_p(3)-costModel.H_max)/PROX_NSUB;
                end
            end
        end
    end
    J = J + min(heightProxy, PROX_CAP);

    % ====================================================================
    % (3) NFZ + 动态障碍  —— v12: 已移除搜索期 Explicit 惩罚
    %     NFZ:   现由 UnifiedCostModel.evaluatePath 作为硬约束计入
    %            penalty_total/J (见 pen_nfz), 搜索与最终同口径, 无错配。
    %     动态障碍: 实际碰撞已在 evaluatePath 的 pen_dyn (硬) 中, 邻近度在
    %            R_dynamic (软) 中; 原 obsP 为双重冗余, 一并移除。
    %     注: cfg.ablate_explicitPenalty 在 v12 已成为 no-op (此块不存在),
    %         "w/o Explicit" 消融变体与 Full 等价, 建议从消融中剔除。
    % ====================================================================

    % ====================================================================
    % (4) 风场前瞻惩罚 (量级本已较小 ~0~8, 保持不变)
    %     软上界 8 防止极端风场情形失控
    % ====================================================================
    if cfg.windLookahead > 0 && nPts > 2
        % v9: 复用同一硬度缩放因子
        if isfield(cfg,'difficultyScale')
            diffS_w = cfg.difficultyScale;
        else
            diffS_w = 1.0;
        end
        if isfield(det,'t_arrivals') && numel(det.t_arrivals)==nPts
            tA_wind = det.t_arrivals;
        else
            tA_wind = t_start + det.T_total*(0:nPts-1)'/max(nPts-1,1);
        end
        T_end = tA_wind(end);
        hp = 0;
        HEADWIND_CAP = 8.0;
        for la = 1:cfg.windLookahead
            ft = t_start + (T_end - t_start) * la / (cfg.windLookahead + 1);
            [~, mi] = min(abs(tA_wind - ft));
            mi = max(2, min(nPts-1, mi));
            pt = path(mi,:);
            w  = env.windField.getWind(pt(1), pt(2), pt(3), ft);
            d3 = path(mi+1,:) - path(mi-1,:);
            d3 = d3 / max(norm(d3), 0.01);
            hw = -dot(w, d3);
            if hw > 2, hp = hp + hw*0.5; end
        end
        J = J + diffS_w * min(hp, HEADWIND_CAP);
    end
end

%% ============== 三次样条路径平滑 (与原版完全一致) ==============
