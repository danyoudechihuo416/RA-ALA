function printCostDecomposition(det, label)
% printCostDecomposition — 打印 evaluatePath 返回的完整代价分解
%
% 用法:
%   printCostDecomposition(details, 'RA-ALA t=0s')
%   printCostDecomposition(details, 'Energy-A*')
%
% 输出说明:
%   第一部分: J = w_e*E + w_t*T/60 + w_c*C + w_r*R + lambda*Pen  逐项展示
%   第二部分: Penalty 按类型分解 (定位哪类违规导致高 Penalty)
%   第三部分: 诊断字段 (不加入 J, 用于溯源)
%   第四部分: 可行性摘要

    if nargin < 2, label = ''; end
    sep = repmat('─', 1, 64);
    fprintf('  %s\n', sep);
    fprintf('  ★ 代价分解: [%s]\n', label);
    fprintf('  %s\n', sep);

    J_from_E   = 1.0   * det.E_total;
    J_from_T   = 0.5   * det.T_total / 60;
    J_from_C   = 2.0   * det.C_climb;
    J_from_R   = 10.0  * det.R_dynamic;
    J_from_Pen = 100.0 * det.penalty_total;
    J_check    = J_from_E + J_from_T + J_from_C + J_from_R + J_from_Pen;

    fprintf('  J_final      = %9.3f\n', det.J_final);
    fprintf('  ├─ w_e×E     = %9.3f    (E        = %.3f Wh)\n', J_from_E,  det.E_total);
    fprintf('  ├─ w_t×T/60  = %9.3f    (T        = %.1f s)\n',  J_from_T,  det.T_total);
    fprintf('  ├─ w_c×C     = %9.3f    (C_climb  = %.5f)\n',    J_from_C,  det.C_climb);
    fprintf('  ├─ w_r×R     = %9.3f    (R_dynamic= %.5f)\n',    J_from_R,  det.R_dynamic);
    fprintf('  └─ λ×Pen     = %9.3f    (Pen_total= %.5f)\n',    J_from_Pen,det.penalty_total);
    if abs(J_check - det.J_final) > 0.5
        fprintf('  [!] 口径校验: sum=%.3f vs J_final=%.3f  差=%.3f\n', ...
            J_check, det.J_final, J_check - det.J_final);
    end

    fprintf('  %s\n', repmat('·', 1, 64));
    fprintf('  Penalty 分解 (penalty_total=%.5f, λ=100 → J贡献=%.3f):\n', ...
        det.penalty_total, J_from_Pen);
    fprintf('  ├─ penalty_height            = %.5f  → %.3f\n', ...
        det.penalty_height,           100*det.penalty_height);
    fprintf('  ├─ penalty_static_collision  = %.5f  → %.3f\n', ...
        det.penalty_static_collision, 100*det.penalty_static_collision);
    fprintf('  ├─ penalty_dynamic_collision = %.5f  → %.3f\n', ...
        det.penalty_dynamic_collision,100*det.penalty_dynamic_collision);
    fprintf('  └─ penalty_battery           = %.5f  → %.3f\n', ...
        det.penalty_battery,          100*det.penalty_battery);

    fprintf('  %s\n', repmat('·', 1, 64));
    fprintf('          NFZ穿越硬罚(已入J)   = %.2f\n',  det.NFZ_penalty);
    fprintf('  [诊断-不入J] smooth/turn惩罚  = %.2f   (仅 evalRA_v2 搜索阶段)\n', ...
        det.turn_or_smoothness_penalty);
    fprintf('  [诊断-不入J] wind_lookahead   = %.2f   (仅 evalRA_v2 搜索阶段)\n', ...
        det.wind_lookahead_penalty);

    % ★ 若存在内部搜索代价, 显示与 final_J 的差距
    if isfield(det, 'internal_search_cost') && ~isnan(det.internal_search_cost)
        delta_guide = det.internal_search_cost - det.J_final;
        fprintf('  %s\n', repmat('·', 1, 64));
        fprintf('  ┌─── 口径透明度 ─────────────────────────────────────────────┐\n');
        fprintf('  │  final_unified_J    = %9.3f  ← 论文报告 / 算法比较用\n', det.J_final);
        fprintf('  │  internal_search_J  = %9.3f  ← ALA 优化器内部目标\n', det.internal_search_cost);
        fprintf('  │  差值 (导向罚项总量)= %+9.3f\n', delta_guide);
        fprintf('  │    = smoothPenalty + nfzPenalty + obsPenalty + headwindPenalty\n');
        if delta_guide > 100
            fprintf('  │  [!] 差值较大: 搜索阶段对路径质量的导向效果显著\n');
        end
        fprintf('  └────────────────────────────────────────────────────────────┘\n');
    end

    fprintf('  %s\n', repmat('·', 1, 64));
    if det.feasible
        feasStr = '✓ 可行 (无硬约束违规)';
    else
        feasStr = sprintf('✗ 不可行 (Pen=%.4f)', det.penalty_total);
    end
    fprintf('  feasible: %s  |  heightViol=%d  |  SoC_end=%.1f%%\n', ...
        feasStr, det.heightViolations, det.SoC_end*100);
    fprintf('  %s\n\n', sep);
end

%% ============== 辅助: 粗估到达时刻 (仅当 evaluatePath 失败时使用) ==============
