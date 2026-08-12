function t_arr = computeApproxTArr(path, t_start)
% computeApproxTArr — 基于 cumDist/12m/s 的回退粗估
% 仅在 evaluatePath 抛出异常时作为安全兜底使用.
    nPts = size(path, 1);
    cumDist = zeros(nPts, 1);
    for k = 2:nPts
        cumDist(k) = cumDist(k-1) + norm(path(k,:) - path(k-1,:));
    end
    t_arr = t_start + cumDist / 12;
end

%% ====================================================================
%%  本地辅助函数（fig7/fig8 专用，无需 Statistics Toolbox）
%% ====================================================================

