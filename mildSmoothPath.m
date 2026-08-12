function smoothed = mildSmoothPath(rawPath)
% mildSmoothPath - v9 新增: 路径的"温和"平滑
%
% 对 rawPath 的内部点做一次 [1/4, 1/2, 1/4] 三点平均, 起终点保持不变.
% 目的: 在 Top-K 阶段提供介于 raw 和 smoothPathSpline 之间的第三候选,
%       避免 hard env 中 "raw 太弯折 但 spline 截弯过头穿 NFZ" 的两难.
%
% 输入:
%   rawPath - N×3 路径
% 输出:
%   smoothed - N×3 路径 (长度不变, 拓扑不变, 只去掉最尖锐的折线)

    nPts = size(rawPath, 1);
    if nPts < 3
        smoothed = rawPath;
        return;
    end

    smoothed = rawPath;   % 起终点保留
    for k = 2:nPts-1
        smoothed(k, :) = 0.25*rawPath(k-1, :) + 0.5*rawPath(k, :) + 0.25*rawPath(k+1, :);
    end
end
