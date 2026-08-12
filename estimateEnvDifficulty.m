function [diffScale, info] = estimateEnvDifficulty(env, startPt, goalPt)
% estimateEnvDifficulty - v9 新增: 估计环境硬度并输出导向罚项的缩放因子
%
% 输入:
%   env     - CityEnvironment 对象 (含 buildings, dynObstacles)
%   startPt - 起点 [x y z]
%   goalPt  - 终点 [x y z]
%
% 输出:
%   diffScale - 标量, 在 [0.3, 1.5] 区间. 用于乘到导向罚项 (NFZ/obs/headwind) 上.
%               1.0 = 中等环境 (默认基准)
%               <1.0 = hard env, 导向罚项缩小, 让 final_J 更主导搜索
%               >1.0 = 简单 env, 导向罚项放大, 加强引导
%   info    - 结构体, 含 coverage / lineHits / hardness 等诊断信息
%
% 设计动机 (v9):
%   在 Scheme 2.5 hard env 中, 静态系数的 NFZ/obs/headwind 导向罚项激活率过高,
%   把 ALA 搜索推向过度绕远的次优路径. 通过对环境硬度自适应缩放罚项, 让 hard env
%   里搜索目标更接近 final_J 本身, 避免"搜索-评估"口径错配.
%
% 经验调参 (v9):
%   - hardness=0 (理想空旷): diffScale=1.5
%   - hardness=0.5 (中等城市): diffScale=0.9
%   - hardness=1 (极端 NFZ 密集): diffScale=0.3

    % ---- (1) NFZ 覆盖率 (占地图面积比例) ----
    mapSide = env.MAP_SIZE;
    if numel(mapSide) > 1, mapSide = mapSide(1); end
    mapArea = mapSide^2;

    nfzArea = 0;
    nfzCount = 0;
    if ~isempty(env.dynObstacles) && isfield(env.dynObstacles, 'tempNFZ')
        nfzList = env.dynObstacles.tempNFZ;
        for ni = 1:length(nfzList)
            nfz = nfzList(ni);
            if ~nfz.active, continue; end
            nfzArea  = nfzArea + pi * nfz.radius^2;
            nfzCount = nfzCount + 1;
        end
    end
    nfzCoverage = nfzArea / mapArea;        % 0~1

    % ---- (2) 建筑覆盖率 ----
    bldgArea = 0;
    nB = size(env.buildings, 1);
    for b = 1:nB
        hw = env.buildings(b, 4);
        hh = env.buildings(b, 5);
        bldgArea = bldgArea + (2*hw) * (2*hh);
    end
    bldgCoverage = bldgArea / mapArea;      % 0~1

    % ---- (3) 起终点直线穿越障碍统计 ----
    % 在 start→goal 的直线上离散采样, 数有多少个采样点落在 NFZ 或建筑内
    nSample = 30;
    line_hits_nfz  = 0;
    line_hits_bldg = 0;
    for s = 1:nSample
        frac = s / (nSample + 1);
        pt = startPt + frac * (goalPt - startPt);

        % NFZ 命中
        if ~isempty(env.dynObstacles) && isfield(env.dynObstacles, 'tempNFZ')
            for ni = 1:length(env.dynObstacles.tempNFZ)
                nfz = env.dynObstacles.tempNFZ(ni);
                if ~nfz.active, continue; end
                dh = norm(pt(1:2) - nfz.center);
                if dh < nfz.radius
                    line_hits_nfz = line_hits_nfz + 1;
                    break;
                end
            end
        end

        % 建筑命中 (XY 投影内 且 z < 建筑高度)
        for b = 1:nB
            cx = env.buildings(b,1); cy = env.buildings(b,2);
            bh = env.buildings(b,3);
            hw = env.buildings(b,4); hh = env.buildings(b,5);
            if abs(pt(1)-cx) < hw && abs(pt(2)-cy) < hh && pt(3) < bh
                line_hits_bldg = line_hits_bldg + 1;
                break;
            end
        end
    end
    lineHitsRatio = (line_hits_nfz + line_hits_bldg) / nSample;   % 0~1

    % ---- (4) 综合硬度评分 (v10: 对 NFZ 更敏感, 整体更陡) ----
    % 权重: NFZ覆盖率 0.50 (×5), 建筑覆盖率 0.15 (×3), 直线穿越 0.35 (×2)
    %   - NFZ 5-10% 覆盖率即可触发 hardness=0.5-1.0
    %   - 起终点直线被高密度阻挡时, 直线穿越项贡献接近上限
    hardness = 0.50 * min(1, nfzCoverage * 5.0) + ...    % NFZ 5% → 25%, 10%+ → 满
               0.15 * min(1, bldgCoverage * 3.0) + ...   % 建筑次要
               0.35 * min(1, lineHitsRatio * 2.0);       % 直线穿越快速饱和
    hardness = max(0, min(1, hardness));

    % ---- (5) v10: 映射到 diffScale ∈ [0.2, 1.5], 斜率从 -1.2 加陡到 -1.5 ----
    % hardness=0 → 1.5 (空旷, 加强引导)
    % hardness=0.5 → 0.75 (中等, 略缩)
    % hardness=1 → 0 → 截断到 0.2 (极硬, 大幅削弱导向)
    diffScale = 1.5 - 1.5 * hardness;
    diffScale = max(0.20, diffScale);   % 下界 0.2, 防止完全失活

    info = struct();
    info.nfzCoverage   = nfzCoverage;
    info.bldgCoverage  = bldgCoverage;
    info.lineHitsRatio = lineHitsRatio;
    info.hardness      = hardness;
    info.nfzCount      = nfzCount;
    info.diffScale     = diffScale;
end
