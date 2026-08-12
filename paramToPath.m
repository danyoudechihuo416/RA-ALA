function path = paramToPath(x, start, goal, nWP, dirUnit, perpUnit, totalDist, env, minH) %#ok<INUSL>
    path = zeros(nWP+2, 3);
    path(1,:) = start; path(end,:) = goal;
    for j = 1:nWP
        frac   = j/(nWP+1);
        latOff = x((j-1)*2+1);
        alt    = x((j-1)*2+2);
        baseXY = start(1:2) + frac*(goal(1:2)-start(1:2));
        ptXY   = baseXY + latOff*perpUnit;
        ptXY(1) = max(1, min(env.MAP_SIZE, ptXY(1)));
        ptXY(2) = max(1, min(env.MAP_SIZE, ptXY(2)));
        ex  = max(1, min(env.MAP_SIZE, round(ptXY(1))));
        ey  = max(1, min(env.MAP_SIZE, round(ptXY(2))));
        alt = max(alt, env.heightMap(ex,ey)+5);
        alt = max(alt, minH);
        path(j+1,:) = [ptXY, alt];
    end
end

%% ============== 搜索阶段内部适应度 (不输出为最终 J) ==============
