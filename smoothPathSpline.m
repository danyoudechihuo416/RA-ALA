function smoothed = smoothPathSpline(rawPath, env, minH, maxH)
% smoothPathSpline — 三次样条平滑
% 修改: 平滑完毕后增加一轮静态建筑段级验证 + 局部插点,
%   解决 pchip 弧在两安全端点之间向下穿过建筑顶部的问题.
    nPts=size(rawPath,1);
    if nPts<4, smoothed=rawPath; return; end
    cumDist=zeros(nPts,1);
    for k=2:nPts, cumDist(k)=cumDist(k-1)+norm(rawPath(k,:)-rawPath(k-1,:)); end
    totalLen=cumDist(end);
    if totalLen<1, smoothed=rawPath; return; end
    tParam=cumDist/totalLen;
    nResample=max(20,round(totalLen/15));
    tNew=linspace(0,1,nResample)';
    try
        xNew=interp1(tParam,rawPath(:,1),tNew,'pchip');
        yNew=interp1(tParam,rawPath(:,2),tNew,'pchip');
        zNew=interp1(tParam,rawPath(:,3),tNew,'pchip');
    catch
        xNew=interp1(tParam,rawPath(:,1),tNew,'linear');
        yNew=interp1(tParam,rawPath(:,2),tNew,'linear');
        zNew=interp1(tParam,rawPath(:,3),tNew,'linear');
    end
    smoothed=[xNew,yNew,zNew];
    smoothed(1,:)=rawPath(1,:); smoothed(end,:)=rawPath(end,:);
    MS=env.MAP_SIZE;
    cl=5;  % 默认净空, 与 UnifiedCostModel.H_clearance 一致
    % 点级高度钳制
    for k=1:size(smoothed,1)
        smoothed(k,1)=max(1,min(MS,smoothed(k,1)));
        smoothed(k,2)=max(1,min(MS,smoothed(k,2)));
        ex=max(1,min(MS,round(smoothed(k,1)))); ey=max(1,min(MS,round(smoothed(k,2))));
        smoothed(k,3)=max(smoothed(k,3),env.heightMap(ex,ey)+cl);
        smoothed(k,3)=max(smoothed(k,3),minH); smoothed(k,3)=min(smoothed(k,3),maxH);
    end
    % ★ 段级静态碰撞检查: 对每段做子采样, 若穿建筑则抬高两端点 z
    % 不插点 (插点会导致指数级路径增长并卡死), 只就地修改端点高度
    SNSUB = 6;
    for k = 1:size(smoothed,1)-1
        p1=smoothed(k,:); p2=smoothed(k+1,:);
        worst_z = -1;  % 该段最严重违规所需的安全高度
        for s=1:SNSUB
            frac=s/(SNSUB+1);
            pt_s=p1+frac*(p2-p1);
            ex=max(1,min(MS,round(pt_s(1)))); ey=max(1,min(MS,round(pt_s(2))));
            gH=env.heightMap(ex,ey);
            minFloor=max(minH, gH+cl);
            need_z = max(minFloor, gH+3);
            if pt_s(3) < need_z
                worst_z = max(worst_z, need_z + 2);
            end
        end
        if worst_z < 0, continue; end
        % 就地抬高段两端点 (不含首末点)
        for fix_end = [k, k+1]
            if fix_end == 1 || fix_end == size(smoothed,1), continue; end
            smoothed(fix_end,3) = min(maxH, max(smoothed(fix_end,3), worst_z));
        end
    end
end

