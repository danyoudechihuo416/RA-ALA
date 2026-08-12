function repaired = repairRawPath(path, env, costModel, t_start, minH, maxH)
% repairRawPath — 优化循环内轻量修复 (支持 3D 动态障碍规避)
%
% 修复策略优先级:
%   (A) 静态建筑: 抬高 z
%   (B) NFZ: 高度逃逸 → 水平外推
%   (C) 动态障碍: 3D 方向外推 (先横向, 再高度辅助)
%   所有修复后做高度钳制.
%
% 注意: 此函数在 ALA 优化循环内被调用 1800+ 次,
%   必须使用轻量的 cumDist/12 粗估时间, 不能调用 evaluatePath.
%   精确 t_arrivals 在 postSmoothRepair (循环外) 中使用.
    MS=env.MAP_SIZE; nPts=size(path,1); repaired=path;
    cl=5; if isprop(costModel,'H_clearance'), cl=costModel.H_clearance; end
    % 轻量时间估算 (cumDist/12 m/s): 在优化循环内不调用 evaluatePath
    t_arr = computeApproxTArr(repaired, t_start);
    % 动态障碍安全距离: 与 evaluatePath 中 checkCollision 的 obs.radius 对齐,
    % 但修复时用更大裕量 3.0r 防止段内插值仍然碰撞
    OBS_SAFE_MULT = 3.0;
    for k=2:nPts-1
        pt=repaired(k,:); t_k=t_arr(k);
        % (A) 静态建筑
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(pt(1)))); ey=max(1,min(MS,round(pt(2))));
            gH=env.heightMap(ex,ey); mA=max(minH,gH+cl);
            if pt(3)<mA
                if mA+3<=maxH, repaired(k,3)=mA+3;
                else
                    sR=30; bH=gH; bXY=pt(1:2);
                    for ai=1:8
                        ang=(ai-1)*pi/4; cXY=pt(1:2)+sR*[cos(ang),sin(ang)];
                        cXY(1)=max(1,min(MS,cXY(1))); cXY(2)=max(1,min(MS,cXY(2)));
                        cx=max(1,min(MS,round(cXY(1)))); cy=max(1,min(MS,round(cXY(2))));
                        cH=env.heightMap(cx,cy); if cH<bH, bH=cH; bXY=cXY; end
                    end
                    repaired(k,1:2)=bXY; repaired(k,3)=max(minH,bH+cl+3);
                end
            end
        end
        % (B) NFZ
        if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'tempNFZ')
            for ni=1:length(env.dynObstacles.tempNFZ)
                nfz=env.dynObstacles.tempNFZ(ni);
                if ~nfz.active||t_k<nfz.t_start||t_k>nfz.t_end,continue;end
                pn=repaired(k,:); dh=norm(pn(1:2)-nfz.center);
                if dh<nfz.radius*1.1&&pn(3)>=nfz.height(1)&&pn(3)<=nfz.height(2)
                    if nfz.height(2)+10<=maxH, repaired(k,3)=nfz.height(2)+10;
                    elseif nfz.height(1)-10>=minH, repaired(k,3)=nfz.height(1)-10;
                    else
                        do=pn(1:2)-nfz.center; dn=norm(do);
                        if dn<0.1,do=[1,0];dn=1;end
                        repaired(k,1:2)=nfz.center+(do/dn)*nfz.radius*1.25;
                    end
                end
            end
        end
        % (C) 动态障碍 — 3D 方向外推, 裕量 OBS_SAFE_MULT
        if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'movingObs')
            for oi=1:length(env.dynObstacles.movingObs)
                obs=env.dynObstacles.movingObs(oi);
                op=env.dynObstacles.getPosition(oi,t_k);
                sR=obs.radius*OBS_SAFE_MULT;
                pn=repaired(k,:); d=norm(pn-op);
                if d<sR
                    if d>0.01
                        % 3D 方向外推: 沿 pn→op 反方向推到 sR+3m
                        ev=(pn-op)/d;
                        newPt=op+ev*(sR+3);
                        % 如果抬高会超过 maxH, 只用横向分量
                        if newPt(3)>maxH
                            ev2=[ev(1),ev(2),0]; n2=norm(ev2);
                            if n2>0.01, ev2=ev2/n2; newPt=op+ev2*(sR+3); end
                            newPt(3)=max(minH,min(maxH,pn(3)));
                        end
                        repaired(k,:)=newPt;
                    else
                        % 完全重合: 横向随机偏移 + 高度辅助
                        ang=rand*2*pi;
                        repaired(k,1)=op(1)+sR*cos(ang)+3;
                        repaired(k,2)=op(2)+sR*sin(ang)+3;
                        repaired(k,3)=min(maxH,op(3)+obs.radius*2);
                    end
                end
            end
        end
        % 最终钳制
        repaired(k,1)=max(1,min(MS,repaired(k,1)));
        repaired(k,2)=max(1,min(MS,repaired(k,2)));
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
            gH=env.heightMap(ex,ey);
            repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
        else, repaired(k,3)=max(repaired(k,3),minH); end
        repaired(k,3)=min(repaired(k,3),maxH);
    end
    repaired(1,:)=path(1,:); repaired(end,:)=path(end,:);
end

%% ============== 平滑后复核 + 局部修复 ==============
