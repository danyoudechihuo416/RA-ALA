function repaired = postSmoothRepair(smoothed, env, costModel, t_start, minH, maxH)
% postSmoothRepair — 平滑后多轮段级修复
%
% 修复流程:
%   Phase-1 (MAX_PASS 轮点级扫描): 静态/NFZ/动态障碍逐点修复
%   Phase-2 (轻量平滑):            3点加权平滑, 平滑前先保存已修复点的安全标志
%   Phase-3 (平滑后强制高度合规)
%   Phase-4 (SEG_PASSES 轮段级 validate):
%       每段 SEG_NSUB 个内部子采样点, 真实 t 插值.
%       若段内任何子点碰撞动态障碍:
%         策略A: 对两端点做 3D evasion (横向+z)
%         策略B (若A不足): 在碰撞子点位置插入新航路点并做 evasion
%   Phase-5 (插点后高度合规扫描 + 末尾强制平滑)

    MAX_PASS  = 3;
    SEG_NSUB  = 8;    % 段内子采样数 (更密, 不含端点)
    SEG_PASSES= 5;    % 段级修复轮数 (更多)
    OBS_CLEAR = 3.2;  % 动态障碍安全倍数 (与 evaluatePath checkCollision 1× 对齐后留裕量)

    MS=env.MAP_SIZE; repaired=smoothed;
    cl=5; if isprop(costModel,'H_clearance'),cl=costModel.H_clearance;end

    % =======================================================================
    % Phase-1: 点级扫描修复 (静态/NFZ/动态障碍)
    % =======================================================================
    for pass=1:MAX_PASS
        anyFix=false;
        nPts=size(repaired,1);
        % 取精确 t_arrivals
        try
            [~, pass_det] = costModel.evaluatePath(repaired, t_start, true);
            t_est_arr = pass_det.t_arrivals;
            if numel(t_est_arr) ~= nPts
                t_est_arr = computeApproxTArr(repaired, t_start);
            end
        catch
            t_est_arr = computeApproxTArr(repaired, t_start);
        end
        for k=2:nPts-1
            pt=repaired(k,:); t_k=t_est_arr(k);
            % (A) 静态建筑
            if ~isempty(env.heightMap)
                ex=max(1,min(MS,round(pt(1)))); ey=max(1,min(MS,round(pt(2))));
                gH=env.heightMap(ex,ey); mA=max(minH,gH+cl);
                if pt(3)<mA
                    if mA+2<=maxH, repaired(k,3)=mA+2; anyFix=true;
                    else
                        sR=25; bH=gH; bXY=pt(1:2);
                        for ai=0:7
                            ang=ai*pi/4; cXY=pt(1:2)+sR*[cos(ang),sin(ang)];
                            cXY(1)=max(1,min(MS,cXY(1))); cXY(2)=max(1,min(MS,cXY(2)));
                            cx=max(1,min(MS,round(cXY(1)))); cy=max(1,min(MS,round(cXY(2))));
                            cH=env.heightMap(cx,cy); if cH<bH,bH=cH;bXY=cXY;end
                        end
                        repaired(k,1:2)=bXY; repaired(k,3)=max(minH,bH+cl+2);
                        repaired(k,3)=min(repaired(k,3),maxH); anyFix=true;
                    end
                end
            end
            % (B) NFZ
            if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'tempNFZ')
                for ni=1:length(env.dynObstacles.tempNFZ)
                    nfz=env.dynObstacles.tempNFZ(ni);
                    if ~nfz.active||t_k<nfz.t_start||t_k>nfz.t_end,continue;end
                    dh=norm(pt(1:2)-nfz.center);
                    if dh<nfz.radius*1.1&&pt(3)>=nfz.height(1)&&pt(3)<=nfz.height(2)
                        if nfz.height(2)+10<=maxH, repaired(k,3)=nfz.height(2)+10; anyFix=true;
                        else
                            do=pt(1:2)-nfz.center; dn=norm(do);
                            if dn<0.1,do=[1,0];dn=1;end
                            repaired(k,1:2)=nfz.center+(do/dn)*nfz.radius*1.2;
                            repaired(k,1)=max(1,min(MS,repaired(k,1)));
                            repaired(k,2)=max(1,min(MS,repaired(k,2))); anyFix=true;
                        end
                    end
                end
            end
            % (C) 动态障碍 — 3D 方向外推 (横向优先, 高度辅助)
            if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'movingObs')
                for oi=1:length(env.dynObstacles.movingObs)
                    obs=env.dynObstacles.movingObs(oi);
                    op=env.dynObstacles.getPosition(oi,t_k);
                    sR=obs.radius*OBS_CLEAR;
                    pn=repaired(k,:); d=norm(pn-op);
                    if d<sR
                        if d>0.01
                            ev=(pn-op)/d;
                            newPt=op+ev*(sR+5);
                            if newPt(3)>maxH
                                % 高度超限时改为纯横向
                                ev2=[ev(1),ev(2),0]; n2=norm(ev2);
                                if n2>0.01, ev2=ev2/n2;
                                    newPt=op+ev2*(sR+5); newPt(3)=max(minH,min(maxH,pn(3)));
                                end
                            end
                            repaired(k,:)=newPt;
                        else
                            ang=rand*2*pi;
                            repaired(k,1)=op(1)+sR*cos(ang)+5;
                            repaired(k,2)=op(2)+sR*sin(ang)+5;
                            repaired(k,3)=min(maxH,op(3)+obs.radius*2);
                        end
                        anyFix=true;
                    end
                end
            end
            % 钳制
            repaired(k,1)=max(1,min(MS,repaired(k,1)));
            repaired(k,2)=max(1,min(MS,repaired(k,2)));
            if ~isempty(env.heightMap)
                ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
                gH=env.heightMap(ex,ey);
                repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
            else, repaired(k,3)=max(repaired(k,3),minH); end
            repaired(k,3)=min(repaired(k,3),maxH);
        end
        if ~anyFix,break;end
    end % Phase-1

    % =======================================================================
    % Phase-2: 轻量平滑 (保持首末点; 仅对非动态障碍近邻点平滑)
    % Phase-3: 平滑后强制高度合规
    % 注意: 平滑放在段级 validate 之前, 防止平滑撤销段级修复
    % =======================================================================
    nPts=size(repaired,1);
    for k=2:nPts-1
        repaired(k,:)=0.2*repaired(k-1,:)+0.6*repaired(k,:)+0.2*repaired(k+1,:);
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
            gH=env.heightMap(ex,ey);
            repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
        else, repaired(k,3)=max(repaired(k,3),minH); end
        repaired(k,3)=min(repaired(k,3),maxH);
        repaired(k,1)=max(1,min(MS,repaired(k,1)));
        repaired(k,2)=max(1,min(MS,repaired(k,2)));
    end
    repaired(1,:)=smoothed(1,:); repaired(end,:)=smoothed(end,:);

    % Phase-3: 末尾强制高度合规 (防止平滑把 z 拉低)
    nPts=size(repaired,1);
    for k=2:nPts-1
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
            gH=env.heightMap(ex,ey);
            repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
        else, repaired(k,3)=max(repaired(k,3),minH); end
        repaired(k,3)=min(repaired(k,3),maxH);
    end

    % =======================================================================
    % Phase-4: 段级 validate — 检查每段内部子采样的时空冲突
    %
    % 关键设计 (修订版):
    %   静态建筑违规: 就地抬高端点 z (不插点, 防止路径指数增长)
    %   动态障碍违规: 插入新航路点规避 (有全局总插点上限)
    %   每轮 sg_pass 后重新取 t_arrivals
    % =======================================================================
    nPts_init = size(repaired,1);
    MAX_TOTAL_INS = nPts_init;  % 全局总插点上限 = 初始路径点数 (防止指数增长)

    for sg_pass = 1:SEG_PASSES
        nPts = size(repaired,1);
        try
            [~, sg_det] = costModel.evaluatePath(repaired, t_start, true);
            sg_tArr = sg_det.t_arrivals;
            if numel(sg_tArr) ~= nPts
                sg_tArr = computeApproxTArr(repaired, t_start);
            end
        catch
            sg_tArr = computeApproxTArr(repaired, t_start);
        end

        inserted    = false;
        total_ins   = 0;     % 本轮总插点数 (静态只抬高不计, 动态才计)

        for k = 1:size(repaired,1)-1
            % 全局插点上限: 防止动态障碍插点使路径指数增长
            if total_ins >= MAX_TOTAL_INS, break; end

            nPts = size(repaired,1);
            p1 = repaired(k,:);   t1 = sg_tArr(min(k,   numel(sg_tArr)));
            p2 = repaired(k+1,:); t2 = sg_tArr(min(k+1, numel(sg_tArr)));

            hit_type   = '';
            hit_s      = 0;
            hit_obs    = 0;
            hit_pt     = [];
            hit_op     = [];
            hit_viol_z = 0;

            for s = 1:SEG_NSUB
                frac = s/(SEG_NSUB+1);
                pt_s = p1 + frac*(p2-p1);
                t_s  = t1 + frac*(t2-t1);

                % (a) 静态建筑
                if ~isempty(env.heightMap)
                    rx=max(1,min(MS,round(pt_s(1)))); ry=max(1,min(MS,round(pt_s(2))));
                    gH_s=env.heightMap(rx,ry);
                    minFloor_s=max(minH, gH_s+cl);
                    if pt_s(3) < gH_s+3 || pt_s(3) < minFloor_s
                        hit_type='static'; hit_s=s; hit_pt=pt_s;
                        hit_viol_z=max(minFloor_s, gH_s+cl+3);
                        break;
                    end
                end

                % (b) 动态障碍
                if ~isempty(env.dynObstacles)&&isfield(env.dynObstacles,'movingObs')
                    for oi=1:length(env.dynObstacles.movingObs)
                        obs=env.dynObstacles.movingObs(oi);
                        op=env.dynObstacles.getPosition(oi,t_s);
                        if norm(pt_s-op) < obs.radius
                            hit_type='dyn'; hit_s=s; hit_obs=oi;
                            hit_pt=pt_s; hit_op=op;
                            break;
                        end
                    end
                    if strcmp(hit_type,'dyn'), break; end
                end
            end

            if isempty(hit_type), continue; end

            % ====== 修复分支 ======
            if strcmp(hit_type, 'static')
                % 静态建筑: 就地抬高段两端点的 z (不插点, 不增加路径长度)
                for fix_end = [k, k+1]
                    if fix_end < 1 || fix_end > nPts, continue; end
                    if fix_end == 1 || fix_end == nPts, continue; end  % 保护首末点
                    pf = repaired(fix_end,:);
                    pf_rx=max(1,min(MS,round(pf(1)))); pf_ry=max(1,min(MS,round(pf(2))));
                    gH_fix=env.heightMap(pf_rx,pf_ry);
                    newZ = min(maxH, max(pf(3), max(hit_viol_z, gH_fix+cl+3)));
                    if newZ > pf(3) + 0.1  % 只有实际需要抬高才标记
                        repaired(fix_end,3) = newZ;
                        inserted = true;   % 标记本轮有修改 (触发下一轮重新检查)
                    end
                end

            else  % 动态障碍: 插入新航路点
                obs_hit  = env.dynObstacles.movingObs(hit_obs);
                safe_rad = obs_hit.radius * OBS_CLEAR;
                ev3=hit_pt-hit_op; ev3_n=norm(ev3);
                if ev3_n<0.01, ev3=[1,0,0]; else, ev3=ev3/ev3_n; end
                ev_horiz=[ev3(1),ev3(2),0]; evh_n=norm(ev_horiz);
                if evh_n>0.01, ev_horiz=ev_horiz/evh_n;
                else, ang_r=rand*2*pi; ev_horiz=[cos(ang_r),sin(ang_r),0]; end

                % 推两端点
                for fix_end=[k+1,k]
                    if fix_end<1||fix_end>nPts||fix_end==1||fix_end==nPts, continue; end
                    pf=repaired(fix_end,:);
                    newPf=hit_op+ev_horiz*(safe_rad+5); newPf(3)=pf(3);
                    op_fix=env.dynObstacles.getPosition(hit_obs,sg_tArr(min(fix_end,numel(sg_tArr))));
                    if norm(newPf-op_fix)<obs_hit.radius
                        newPf(3)=min(maxH,op_fix(3)+obs_hit.radius*OBS_CLEAR+5);
                    end
                    newPf(1)=max(1,min(MS,newPf(1))); newPf(2)=max(1,min(MS,newPf(2)));
                    if ~isempty(env.heightMap)
                        rx=max(1,min(MS,round(newPf(1)))); ry=max(1,min(MS,round(newPf(2))));
                        newPf(3)=max(newPf(3),max(minH,env.heightMap(rx,ry)+cl));
                    end
                    newPf(3)=max(newPf(3),minH); newPf(3)=min(newPf(3),maxH);
                    repaired(fix_end,:)=newPf;
                end

                % 插点 (动态障碍才插)
                ins_dyn=hit_op+ev_horiz*(safe_rad+8); ins_dyn(3)=hit_pt(3);
                if norm(ins_dyn-hit_op)<safe_rad, ins_dyn(3)=min(maxH,hit_op(3)+safe_rad+8); end
                ins_dyn(1)=max(1,min(MS,ins_dyn(1))); ins_dyn(2)=max(1,min(MS,ins_dyn(2)));
                if ~isempty(env.heightMap)
                    rx=max(1,min(MS,round(ins_dyn(1)))); ry=max(1,min(MS,round(ins_dyn(2))));
                    ins_dyn(3)=max(ins_dyn(3),max(minH,env.heightMap(rx,ry)+cl));
                end
                ins_dyn(3)=max(ins_dyn(3),minH); ins_dyn(3)=min(ins_dyn(3),maxH);
                repaired=[repaired(1:k,:); ins_dyn; repaired(k+1:end,:)];
                frac_ins=hit_s/(SEG_NSUB+1); t_ins=t1+frac_ins*(t2-t1);
                sg_tArr=[sg_tArr(1:k); t_ins; sg_tArr(k+1:end)];
                inserted=true;
                total_ins=total_ins+1;
            end
        end  % for k (segment loop)

        if ~inserted, break; end
    end  % sg_pass

    % =======================================================================
    % Phase-5: 插点后高度合规扫描 (新插点可能未经高度检查)
    % =======================================================================
    nPts=size(repaired,1);
    for k=2:nPts-1
        repaired(k,1)=max(1,min(MS,repaired(k,1)));
        repaired(k,2)=max(1,min(MS,repaired(k,2)));
        if ~isempty(env.heightMap)
            ex=max(1,min(MS,round(repaired(k,1)))); ey=max(1,min(MS,round(repaired(k,2))));
            gH=env.heightMap(ex,ey);
            repaired(k,3)=max(repaired(k,3),max(minH,gH+cl));
        else, repaired(k,3)=max(repaired(k,3),minH); end
        repaired(k,3)=min(repaired(k,3),maxH);
    end
    repaired(1,:)=smoothed(1,:); repaired(end,:)=smoothed(end,:);
end

%% ====================================================================
%%  ★ 完整代价分解打印函数 (目标1)
%% ====================================================================
