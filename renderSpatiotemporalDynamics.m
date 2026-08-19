function fig = renderSpatiotemporalDynamics(env, paths, departureTimes, ...
        departColors, startPt, goalPt, mapSize)
%RENDERSPATIOTEMPORALDYNAMICS Render archived environment and altitude states.

    buildings = env.buildings;
    movObs = env.dynObstacles.movingObs;
    tempNFZ = env.dynObstacles.tempNFZ;
    departLabels = arrayfun(@(t) sprintf('t=%ds', t), departureTimes, ...
        'UniformOutput', false);
    showTimes = [1, 3, 5];

    fig = figure('Units','centimeters','Position',[0.5 0.5 38 21.5], ...
        'Color','w','Renderer','painters');
    layout = tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');

    for si = 1:3
        d = showTimes(si);
        tDep = departureTimes(d);
        ax = nexttile(layout,si);
        hold(ax,'on'); axis(ax,'equal');
        set(ax,'FontSize',18,'FontName','Times New Roman');

        for i = 1:size(buildings,1)
            cx=buildings(i,1); cy=buildings(i,2);
            hw=buildings(i,4); hh=buildings(i,5);
            rectangle(ax,'Position',[cx-hw,cy-hh,2*hw,2*hh], ...
                'FaceColor',[0.78 0.78 0.78 0.8], ...
                'EdgeColor',[0.5 0.5 0.5],'LineWidth',0.2);
        end

        wStep=60;
        [wx,wy]=meshgrid(wStep:wStep:mapSize-wStep, ...
            wStep:wStep:mapSize-wStep);
        wu=zeros(size(wx)); wv=zeros(size(wy));
        for ii=1:numel(wx)
            wind=env.windField.getWind(wx(ii),wy(ii),60,tDep);
            wu(ii)=wind(1); wv(ii)=wind(2);
        end
        quiver(ax,wx,wy,wu,wv,1.2,'Color',[0.4 0.6 0.9 0.5], ...
            'LineWidth',0.5);

        for i=1:numel(movObs)
            pos=env.dynObstacles.getPosition(i,tDep);
            theta=linspace(0,2*pi,20);
            fill(ax,pos(1)+movObs(i).radius*cos(theta), ...
                pos(2)+movObs(i).radius*sin(theta),[1 0.6 0.2], ...
                'FaceAlpha',0.5,'EdgeColor',[0.85 0.45 0], ...
                'LineWidth',0.6);
        end
        for i=1:numel(tempNFZ)
            nfz=tempNFZ(i);
            if tDep>=nfz.t_start && tDep<=nfz.t_end
                theta=linspace(0,2*pi,40);
                fill(ax,nfz.center(1)+nfz.radius*cos(theta), ...
                    nfz.center(2)+nfz.radius*sin(theta),[1 0.15 0.15], ...
                    'FaceAlpha',0.2,'EdgeColor','r','LineWidth',1.2, ...
                    'LineStyle','--');
            end
        end

        path=paths{d};
        if ~isempty(path)
            plot(ax,path(:,1),path(:,2),'-','Color',departColors(d,:), ...
                'LineWidth',2.8);
        end
        plot(ax,startPt(1),startPt(2),'p','MarkerSize',12, ...
            'MarkerFaceColor',[0 0.7 0],'MarkerEdgeColor','k');
        plot(ax,goalPt(1),goalPt(2),'h','MarkerSize',12, ...
            'MarkerFaceColor',[1 0 0],'MarkerEdgeColor','k');
        title(ax,sprintf('Wind & Obstacles at %s',departLabels{d}), ...
            'FontSize',20,'FontWeight','bold');
        xlabel(ax,'X (m)','FontSize',20);
        ylabel(ax,'Y (m)','FontSize',20);
        xlim(ax,[0 mapSize]); ylim(ax,[0 mapSize]);
        grid(ax,'on'); box(ax,'on');
    end

    for si = 1:3
        d=showTimes(si);
        ax=nexttile(layout,3+si);
        hold(ax,'on');
        set(ax,'FontSize',18,'FontName','Times New Roman');
        path=paths{d};
        if ~isempty(path)
            cumDist=zeros(size(path,1),1);
            for k=2:size(path,1)
                cumDist(k)=cumDist(k-1)+ ...
                    norm(path(k,1:2)-path(k-1,1:2));
            end
            plot(ax,cumDist,path(:,3),'-','Color',departColors(d,:), ...
                'LineWidth',2.5);
        end
        yline(ax,30,'k--','LineWidth',0.7);
        yline(ax,120,'k--','LineWidth',0.7);
        xlabel(ax,'Horizontal Distance (m)','FontSize',20);
        ylabel(ax,'Altitude (m)','FontSize',20);
        title(ax,sprintf('Altitude Profile (%s)',departLabels{d}), ...
            'FontSize',20,'FontWeight','bold');
        grid(ax,'on'); box(ax,'on'); ylim(ax,[0 160]);
    end

    title(layout,'Spatial-Temporal Environment Dynamics and Adaptive Path Generation', ...
        'FontSize',23,'FontWeight','bold','FontName','Times New Roman');
end
