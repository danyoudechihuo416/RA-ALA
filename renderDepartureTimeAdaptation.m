function fig = renderDepartureTimeAdaptation(env, paths, departureTimes, ...
        departColors, startPt, goalPt, mapSize)
%RENDERDEPARTURETIMEADAPTATION Plot archived routes without rerunning planners.

    nDepart = numel(departureTimes);
    departLabels = arrayfun(@(t) sprintf('t=%ds', t), departureTimes, ...
        'UniformOutput', false);
    buildings = env.buildings;
    movObs = env.dynObstacles.movingObs;
    tempNFZ = env.dynObstacles.tempNFZ;

    fig = figure('Units','centimeters','Position',[0.5 0.5 28 25], ...
        'Color','w','Renderer','painters');
    ax = axes(fig); hold(ax,'on'); axis(ax,'equal');
    set(ax,'FontSize',15,'FontName','Times New Roman');

    for i = 1:size(buildings,1)
        cx=buildings(i,1); cy=buildings(i,2); bh=buildings(i,3);
        hw=buildings(i,4); hh=buildings(i,5);
        gv=max(0.4,0.88-bh/200);
        rectangle(ax,'Position',[cx-hw,cy-hh,2*hw,2*hh], ...
            'FaceColor',[gv gv gv+0.02 0.75], ...
            'EdgeColor',[0.35 0.35 0.35],'LineWidth',0.3, ...
            'HandleVisibility','off');
    end

    hNFZ=gobjects(1);
    for i = 1:numel(tempNFZ)
        nfz=tempNFZ(i); theta=linspace(0,2*pi,60);
        hThis=fill(ax,nfz.center(1)+nfz.radius*cos(theta), ...
            nfz.center(2)+nfz.radius*sin(theta),[1 0.2 0.2], ...
            'FaceAlpha',0.15,'EdgeColor','r','LineWidth',1.2, ...
            'LineStyle','--','HandleVisibility','off');
        if i==1
            hNFZ=hThis;
        end
        text(ax,nfz.center(1),nfz.center(2),sprintf('NFZ%d',i), ...
            'HorizontalAlignment','center','FontSize',11, ...
            'Color',[0.7 0 0],'FontWeight','bold');
    end

    for d = 1:nDepart
        tDep=departureTimes(d);
        for i = 1:numel(movObs)
            pos=env.dynObstacles.getPosition(i,tDep);
            theta=linspace(0,2*pi,20); r=movObs(i).radius;
            plot(ax,pos(1)+r*cos(theta),pos(2)+r*sin(theta),'-', ...
                'Color',[departColors(d,:) 0.35],'LineWidth',0.7, ...
                'HandleVisibility','off');
        end
    end

    hPaths=gobjects(nDepart,1);
    for d = 1:nDepart
        p=paths{d};
        if isempty(p), continue; end
        hPaths(d)=plot(ax,p(:,1),p(:,2),'-', ...
            'Color',departColors(d,:),'LineWidth',2.5);
    end

    plot(ax,startPt(1),startPt(2),'p','MarkerSize',16, ...
        'MarkerFaceColor',[0 0.75 0],'MarkerEdgeColor','k', ...
        'LineWidth',1.2,'HandleVisibility','off');
    plot(ax,goalPt(1),goalPt(2),'h','MarkerSize',16, ...
        'MarkerFaceColor',[1 0 0],'MarkerEdgeColor','k', ...
        'LineWidth',1.2,'HandleVisibility','off');
    text(ax,startPt(1)+25,startPt(2)-30,'Start', ...
        'FontSize',15,'FontWeight','bold');
    text(ax,goalPt(1)-70,goalPt(2)+25,'Goal', ...
        'FontSize',15,'FontWeight','bold');

    valid=isgraphics(hPaths);
    handles=[hPaths(valid);hNFZ];
    labels=[departLabels(valid),{'Temporary NFZ footprint'}];
    lgd=legend(ax,handles,labels,'Location','southoutside', ...
        'Orientation','horizontal','NumColumns',3,'FontSize',22,'Box','on');
    lgd.ItemTokenSize=[26 18];

    text(ax,0.98,0.02,'NFZ activation is route-time dependent; footprints shown over the full mission horizon', ...
        'Units','normalized','HorizontalAlignment','right', ...
        'VerticalAlignment','bottom','FontSize',11,'Color',[0.55 0 0], ...
        'BackgroundColor','w','Margin',2);
    xlabel(ax,'X (m)','FontSize',16); ylabel(ax,'Y (m)','FontSize',16);
    title(ax,{'Temporal Adaptation of RA-ALA across Departure Times', ...
        'Same mission; route-specific environmental exposure'}, ...
        'FontSize',17,'FontWeight','bold');
    xlim(ax,[0 mapSize]); ylim(ax,[0 mapSize]); grid(ax,'on'); box(ax,'on');
end
