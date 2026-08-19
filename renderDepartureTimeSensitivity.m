function fig = renderDepartureTimeSensitivity(metricByTime, departureTimes)
%RENDERDEPARTURETIMESENSITIVITY Plot archived departure-time metrics.

    nDepart = numel(departureTimes);
    if size(metricByTime,1) ~= nDepart || size(metricByTime,2) ~= 4
        error('DepartureTimeSensitivity:InvalidInput', ...
            'metricByTime must contain one row per departure time and four columns.');
    end

    departLabels = arrayfun(@(t) sprintf('t=%ds',t),departureTimes, ...
        'UniformOutput',false);
    metricLabels = {'Cost, J','Energy, E (Wh)','Time, T (s)', ...
        'Dynamic risk, R_{dyn}'};

    fig = figure('Units','centimeters','Position',[0.5 0.5 32 9.5], ...
        'Color','w','Renderer','painters');
    layout = tiledlayout(fig,1,4,'TileSpacing','compact','Padding','compact');

    for m = 1:4
        ax = nexttile(layout); hold(ax,'on');
        set(ax,'FontSize',18,'FontName','Times New Roman','LineWidth',0.9);
        bar(ax,metricByTime(:,m),0.58,'FaceColor',[0.85 0.15 0.15], ...
            'EdgeColor',[0.65 0.08 0.08],'FaceAlpha',0.75,'LineWidth',0.8);
        set(ax,'XTick',1:nDepart,'XTickLabel',departLabels, ...
            'XTickLabelRotation',30);
        ylabel(ax,metricLabels{m},'FontSize',20);
        grid(ax,'on'); box(ax,'on');

        yMax = max(metricByTime(:,m));
        if ~isfinite(yMax) || yMax <= 0
            yMax = 1;
        end
        ylim(ax,[0 1.20*yMax]);

        for d = 1:nDepart
            if m == 4
                valueLabel = sprintf('%.3f',metricByTime(d,m));
            else
                valueLabel = sprintf('%.1f',metricByTime(d,m));
            end
            labelOffset = (0.018 + 0.050*mod(d,2))*yMax;
            text(ax,d,metricByTime(d,m)+labelOffset,valueLabel, ...
                'HorizontalAlignment','center','VerticalAlignment','bottom', ...
                'FontSize',16,'FontName','Times New Roman','FontWeight','normal');
        end
    end

    title(layout, ...
        'Sensitivity of Path Quality to Departure Time in Time-Varying Urban Environments', ...
        'FontSize',21,'FontWeight','bold','FontName','Times New Roman');
end
