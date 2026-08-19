function fig = plotClusterAwareStatistics(results, outputFile)
%PLOTCLUSTERAWARESTATISTICS Plot environment-level Section 5.5 inference.
%   Each urban environment is one independent block. The three algorithmic
%   seeds are aggregated within environment before exact paired inference.

    if nargin < 2 || isempty(outputFile)
        outputFile = 'fig8_statistical_significance.png';
    end
    if ~isfield(results,'continuous') || isempty(results.continuous)
        error('ClusterPlot:MissingResults', ...
            'results.continuous is required. Run runClusterAwareStatistics first.');
    end

    C = results.continuous;
    metrics = {'Composite cost','Energy'};
    metricTitles = {'Unified Cost J','Energy E'};
    colors = [0.16 0.50 0.73; 0.15 0.63 0.25; ...
              0.49 0.18 0.56; 0.58 0.58 0.58];

    fig = figure('Units','centimeters','Position',[1 1 48 30], ...
        'Color','w','ToolBar','none','MenuBar','none');
    layout = tiledlayout(2,2,'TileSpacing','loose','Padding','loose');
    title(layout,'Environment-Level Paired Inference (N = 10 Environments)', ...
        'FontSize',22,'FontWeight','bold','FontName','Times New Roman');

    for m = 1:numel(metrics)
        rows = strcmp(C.metric,metrics{m});
        T = C(rows,:);
        labels = T.baseline;
        n = height(T);
        barColors = colors(1:n,:);
        y = 1:n;

        % Holm-adjusted exact signed-rank p-values.
        axP = nexttile(layout,m);
        hold(axP,'on');
        p = T.p_holm;
        pPlot = max(p,1e-4);
        b = barh(axP,y,pPlot,0.58,'FaceColor','flat','EdgeColor','k', ...
            'LineWidth',0.8,'BaseValue',1e-4);
        b.CData = barColors;
        xline(axP,0.05,'--','Color',[0.78 0.12 0.12],'LineWidth',1.6);
        set(axP,'XScale','log','XLim',[1e-4 1], ...
            'YLim',[0.4 n+0.6],'YTick',y,'YTickLabel',labels, ...
            'TickLabelInterpreter','none','FontName','Times New Roman', ...
            'FontSize',19,'LineWidth',1.1,'TickDir','out');

        for i = 1:n
            if p(i) < 0.001
                valueLabel = 'p_{Holm} < 0.001';
            else
                valueLabel = sprintf('p_{Holm} = %.3f',p(i));
            end
            text(axP,0.82,i,valueLabel,'HorizontalAlignment','right', ...
                'VerticalAlignment','middle','FontSize',17, ...
                'FontWeight','bold','FontName','Times New Roman', ...
                'BackgroundColor','w','Margin',1,'Interpreter','tex');
        end

        xlabel(axP,'Holm-adjusted p-value (dashed line: \alpha = 0.05)', ...
            'FontSize',20,'FontWeight','bold');
        ylabel(axP,'Baseline planner','FontSize',20,'FontWeight','bold');
        title(axP,['Exact Paired Signed-Rank Test: ',metricTitles{m}], ...
            'FontSize',21,'FontWeight','bold');
        grid(axP,'on'); box(axP,'on');

        % Matched-pairs rank-biserial effect size.
        axR = nexttile(layout,m+2);
        hold(axR,'on');
        effect = T.rank_biserial;
        b = barh(axR,y,effect,0.58,'FaceColor','flat', ...
            'EdgeColor','k','LineWidth',0.8);
        b.CData = barColors;
        xline(axR,0,'-','Color','k','LineWidth',1.2);
        set(axR,'XLim',[-1.05 1.05],'YLim',[0.4 n+0.6], ...
            'YTick',y,'YTickLabel',labels,'TickLabelInterpreter','none', ...
            'FontName','Times New Roman','FontSize',19,'LineWidth',1.1,'TickDir','out');

        for i = 1:n
            % Put positive-effect labels left of zero and negative labels right.
            if effect(i) >= 0
                xText = -0.055;
                align = 'right';
            else
                xText = 0.055;
                align = 'left';
            end
            text(axR,xText,i,sprintf('r_{rb} = %+.3f',effect(i)), ...
                'HorizontalAlignment',align,'VerticalAlignment','middle', ...
                'FontSize',17,'FontWeight','bold', ...
                'FontName','Times New Roman','Interpreter','tex', ...
                'BackgroundColor','w','Margin',0.8,'Clipping','on');
        end

        xlabel(axR, ...
            'Matched-pairs rank-biserial effect size, r_{rb}', ...
            'FontSize',20,'FontWeight','bold');
        ylabel(axR,'Baseline planner','FontSize',20,'FontWeight','bold');
        title(axR,['Paired Effect Size: ',metricTitles{m}], ...
            'FontSize',21,'FontWeight','bold');
        grid(axR,'on'); box(axR,'on');
    end

    exportPublicationFigure(fig,outputFile);
end
