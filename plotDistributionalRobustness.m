function fig = plotDistributionalRobustness(stat_J, stat_E, stat_feasible, stat_env, env_seeds_used, algNames, outputFile)
%PLOTDISTRIBUTIONALROBUSTNESS Plot all-path distributions by environment.
% Three runs within each environment are reduced to a median using all paths.
% Penalty-dominated values remain in the statistics but are marked off-scale.

if nargin < 7 || isempty(outputFile)
    outputFile = 'fig7_distributional_robustness.png';
end
nAlg = numel(algNames);
nEnv = numel(env_seeds_used);
nStat = size(stat_J,2);
colors = [0.75 0.13 0.13; 0.16 0.50 0.73; 0.15 0.63 0.25; ...
          0.49 0.18 0.56; 0.58 0.58 0.58];

Jenv = nan(nAlg,nEnv); Eenv = nan(nAlg,nEnv);
for ei = 1:nEnv
    inEnv = stat_env == env_seeds_used(ei);
    for a = 1:nAlg
        Jenv(a,ei) = median(stat_J(a,inEnv),'omitnan');
        Eenv(a,ei) = median(stat_E(a,inEnv),'omitnan');
    end
end

fig = figure('Units','centimeters','Position',[2 2 36 21], ...
    'Color','w','ToolBar','none','MenuBar','none');
sgtitle({sprintf('Distributional Robustness across %d Independent Urban Environments',nEnv), 'Three Runs per Environment; High Complexity; t=0 s'}, 'FontName','Times New Roman','FontSize',22,'FontWeight','bold');
metrics = {Jenv,Eenv};
titles = {'Unified Cost J_{final}','Energy E (Wh)'};
positions = {[0.070 0.160 0.410 0.660],[0.565 0.160 0.410 0.660]};

for m = 1:2
    ax = axes(fig,'Position',positions{m}); hold(ax,'on');
    ax.Toolbar.Visible = 'off';
    values = []; groups = []; present = [];
    for a = 1:nAlg
        v = metrics{m}(a,:); v = v(isfinite(v) & v > 0);
        if ~isempty(v)
            values = [values; v(:)]; groups = [groups; a*ones(numel(v),1)]; %#ok<AGROW>
            present(end+1) = a; %#ok<AGROW>
        end
    end
    axes(ax);
    boxplot(values,groups,'Positions',present,'Colors',colors(present,:), ...
        'Widths',0.66,'Symbol','o','OutlierSize',7);
    set(findobj(ax,'Type','line'),'LineWidth',1.9);
    boxes = findobj(ax,'Tag','Box');
    for bi = 1:numel(boxes)
        a = present(end-bi+1);
        patch(ax,get(boxes(bi),'XData'),get(boxes(bi),'YData'),colors(a,:), ...
            'FaceAlpha',0.45,'EdgeColor',colors(a,:),'LineWidth',2.0);
    end
    set(ax,'YScale','log','XLim',[0.42 nAlg+0.58], ...
        'XTick',1:nAlg,'XTickLabel',algNames,'XTickLabelRotation',25, ...
        'TickLabelInterpreter','none','FontName','Times New Roman', ...
        'FontSize',18,'LineWidth',1.1,'TickDir','out');
    grid(ax,'on'); box(ax,'on'); ax.GridAlpha=0.22; ax.MinorGridAlpha=0.12; ax.YMinorGrid='on';
    nonGreedy = metrics{m}(1:nAlg-1,:);
    nonGreedy = nonGreedy(isfinite(nonGreedy) & nonGreedy > 0);
    ylim(ax,[min(nonGreedy)*0.68 max(nonGreedy)*2.35]);
    yl=ylim(ax); logSpan=log(yl(2))-log(yl(1));
    yCount=exp(log(yl(1))+0.955*logSpan); yMean=exp(log(yl(1))+0.835*logSpan); yNone=exp(log(yl(1))+0.63*logSpan);
    for a = 1:nAlg
        v=metrics{m}(a,:); v=v(isfinite(v) & v > 0);
        if a < nAlg
            mu=mean(v);
            plot(ax,a,mu,'o','MarkerSize',14,'MarkerFaceColor','white','MarkerEdgeColor','k','LineWidth',2.2);
            plot(ax,a,mu,'o','MarkerSize',7,'MarkerFaceColor',colors(a,:),'MarkerEdgeColor',0.6*colors(a,:));
            text(ax,a,yMean,sprintf('mean %.2f',mu),'FontName','Times New Roman', ...
                'FontSize',14,'FontWeight','bold','Color',0.55*colors(a,:), ...
                'HorizontalAlignment','center','VerticalAlignment','middle', ...
                'BackgroundColor','w','Margin',0.5,'EdgeColor',[0.86 0.86 0.86], ...
                'Clipping','on');
        else
            plot(ax,a,yNone,'^','MarkerSize',12,'MarkerFaceColor',colors(a,:), ...
                'MarkerEdgeColor',0.6*colors(a,:),'LineWidth',1.2,'Clipping','on');
            text(ax,a,yNone,sprintf('off-scale\nmean = %.1f',mean(v)), ...
                'HorizontalAlignment','center','VerticalAlignment','top', ...
                'FontName','Times New Roman','FontSize',14,'FontWeight','bold', ...
                'Color',0.48*colors(a,:),'BackgroundColor','w', ...
                'EdgeColor',colors(a,:),'Margin',2,'Interpreter','none');
        end
        text(ax,a,yCount,sprintf('feas. %d/%d',sum(stat_feasible(a,:)),nStat), ...
            'HorizontalAlignment','center','FontName','Times New Roman','FontSize',13, ...
            'FontWeight','bold','Color',[0.76 0.08 0.08],'BackgroundColor','w', ...
            'EdgeColor',[0.86 0.25 0.25],'Margin',1);
    end
    title(ax,titles{m},'FontName','Times New Roman','FontSize',21,'FontWeight','bold');
    ylabel(ax,[titles{m},' (log scale)'],'FontName','Times New Roman','FontSize',20,'FontWeight','bold');
end
annotation(fig,'textbox',[0.16 0.025 0.68 0.050], ...
    'String',['Boxes summarize all paths after environment-level aggregation; ', ...
              'penalty-dominated values are marked off-scale.'], ...
    'HorizontalAlignment','center','VerticalAlignment','middle','FontName','Times New Roman', ...
    'FontSize',16,'FontAngle','italic','LineStyle','none');
exportPublicationFigure(fig,outputFile);
end
