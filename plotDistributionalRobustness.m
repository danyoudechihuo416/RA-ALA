function fig = plotDistributionalRobustness(stat_J, stat_E, stat_feasible, stat_env, env_seeds_used, algNames, outputFile, uniqueTrial)
%PLOTDISTRIBUTIONALROBUSTNESS Plot finite evaluated-output distributions.
% Stochastic runs within each environment are reduced to a median; deterministic
% planners contribute their single unique run rather than copied plotting slots.
% No-path failures contribute no score or energy value. Penalty-dominated finite
% values remain in the statistics but are marked off-scale.

if nargin < 7 || isempty(outputFile)
    outputFile = 'fig7_distributional_robustness.png';
end
if nargin < 8 || isempty(uniqueTrial)
    uniqueTrial = true(size(stat_feasible));
end
if ~isequal(size(uniqueTrial),size(stat_feasible))
    error('uniqueTrial must have the same size as stat_feasible.');
end
nAlg = numel(algNames);
nEnv = numel(env_seeds_used);
colors = [0.75 0.13 0.13; 0.16 0.50 0.73; 0.15 0.63 0.25; ...
          0.49 0.18 0.56; 0.58 0.58 0.58];

Jenv = nan(nAlg,nEnv); Eenv = nan(nAlg,nEnv);
for ei = 1:nEnv
    inEnv = stat_env == env_seeds_used(ei);
    for a = 1:nAlg
        use = inEnv & uniqueTrial(a,:);
        Jenv(a,ei) = median(stat_J(a,use),'omitnan');
        Eenv(a,ei) = median(stat_E(a,use),'omitnan');
    end
end

fig = figure('Units','centimeters','Position',[2 2 36 21], ...
    'Color','w','ToolBar','none','MenuBar','none');
sgtitle({'Evaluated-Output Distributions', ...
    sprintf('High complexity; %d environments; t_0 = 0 s',nEnv)}, ...
    'FontName','Times New Roman','FontSize',22,'FontWeight','bold');
metrics = {Jenv,Eenv};
titles = {'Composite Score J_{final}','Energy E (Wh)'};
positions = {[0.070 0.195 0.410 0.630],[0.565 0.195 0.410 0.630]};

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
    topPadding = 2.35;
    if m == 1, topPadding = 4; end
    ylim(ax,[min(nonGreedy)*0.68 max(nonGreedy)*topPadding]);
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
        use = uniqueTrial(a,:);
        finiteEnvN = sum(isfinite(metrics{m}(a,:)) & metrics{m}(a,:) > 0);
        text(ax,a,yCount,sprintf('N = %d\n%d/%d feas.', ...
            finiteEnvN,sum(stat_feasible(a,use)),sum(use)), ...
            'HorizontalAlignment','center','VerticalAlignment','top', ...
            'FontName','Times New Roman','FontSize',15, ...
            'FontWeight','bold','Color',[0.76 0.08 0.08],'BackgroundColor','w', ...
            'EdgeColor',[0.86 0.25 0.25],'Margin',1);
    end
    title(ax,titles{m},'FontName','Times New Roman','FontSize',21,'FontWeight','bold');
    ylabel(ax,[titles{m},' (log scale)'],'FontName','Times New Roman','FontSize',20,'FontWeight','bold');
    set(ax,'PositionConstraint','innerposition','Position',positions{m});
end
% Preserve the full canvas: tight export can clip rotated algorithm labels.
[outDir,baseName,~] = fileparts(char(outputFile));
if isempty(outDir), outDir = pwd; end
if ~exist(outDir,'dir'), mkdir(outDir); end
set(fig,'PaperUnits','centimeters','PaperSize',[36 21], ...
    'PaperPosition',[0 0 36 21],'PaperPositionMode','manual');
drawnow;
print(fig,fullfile(outDir,[baseName,'.png']),'-dpng','-r600');
print(fig,fullfile(outDir,[baseName,'.pdf']),'-dpdf','-painters');
end
