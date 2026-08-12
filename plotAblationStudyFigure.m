function plotAblationStudyFigure(abl_J, abl_E, abl_T, abl_R, abl_Pen, ablationNames, N_ABL, outputFile)
%PLOTABLATIONSTUDYFIGURE Draw the four-variant publication ablation figure.

if nargin < 8 || isempty(outputFile)
    outputFile = 'fig9_ablation_study.png';
end

nAbl = numel(ablationNames);
if nAbl ~= 4
    error('The revised ablation figure requires exactly four valid variants.');
end

colors = [0.82 0.10 0.10;
          0.25 0.45 0.78;
          0.18 0.65 0.32;
          0.50 0.50 0.50];

metrics = {abl_J, abl_E, abl_T, abl_R, abl_Pen};
titles = {'Unified Cost', 'Energy', 'Flight Time', ...
          'Dynamic Risk', 'Constraint Penalty'};
ylabels = {'Unified Cost, J', 'Energy, E (Wh)', 'Flight Time, T (s)', ...
           'Dynamic Risk, R', 'Constraint Penalty, P'};
useLog = [true false false true true];
formats = {'%.2f','%.2f','%.1f','%.3f','%.3f'};
xlabels = {'Full','w/o Smooth','w/o Headwind','w/o Top-K Re-eval.'};

fig = figure('Units','centimeters','Position',[2 2 40 26], ...
    'Color','w','ToolBar','none','MenuBar','none');

annotation(fig,'textbox',[0.04 0.948 0.92 0.040], ...
    'String',sprintf('Ablation Analysis (N=%d per Variant; High Complexity; t=0 s)',N_ABL), ...
    'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'FontName','Times New Roman','FontSize',25,'FontWeight','bold', ...
    'LineStyle','none');

positions = {
    [0.055 0.565 0.270 0.315]
    [0.365 0.565 0.270 0.315]
    [0.675 0.565 0.270 0.315]
    [0.055 0.105 0.270 0.315]
    [0.365 0.105 0.270 0.315]
};

for m = 1:5
    ax = axes(fig,'Position',positions{m});
    hold(ax,'on');
    ax.Toolbar.Visible = 'off';

    mu = mean(metrics{m},2,'omitnan');
    sd = std(metrics{m},0,2,'omitnan');

    for aa = 1:nAbl
        bar(ax,aa,mu(aa),0.60,'FaceColor',colors(aa,:), ...
            'FaceAlpha',0.82,'EdgeColor',0.70*colors(aa,:),'LineWidth',1.2);

        lowerErr = sd(aa);
        if useLog(m)
            lowerErr = min(sd(aa),0.80*mu(aa));
        end
        errorbar(ax,aa,mu(aa),lowerErr,sd(aa),'k', ...
            'LineStyle','none','LineWidth',1.2,'CapSize',7);
    end

    if useLog(m)
        set(ax,'YScale','log');
        positiveMu = mu(mu > 0);
        lowerCandidates = mu - min(sd,0.80*mu);
        lowerCandidates = lowerCandidates(lowerCandidates > 0);
        if isempty(lowerCandidates)
            yMin = min(positiveMu)*0.20;
        else
            yMin = min(lowerCandidates)*0.70;
        end
        yMax = max(mu+sd)*4.8;
        if ~(isfinite(yMin) && yMin > 0), yMin = 1e-4; end
        if ~(isfinite(yMax) && yMax > yMin), yMax = max(positiveMu)*10; end
        ylim(ax,[yMin yMax]);
    else
        lower = min(0,min(mu-sd));
        upper = max(mu+sd);
        span = max(upper-lower,1e-6);
        ylim(ax,[lower-0.03*span, upper+0.42*span]);
    end

    yline(ax,mu(1),'--','Color',[0.86 0.25 0.25], ...
        'LineWidth',1.1,'Alpha',0.75);

    limits = ylim(ax);
    for aa = 1:nAbl
        if useLog(m)
            yText = (mu(aa)+sd(aa))*1.22;
        else
            yText = mu(aa)+sd(aa)+0.045*(limits(2)-limits(1));
        end

        if aa == 1
            label = sprintf(formats{m},mu(aa));
        else
            delta = 100*(mu(aa)-mu(1))/mu(1);
            label = sprintf([formats{m} '\n(%+.1f%%)'],mu(aa),delta);
        end

        text(ax,aa,yText,label,'HorizontalAlignment','center', ...
            'VerticalAlignment','bottom','FontName','Times New Roman', ...
            'FontSize',17,'FontWeight','bold','Color',0.65*colors(aa,:), ...
            'BackgroundColor','w','Margin',0.5,'Clipping','on');
    end

    xlim(ax,[0.45 nAbl+0.55]);
    set(ax,'XTick',1:nAbl,'XTickLabel',xlabels, ...
        'XTickLabelRotation',20,'FontName','Times New Roman', ...
        'FontSize',19,'LineWidth',1.0,'TickDir','out');
    title(ax,titles{m},'FontName','Times New Roman', ...
        'FontSize',21,'FontWeight','bold');
    ylabel(ax,ylabels{m},'FontName','Times New Roman','FontSize',19,'FontWeight','bold');
    grid(ax,'on');
    ax.GridAlpha = 0.18;
    ax.MinorGridAlpha = 0.10;
    if useLog(m), ax.YMinorGrid = 'on'; end
    box(ax,'on');
end

axLegend = axes(fig,'Position',[0.675 0.105 0.270 0.315]);
axis(axLegend,'off');
axLegend.Toolbar.Visible = 'off';
xlim(axLegend,[0 1]);
ylim(axLegend,[0 1]);
text(axLegend,0.50,0.94,'Legend','HorizontalAlignment','center', ...
    'FontName','Times New Roman','FontSize',22,'FontWeight','bold');

legendNames = {'Full RA-ALA','w/o Smooth','w/o Headwind Guidance','w/o Top-K Re-evaluation'};
legendDescriptions = {'complete method', ...
    'no smoothness guidance', ...
    'no headwind look-ahead guidance', ...
    'internal-best raw path without Top-K candidate re-evaluation'};
yPositions = [0.77 0.57 0.37 0.17];

for aa = 1:nAbl
    rectangle(axLegend,'Position',[0.06 yPositions(aa)-0.045 0.09 0.09], ...
        'FaceColor',colors(aa,:),'EdgeColor',0.70*colors(aa,:), ...
        'LineWidth',1.1);
    text(axLegend,0.19,yPositions(aa)+0.020,legendNames{aa}, ...
        'FontName','Times New Roman','FontSize',18,'FontWeight','bold', ...
        'Color',0.65*colors(aa,:),'VerticalAlignment','middle');
    text(axLegend,0.19,yPositions(aa)-0.027,legendDescriptions{aa}, ...
        'FontName','Times New Roman','FontSize',15,'FontAngle','italic', ...
        'Color',[0.25 0.25 0.25],'VerticalAlignment','middle');
end

exportPublicationFigure(fig,outputFile);
end

