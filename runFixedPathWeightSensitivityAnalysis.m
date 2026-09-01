function outputs=runFixedPathWeightSensitivityAnalysis(cohortFile,userOpts)
% Rescore archived 0.75 m paths after one-at-a-time coefficient changes.
projectDir=fileparts(mfilename('fullpath'));
if nargin<1||isempty(cohortFile)
    cohortFile=fullfile(projectDir,'main_experiment_cohort.mat');
end
if nargin<2, userOpts=struct(); end
opts=struct('OutputDir',fullfile(fileparts(cohortFile), ...
    'fixed_path_weight_sensitivity_results'),'MakeFigure',true,'CheckOnly',false);
names=fieldnames(userOpts);
for i=1:numel(names)
    if ~isfield(opts,names{i})
        error('FixedPathWeights:UnknownOption','Unknown option: %s',names{i});
    end
    opts.(names{i})=userOpts.(names{i});
end
if ~isfile(cohortFile)
    error('FixedPathWeights:MissingCohort','Cohort not found: %s',cohortFile);
end
S=load(cohortFile);
required={'algNames','stat_env','stat_final_evaluation_details'};
for i=1:numel(required)
    if ~isfield(S,required{i})
        error('FixedPathWeights:MissingField','Missing %s.',required{i});
    end
end
sampling=resolveCohortEvaluationSettings(S);
if abs(sampling.PlanningSpacingM-0.75)>1e-12|| ...
        abs(sampling.FinalSpacingM-0.75)>1e-12
    error('FixedPathWeights:WrongCohort','The audit requires the final 0.75 m cohort.');
end
configNames=string({'Base';'Energy 0.5x';'Energy 1.5x';'Time 0.5x'; ...
    'Time 1.5x';'Climb 0.5x';'Climb 1.5x';'Risk 0.5x'; ...
    'Risk 1.5x';'Penalty 0.5x';'Penalty 1.5x'});
changed=string({'None';'Energy';'Energy';'Time';'Time';'Climb';'Climb'; ...
    'Risk';'Risk';'Penalty';'Penalty'});
multiplier=[1;0.5;1.5;0.5;1.5;0.5;1.5;0.5;1.5;0.5;1.5];
baseWeights=[1 0.5 2 10 100];
weights=repmat(baseWeights,11,1);
component=[0 1 1 2 2 3 3 4 4 5 5];
for k=2:11
    weights(k,component(k))=baseWeights(component(k))*multiplier(k);
end
if opts.CheckOnly
    outputs=struct('preflight_passed',true,'configuration_count',11, ...
        'algorithm_count',numel(S.algNames),'sampling',sampling, ...
        'cohort_file',cohortFile);
    fprintf('Fixed-path weight-sensitivity inputs passed. No files written.\n');
    return;
end
if ~isfolder(opts.OutputDir), mkdir(opts.OutputDir); end
nAlg=numel(S.algNames); nCase=numel(S.stat_env);
uniqueTrial=true(nAlg,nCase);
if isfield(S,'stat_is_unique_trial')
    uniqueTrial=logical(S.stat_is_unique_trial);
end
algorithmSeeds=nan(nAlg,nCase);
if isfield(S,'stat_algorithm_seed')
    algorithmSeeds=S.stat_algorithm_seed;
elseif isfield(S,'stat_ra_seed')
    algorithmSeeds(1,:)=S.stat_ra_seed;
end
rows=repmat(localEmptyRow(),0,1);
baseDifference=[];
for k=1:11
    for a=1:nAlg
        for c=1:nCase
            if ~uniqueTrial(a,c), continue; end
            row=localEmptyRow();
            row.Configuration=configNames(k);
            row.ChangedComponent=changed(k);
            row.Multiplier=multiplier(k);
            row.Algorithm=string(S.algNames{a});
            row.EnvironmentSeed=S.stat_env(c);
            row.AlgorithmSeed=algorithmSeeds(a,c);
            d=S.stat_final_evaluation_details{a,c};
            if isstruct(d)&&isfield(d,'numerically_valid')&&d.numerically_valid
                needed={'E_total','T_total','C_climb','R_dynamic','penalty_total'};
                if ~all(isfield(d,needed))
                    error('FixedPathWeights:IncompleteDetails', ...
                        'Incomplete archived evaluator details.');
                end
                row.EvaluationAvailable=true;
                row.Feasible=logical(d.feasible);
                row.EnergyWh=d.E_total;
                row.TimeS=d.T_total;
                row.ClimbCost=d.C_climb;
                row.DynamicRisk=d.R_dynamic;
                row.TotalPenalty=d.penalty_total;
                row.RescoredJ=weights(k,1)*row.EnergyWh+ ...
                    weights(k,2)*row.TimeS/60+weights(k,3)*row.ClimbCost+ ...
                    weights(k,4)*row.DynamicRisk+weights(k,5)*row.TotalPenalty;
                if isfield(d,'J_final'), row.ArchivedJ=d.J_final; end
                if k==1&&isfinite(row.ArchivedJ)
                    baseDifference(end+1,1)=abs(row.RescoredJ-row.ArchivedJ); %#ok<AGROW>
                end
            end
            rows(end+1,1)=row; %#ok<AGROW>
        end
    end
end
caseResults=struct2table(rows);
if isempty(baseDifference)||max(baseDifference)>1e-6
    error('FixedPathWeights:BaseMismatch', ...
        'Nominal rescoring does not reproduce the archived score.');
end
summaryRows=repmat(localEmptySummary(),0,1);
for k=1:11
    for a=1:nAlg
        idx=caseResults.Configuration==configNames(k)& ...
            caseResults.Algorithm==string(S.algNames{a});
        part=caseResults(idx,:);
        row=localEmptySummary();
        row.Configuration=configNames(k);
        row.ChangedComponent=changed(k);
        row.Multiplier=multiplier(k);
        row.Algorithm=string(S.algNames{a});
        row.N=height(part);
        row.EvaluatedN=sum(part.EvaluationAvailable);
        row.FeasibleN=sum(part.Feasible);
        row.FeasibilityRate=row.FeasibleN/row.N;
        row.MedianJAllEvaluated=median( ...
            part.RescoredJ(part.EvaluationAvailable),'omitnan');
        row.MedianJFeasible=median(part.RescoredJ(part.Feasible),'omitnan');
        summaryRows(end+1,1)=row; %#ok<AGROW>
    end
end
summary=struct2table(summaryRows);
ranking=table();
for k=1:11
    part=summary(summary.Configuration==configNames(k),:);
    part.NegativeFeasibility=-part.FeasibilityRate;
    part.MissingMedian=isnan(part.MedianJFeasible);
    part=sortrows(part,{'NegativeFeasibility','MissingMedian','MedianJFeasible'});
    part.Rank=(1:height(part))';
    part.NegativeFeasibility=[];
    part.MissingMedian=[];
    ranking=[ranking;part]; %#ok<AGROW>
end
writetable(caseResults,fullfile(opts.OutputDir,'fixed_path_weight_cases.csv'));
writetable(summary,fullfile(opts.OutputDir,'fixed_path_weight_summary.csv'));
writetable(ranking,fullfile(opts.OutputDir,'fixed_path_weight_ranking.csv'));
figureFiles=struct('png','','pdf','');
if opts.MakeFigure
    figureFiles=localPlot(summary,S.algNames,configNames,opts.OutputDir);
end
provenance=validationProvenance(cohortFile);
interpretation=['Post hoc fixed-path score sensitivity. Archived 0.75 m ', ...
    'planner outputs and evaluator components were held fixed; only one ', ...
    'coefficient at a time was changed. This audit does not measure search ', ...
    'sensitivity, re-optimization performance, or weight optimality.'];
save(fullfile(opts.OutputDir,'fixed_path_weight_sensitivity_results.mat'), ...
    'caseResults','summary','ranking','weights','baseWeights', ...
    'configNames','changed','multiplier','sampling','provenance', ...
    'interpretation','figureFiles');
outputs=struct('caseResults',caseResults,'summary',summary, ...
    'ranking',ranking,'weights',weights,'sampling',sampling, ...
    'interpretation',interpretation,'figureFiles',figureFiles, ...
    'output_dir',opts.OutputDir,'cohort_file',cohortFile);
fprintf('\nFixed-path weight sensitivity complete: %s\n',opts.OutputDir);
fprintf('No planner was rerun; archived physical outputs were rescored only.\n');
end

function row=localEmptyRow()
row=struct('Configuration',string(''),'ChangedComponent',string(''), ...
    'Multiplier',NaN,'Algorithm',string(''),'EnvironmentSeed',NaN, ...
    'AlgorithmSeed',NaN,'EvaluationAvailable',false,'Feasible',false, ...
    'EnergyWh',NaN,'TimeS',NaN,'ClimbCost',NaN,'DynamicRisk',NaN, ...
    'TotalPenalty',NaN,'RescoredJ',NaN,'ArchivedJ',NaN);
end

function row=localEmptySummary()
row=struct('Configuration',string(''),'ChangedComponent',string(''), ...
    'Multiplier',NaN,'Algorithm',string(''),'N',0,'EvaluatedN',0, ...
    'FeasibleN',0,'FeasibilityRate',NaN,'MedianJAllEvaluated',NaN, ...
    'MedianJFeasible',NaN);
end

function files=localPlot(summary,algNames,configNames,outDir)
fig=figure('Color','w','Position',[100 100 1400 620]);
ax=axes(fig); hold(ax,'on');
colors=lines(numel(algNames));
for a=1:numel(algNames)
    part=summary(summary.Algorithm==string(algNames{a}),:);
    [~,order]=ismember(part.Configuration,configNames);
    [~,sortIndex]=sort(order);
    values=part.MedianJFeasible(sortIndex);
    base=values(1);
    if isfinite(base)&&base~=0, values=values/base; end
    plot(ax,1:numel(configNames),values,'-o','LineWidth',1.6, ...
        'MarkerSize',5,'Color',colors(a,:),'DisplayName',algNames{a});
end
yline(ax,1,':','Color',[0.35 0.35 0.35],'HandleVisibility','off');
xlim(ax,[0.5 numel(configNames)+0.5]);
grid(ax,'on'); box(ax,'on');
ax.XTick=1:numel(configNames);
ax.XTickLabel=cellstr(configNames);
ax.XTickLabelRotation=30;
ax.FontSize=11;
xlabel(ax,'One-at-a-time coefficient setting');
ylabel(ax,'Median feasible J / nominal median feasible J');
title(ax,'Fixed-Path Composite-Score Sensitivity');
legend(ax,'Location','eastoutside');
png=fullfile(outDir,'fig_fixed_path_weight_sensitivity.png');
pdf=fullfile(outDir,'fig_fixed_path_weight_sensitivity.pdf');
exportgraphics(fig,png,'Resolution',600);
exportgraphics(fig,pdf,'ContentType','vector');
close(fig);
files=struct('png',png,'pdf',pdf);
end
