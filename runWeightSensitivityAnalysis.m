function outputs = runWeightSensitivityAnalysis(dataFile,userOpts)
%RUNWEIGHTSENSITIVITYANALYSIS RA-ALA one-at-a-time weight sensitivity.
%   Only RA-ALA is re-optimized. This is a within-method calibration audit,
%   not a cross-planner ranking experiment. Composite scores from different
%   weight definitions are reported for traceability but are not compared.

    projectDir = fileparts(mfilename('fullpath'));
    if nargin < 2, userOpts = struct(); end
    opts = struct('OutputDir',fullfile(projectDir,'ra_weight_sensitivity_results'), ...
        'MakeFigure',true,'Resume',true,'CheckOnly',false);
    names = fieldnames(userOpts);
    for i=1:numel(names)
        if ~isfield(opts,names{i})
            error('WeightSensitivity:UnknownOption','Unknown option: %s',names{i});
        end
        opts.(names{i})=userOpts.(names{i});
    end
    switches = {'MakeFigure','Resume','CheckOnly'};
    for i=1:numel(switches)
        validateattributes(opts.(switches{i}),{'logical','numeric'}, ...
            {'scalar','binary'},mfilename,switches{i});
        opts.(switches{i})=logical(opts.(switches{i}));
    end
    if nargin < 1 || isempty(dataFile)
        dataFile=fullfile(projectDir,'main_experiment_cohort.mat');
    elseif ~isfile(dataFile)
        dataFile=fullfile(projectDir,dataFile);
    end
    if ~isfile(dataFile)
        error('WeightSensitivity:MissingCohort', ...
            'Authoritative Figure 7-8 cohort not found: %s',dataFile);
    end
    S=load(dataFile);
    required={'env_seeds_used','stat_env','stat_ra_seed','stat_J', ...
        'ala_cfg_stat','mapSize','gridStep','windLevel','riskLevel', ...
        'startPt','goalPt','main_collision_sample_spacing_m'};
    for i=1:numel(required)
        if ~isfield(S,required{i})
            error('WeightSensitivity:MissingField','Missing %s in %s.', ...
                required{i},dataFile);
        end
    end
    sampling=resolveCohortEvaluationSettings(S);
    if abs(sampling.PlanningSpacingM-0.75)>eps || ...
            abs(sampling.FinalSpacingM-0.75)>eps
        error('WeightSensitivity:WrongCohort', ...
            'RA-ALA weight sensitivity requires the 0.75 m Figure 7-8 cohort.');
    end
    provenance=validationProvenance(dataFile);

    envSeeds=S.env_seeds_used(:)';
    nEnv=numel(envSeeds);
    raSeeds=nan(1,nEnv);
    baseStatIndex=nan(1,nEnv);
    for ei=1:nEnv
        idx=find(S.stat_env==envSeeds(ei),1,'first');
        if isempty(idx)
            error('WeightSensitivity:MissingSeed', ...
                'No RA-ALA seed found for environment %d.',envSeeds(ei));
        end
        baseStatIndex(ei)=idx;
        raSeeds(ei)=S.stat_ra_seed(idx);
    end
    cfg=S.ala_cfg_stat;
    [configTable,weightConfigs]=localBuildWeightConfigs();
    nConfig=height(configTable);
    if opts.CheckOnly
        outputs=struct('preflight_passed',true,'cohort_file',dataFile, ...
            'sampling',sampling,'environment_count',nEnv, ...
            'configuration_count',nConfig,'options',opts);
        fprintf('RA-ALA weight-sensitivity input checks passed. No planning started.\n');
        return;
    end
    if ~exist(opts.OutputDir,'dir'), mkdir(opts.OutputDir); end
    writetable(configTable,fullfile(opts.OutputDir, ...
        'ra_weight_sensitivity_configurations.csv'));

    nRows=nConfig*nEnv;
    ConfigID=zeros(nRows,1);
    ConfigName=cell(nRows,1);
    VariedComponent=cell(nRows,1);
    Multiplier=nan(nRows,1);
    EnvironmentID=zeros(nRows,1);
    EnvironmentSeed=zeros(nRows,1);
    AlgorithmSeed=zeros(nRows,1);
    J=nan(nRows,1);
    Energy_Wh=nan(nRows,1);
    FlightTime_s=nan(nRows,1);
    ClimbCost=nan(nRows,1);
    DynamicRisk=nan(nRows,1);
    PenaltyTotal=nan(nRows,1);
    Feasible=false(nRows,1);
    PlanningTime_s=nan(nRows,1);
    EvaluationTime_s=nan(nRows,1);
    RunStatus=repmat({'not_run'},nRows,1);
    completedEnv=false(nEnv,1);

    sourceText=strrep(fileread([mfilename('fullpath'),'.m']), ...
        sprintf('\r\n'),sprintf('\n'));
    checkpointDesign=struct('version',3,'analysis','RA-ALA-only', ...
        'provenance',provenance,'sampling',sampling,'configuration',cfg, ...
        'raSeeds',raSeeds,'source',sourceText);
    checkpointFile=fullfile(opts.OutputDir, ...
        'ra_weight_sensitivity_checkpoint_v3.mat');
    if opts.Resume && isfile(checkpointFile)
        C=load(checkpointFile);
        sameDesign=isfield(C,'checkpointEnvSeeds') && ...
            isequal(C.checkpointEnvSeeds(:)',envSeeds) && ...
            isfield(C,'checkpointConfigTable') && ...
            isequaln(C.checkpointConfigTable,configTable) && ...
            isfield(C,'checkpointDesign') && ...
            isequaln(C.checkpointDesign,checkpointDesign);
        if ~sameDesign
            error('WeightSensitivity:CheckpointMismatch', ...
                ['The RA-ALA checkpoint belongs to another code/cohort design. ', ...
                'Choose another OutputDir or archive the old checkpoint.']);
        end
        fields={'ConfigID','ConfigName','VariedComponent','Multiplier', ...
            'EnvironmentID','EnvironmentSeed','AlgorithmSeed','J','Energy_Wh', ...
            'FlightTime_s','ClimbCost','DynamicRisk','PenaltyTotal','Feasible', ...
            'PlanningTime_s','EvaluationTime_s','RunStatus','completedEnv'};
        for i=1:numel(fields), eval([fields{i},'=C.',fields{i},';']); end
        fprintf('Resuming RA-ALA weight sensitivity: %d/%d environments complete.\n', ...
            sum(completedEnv),nEnv);
    end

    fprintf('\nRA-ALA one-at-a-time weight sensitivity\n');
    fprintf('  Design: %d environments x %d configurations\n',nEnv,nConfig);
    fprintf('  Planning/final spacing: %.3g/%.3g m\n', ...
        sampling.PlanningSpacingM,sampling.FinalSpacingM);
    fprintf('  Baseline planners are not rerun in this within-method analysis.\n');

    for ei=1:nEnv
        if completedEnv(ei), continue; end
        envSeed=envSeeds(ei);
        rng(envSeed,'twister');
        env=CityEnvironment(S.mapSize,S.gridStep);
        env.generate('high',S.windLevel,S.riskLevel,envSeed);
        env.setTaskPoints(S.startPt,S.goalPt);
        fprintf('\nEnvironment %d/%d (seed=%d)\n',ei,nEnv,envSeed);

        for ci=1:nConfig
            row=(ci-1)*nEnv+ei;
            ConfigID(row)=ci;
            ConfigName{row}=configTable.ConfigName{ci};
            VariedComponent{row}=configTable.VariedComponent{ci};
            Multiplier(row)=configTable.Multiplier(ci);
            EnvironmentID(row)=ei;
            EnvironmentSeed(row)=envSeed;
            AlgorithmSeed(row)=raSeeds(ei);

            cm=localCreateCostModel(env,weightConfigs(ci), ...
                sampling.PlanningSpacingM,sampling.MinSamples);
            cmFinal=localCreateCostModel(env,weightConfigs(ci), ...
                sampling.FinalSpacingM,sampling.MinSamples);
            planner=PathPlanners(env,cm);
            planner.setBudget(15,5000,2000);
            rng(raSeeds(ei),'twister');
            timer=tic;
            try
                evalc('[path,~,detSearch,stage] = runRA_ALA(planner,cm,env,S.startPt,S.goalPt,0,true,cfg);');
                PlanningTime_s(row)=toc(timer);
                if isempty(path) || size(path,1)<2
                    error('WeightSensitivity:EmptyPath','RA-ALA returned no path.');
                end
                if isfield(stage,'timing') && isfield(stage.timing,'total_s')
                    PlanningTime_s(row)=stage.timing.total_s;
                end
                timerEval=tic;
                [~,det]=cmFinal.evaluatePath(path,0,true);
                EvaluationTime_s(row)=toc(timerEval);
                if ~det.numerically_valid
                    RunStatus{row}=det.evaluation_status;
                    continue;
                end
                J(row)=localField(det,'J_final',NaN);
                Energy_Wh(row)=localField(det,'E_total',NaN);
                FlightTime_s(row)=localField(det,'T_total',NaN);
                ClimbCost(row)=localField(det,'C_climb',NaN);
                DynamicRisk(row)=localField(det,'R_dynamic',NaN);
                PenaltyTotal(row)=localField(det,'penalty_total',NaN);
                Feasible(row)=logical(localField(det,'feasible',false));
                RunStatus{row}='ok';
            catch ME
                PlanningTime_s(row)=toc(timer);
                RunStatus{row}=['failed:',localExceptionId(ME)];
                warning('RA-ALA config %d, environment %d failed: %s', ...
                    ci,ei,ME.message);
            end
        end

        completedEnv(ei)=true;
        checkpointEnvSeeds=envSeeds; %#ok<NASGU>
        checkpointConfigTable=configTable; %#ok<NASGU>
        save(checkpointFile,'checkpointEnvSeeds','checkpointConfigTable', ...
            'checkpointDesign','ConfigID','ConfigName','VariedComponent', ...
            'Multiplier','EnvironmentID','EnvironmentSeed','AlgorithmSeed', ...
            'J','Energy_Wh','FlightTime_s','ClimbCost','DynamicRisk', ...
            'PenaltyTotal','Feasible','PlanningTime_s','EvaluationTime_s', ...
            'RunStatus','completedEnv');
        fprintf('Completed environment %d/%d. Checkpoint saved.\n',ei,nEnv);
    end

    caseResults=table(ConfigID,ConfigName,VariedComponent,Multiplier, ...
        EnvironmentID,EnvironmentSeed,AlgorithmSeed,J,Energy_Wh,FlightTime_s, ...
        ClimbCost,DynamicRisk,PenaltyTotal,Feasible,PlanningTime_s, ...
        EvaluationTime_s,RunStatus);
    summaryResults=localSummarize(caseResults,configTable,nEnv);
    stabilityResults=localStability(summaryResults);

    writetable(caseResults,fullfile(opts.OutputDir, ...
        'ra_weight_sensitivity_case_results.csv'));
    writetable(summaryResults,fullfile(opts.OutputDir, ...
        'ra_weight_sensitivity_summary.csv'));
    writetable(stabilityResults,fullfile(opts.OutputDir, ...
        'ra_weight_sensitivity_stability.csv'));

    baseRows=caseResults.ConfigID==1;
    expectedBaseJ=S.stat_J(1,baseStatIndex)';
    actualBaseJ=caseResults.J(baseRows);
    valid=isfinite(expectedBaseJ) & isfinite(actualBaseJ);
    if any(valid)
        maxBaseDifference=max(abs(expectedBaseJ(valid)-actualBaseJ(valid)));
    else
        maxBaseDifference=NaN;
    end
    reproductionPassed=all(valid) && maxBaseDifference<=1e-9;
    if ~reproductionPassed
        warning('WeightSensitivity:BaseReproduction', ...
            'Base configuration differs from the cohort (max |dJ| = %.3g).', ...
            maxBaseDifference);
    end

    if opts.MakeFigure
        localPlotSensitivity(stabilityResults,fullfile(opts.OutputDir, ...
            'fig_ra_weight_sensitivity.png'));
    end
    localWriteReport(fullfile(opts.OutputDir, ...
        'ra_weight_sensitivity_method_report.txt'),dataFile,sampling,nEnv, ...
        nConfig,reproductionPassed,maxBaseDifference);
    save(fullfile(opts.OutputDir,'ra_weight_sensitivity_results.mat'), ...
        'caseResults','summaryResults','stabilityResults','configTable', ...
        'weightConfigs','envSeeds','raSeeds','cfg','sampling','provenance', ...
        'reproductionPassed','maxBaseDifference','checkpointDesign');

    outputs=struct('outputDir',opts.OutputDir,'caseResults',caseResults, ...
        'summaryResults',summaryResults,'stabilityResults',stabilityResults, ...
        'reproductionPassed',reproductionPassed, ...
        'maxBaseDifference',maxBaseDifference,'checkpointFile',checkpointFile);
    fprintf('\nRA-ALA weight sensitivity complete: %s\n',opts.OutputDir);
end

function [configTable,configs]=localBuildWeightConfigs()
    base=struct('w_energy',1.0,'w_time',0.5,'w_climb',2.0, ...
        'w_risk',10.0,'lambda_penalty',100.0);
    names={'Base','Energy 0.5x','Energy 1.5x','Time 0.5x','Time 1.5x', ...
        'Climb 0.5x','Climb 1.5x','Risk 0.5x','Risk 1.5x', ...
        'Penalty 0.5x','Penalty 1.5x'};
    components={'None','Energy','Energy','Time','Time','Climb','Climb', ...
        'Risk','Risk','Penalty','Penalty'};
    multipliers=[1 0.5 1.5 0.5 1.5 0.5 1.5 0.5 1.5 0.5 1.5]';
    fields={'','w_energy','w_energy','w_time','w_time','w_climb', ...
        'w_climb','w_risk','w_risk','lambda_penalty','lambda_penalty'};
    configs=repmat(base,numel(names),1);
    for i=2:numel(names)
        configs(i).(fields{i})=base.(fields{i})*multipliers(i);
    end
    configTable=table((1:numel(names))',names',components',multipliers, ...
        [configs.w_energy]',[configs.w_time]',[configs.w_climb]', ...
        [configs.w_risk]',[configs.lambda_penalty]', ...
        'VariableNames',{'ConfigID','ConfigName','VariedComponent', ...
        'Multiplier','w_energy','w_time','w_climb','w_risk','lambda_penalty'});
end

function cm=localCreateCostModel(env,w,spacing,minSamples)
    constructorWeights=struct('w_energy',w.w_energy,'w_time',w.w_time, ...
        'w_climb',w.w_climb,'w_risk',w.w_risk);
    cm=UnifiedCostModel([],constructorWeights);
    cm.lambda_penalty=w.lambda_penalty;
    cm.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
    cm.setCollisionSampling(spacing,minSamples);
end

function T=localSummarize(R,C,nEnv)
    n=height(C);
    SuccessfulPaths=zeros(n,1);
    FeasibleCount=zeros(n,1);
    FeasibilityPct=zeros(n,1);
    MedianCompositeScore=nan(n,1);
    MedianEnergyWh=nan(n,1);
    MedianFlightTimeS=nan(n,1);
    MedianDynamicRisk=nan(n,1);
    MedianPenaltyAllValid=nan(n,1);
    MedianPlanningTimeS=nan(n,1);
    for ci=1:n
        mask=R.ConfigID==ci;
        valid=mask & strcmp(R.RunStatus,'ok') & isfinite(R.J);
        feasibleValid=valid & R.Feasible;
        SuccessfulPaths(ci)=sum(valid);
        FeasibleCount(ci)=sum(feasibleValid);
        FeasibilityPct(ci)=100*FeasibleCount(ci)/nEnv;
        MedianCompositeScore(ci)=localMedian(R.J(feasibleValid));
        MedianEnergyWh(ci)=localMedian(R.Energy_Wh(feasibleValid));
        MedianFlightTimeS(ci)=localMedian(R.FlightTime_s(feasibleValid));
        MedianDynamicRisk(ci)=localMedian(R.DynamicRisk(feasibleValid));
        MedianPenaltyAllValid(ci)=localMedian(R.PenaltyTotal(valid));
        MedianPlanningTimeS(ci)=localMedian(R.PlanningTime_s(valid));
    end
    T=table(C.ConfigID,C.ConfigName,C.VariedComponent,C.Multiplier, ...
        SuccessfulPaths,FeasibleCount,FeasibilityPct,MedianCompositeScore, ...
        MedianEnergyWh,MedianFlightTimeS,MedianDynamicRisk, ...
        MedianPenaltyAllValid,MedianPlanningTimeS, ...
        'VariableNames',{'ConfigID','ConfigName','VariedComponent','Multiplier', ...
        'SuccessfulPaths','FeasibleCount','FeasibilityPct', ...
        'MedianCompositeScore','MedianEnergyWh','MedianFlightTimeS', ...
        'MedianDynamicRisk','MedianPenaltyAllValid','MedianPlanningTimeS'});
end

function T=localStability(S)
    n=height(S);
    DeltaFeasibilityPct=S.FeasibilityPct-S.FeasibilityPct(1);
    EnergyRatioToBase=localRatios(S.MedianEnergyWh,S.MedianEnergyWh(1));
    TimeRatioToBase=localRatios(S.MedianFlightTimeS,S.MedianFlightTimeS(1));
    RiskRatioToBase=localRatios(S.MedianDynamicRisk,S.MedianDynamicRisk(1));
    T=table(S.ConfigID,S.ConfigName,S.VariedComponent,S.Multiplier, ...
        S.FeasibilityPct,DeltaFeasibilityPct,S.MedianEnergyWh, ...
        EnergyRatioToBase,S.MedianFlightTimeS,TimeRatioToBase, ...
        S.MedianDynamicRisk,RiskRatioToBase,S.MedianPenaltyAllValid, ...
        'VariableNames',{'ConfigID','ConfigName','VariedComponent','Multiplier', ...
        'FeasibilityPct','DeltaFeasibilityPct','MedianEnergyWh', ...
        'EnergyRatioToBase','MedianFlightTimeS','TimeRatioToBase', ...
        'MedianDynamicRisk','RiskRatioToBase','MedianPenaltyAllValid'});
end

function r=localRatios(x,base)
    r=nan(size(x));
    if isfinite(base) && abs(base)>eps, r=x/base; end
end

function localPlotSensitivity(T,file)
    fig=figure('Color','w','Units','centimeters','Position',[2 2 30 20], ...
        'ToolBar','none','MenuBar','none');
    tl=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
    x=1:height(T);
    ax1=nexttile(tl);
    bar(ax1,x,T.FeasibilityPct,0.7,'FaceColor',[0.75 0.18 0.18]);
    ylim(ax1,[0 105]); ylabel(ax1,'Feasibility (%)');
    title(ax1,'RA-ALA Feasibility across Weight Configurations');
    grid(ax1,'on');
    ax2=nexttile(tl); hold(ax2,'on');
    plot(ax2,x,T.EnergyRatioToBase,'-o','LineWidth',1.6);
    plot(ax2,x,T.TimeRatioToBase,'-s','LineWidth',1.6);
    plot(ax2,x,T.RiskRatioToBase,'-^','LineWidth',1.6);
    yline(ax2,1,'--k','Base');
    ylabel(ax2,'Ratio to base configuration');
    legend(ax2,{'Energy','Flight time','Dynamic risk'}, ...
        'Location','best','Box','off');
    grid(ax2,'on');
    for ax=[ax1 ax2]
        set(ax,'XTick',x,'XTickLabel',T.ConfigName, ...
            'TickLabelInterpreter','none','FontName','Times New Roman', ...
            'FontSize',11,'LineWidth',1,'TickDir','out');
        xtickangle(ax,30);
    end
    exportPublicationFigure(fig,file);
    close(fig);
end

function localWriteReport(file,dataFile,sampling,nEnv,nConfig,repro,maxDiff)
    fid=fopen(file,'w');
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'RA-ALA-ONLY WEIGHT SENSITIVITY\n\n');
    fprintf(fid,'Source cohort: %s\n',dataFile);
    fprintf(fid,'Design: %d environments x %d one-at-a-time configurations.\n', ...
        nEnv,nConfig);
    fprintf(fid,'Planning/final spacing: %.3g/%.3g m.\n', ...
        sampling.PlanningSpacingM,sampling.FinalSpacingM);
    fprintf(fid,['Interpretation: within-method calibration under the prespecified ', ...
        'RA-ALA budget; no cross-planner ranking is performed.\n']);
    fprintf(fid,['Composite scores under different weight definitions are not ', ...
        'treated as directly comparable outcomes.\n']);
    fprintf(fid,['Physical metrics are summarized among feasible outputs; feasibility ', ...
        'retains all attempted environments in its denominator.\n']);
    fprintf(fid,'Base-cohort reproduction: %d; max |dJ| = %.6g.\n',repro,maxDiff);
end

function value=localField(S,name,defaultValue)
    if isstruct(S) && isfield(S,name) && ~isempty(S.(name))
        value=S.(name);
    else
        value=defaultValue;
    end
end

function value=localMedian(x)
    x=x(isfinite(x));
    if isempty(x), value=NaN; else, value=median(x); end
end

function id=localExceptionId(ME)
    id=ME.identifier;
    if isempty(id), id='unidentified_error'; end
end
