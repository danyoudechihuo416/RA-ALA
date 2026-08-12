function outputs = runWeightSensitivityAnalysis(dataFile)
%RUNWEIGHTSENSITIVITYANALYSIS One-at-a-time sensitivity of reported cost weights.
%
% The analysis reuses the ten Section 5.5 High-complexity environments and
% the first pre-specified algorithm seed from each environment. RA-ALA is
% re-optimized for every weight setting. The three baseline paths are
% generated once per environment and re-evaluated under every setting,
% because their implemented path-generation objectives do not use the
% reported composite-cost weights.

    projectDir = fileparts(mfilename('fullpath'));
    if nargin < 1 || isempty(dataFile)
        dataFile = fullfile(projectDir, 'section55_same_cohort_data.mat');
    elseif ~isfile(dataFile)
        dataFile = fullfile(projectDir, dataFile);
    end

    if ~isfile(dataFile)
        error(['Section 5.5 cohort data were not found. Run RA_ALA_demo.m ', ...
            'through Section 5.5 first so that section55_same_cohort_data.mat is created.']);
    end

    S = load(dataFile);
    required = {'env_seeds_used','stat_env','stat_ra_seed','stat_J', ...
        'ala_cfg_stat','mapSize','gridStep','windLevel','riskLevel', ...
        'startPt','goalPt','main_collision_sample_spacing_m'};
    for k = 1:numel(required)
        if ~isfield(S, required{k})
            error('Missing variable "%s" in %s.', required{k}, dataFile);
        end
    end

    probeCostModel = UnifiedCostModel();
    canonicalSpacingM = probeCostModel.collision_sample_spacing;
    if abs(double(S.main_collision_sample_spacing_m)-canonicalSpacingM) >= eps
        error('WeightSensitivity:ResolutionMismatch', ...
            ['The saved Section 5.5 cohort used %.3g m sampling, but the ', ...
            'current canonical resolution is %.3g m. Rerun RA_ALA_demo first.'], ...
            S.main_collision_sample_spacing_m,canonicalSpacingM);
    end

    envSeeds = S.env_seeds_used(:)';
    nEnv = numel(envSeeds);
    if nEnv ~= 10
        warning('Expected 10 Section 5.5 environments, but found %d.', nEnv);
    end

    raSeeds = nan(1, nEnv);
    baseStatIndex = nan(1, nEnv);
    for ei = 1:nEnv
        idx = find(S.stat_env == envSeeds(ei), 1, 'first');
        if isempty(idx)
            error('No Section 5.5 run seed was found for environment seed %d.', envSeeds(ei));
        end
        baseStatIndex(ei) = idx;
        raSeeds(ei) = S.stat_ra_seed(idx);
    end

    cfg = S.ala_cfg_stat;
    tStart = 0;
    hasPayload = true;
    algNames = {'RA-ALA','Energy-A*','Informed-RRT*','ST-EA*','Greedy'};
    nAlg = numel(algNames);

    [configTable, weightConfigs] = localBuildWeightConfigs();
    nConfig = height(configTable);

    outputDir = fullfile(projectDir, 'weight_sensitivity_results');
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end
    writetable(configTable, fullfile(outputDir, 'weight_sensitivity_configurations.csv'));

    nRows = nConfig * nEnv * nAlg;
    configId = zeros(nRows,1);
    configName = cell(nRows,1);
    variedComponent = cell(nRows,1);
    multiplier = nan(nRows,1);
    environmentId = zeros(nRows,1);
    environmentSeed = zeros(nRows,1);
    algorithmName = cell(nRows,1);
    algorithmSeed = nan(nRows,1);
    J = nan(nRows,1);
    energyWh = nan(nRows,1);
    flightTimeS = nan(nRows,1);
    climbCost = nan(nRows,1);
    dynamicRisk = nan(nRows,1);
    penaltyTotal = nan(nRows,1);
    feasible = false(nRows,1);
    planningTimeS = nan(nRows,1);
    evaluationTimeS = nan(nRows,1);
    pathReusedAcrossWeights = false(nRows,1);
    runStatus = repmat({'not_run'}, nRows, 1);
    completedEnv = false(nEnv,1);

    checkpointFile = fullfile(outputDir, 'weight_sensitivity_checkpoint.mat');
    if isfile(checkpointFile)
        C = load(checkpointFile);
        sameDesign = isfield(C,'checkpointEnvSeeds') && ...
            isequal(C.checkpointEnvSeeds(:)', envSeeds) && ...
            isfield(C,'checkpointConfigTable') && ...
            isequaln(C.checkpointConfigTable, configTable) && ...
            isfield(C,'checkpointSpacingM') && ...
            isequal(C.checkpointSpacingM,canonicalSpacingM);
        if sameDesign
            configId = C.configId;
            configName = C.configName;
            variedComponent = C.variedComponent;
            multiplier = C.multiplier;
            environmentId = C.environmentId;
            environmentSeed = C.environmentSeed;
            algorithmName = C.algorithmName;
            algorithmSeed = C.algorithmSeed;
            J = C.J;
            energyWh = C.energyWh;
            flightTimeS = C.flightTimeS;
            climbCost = C.climbCost;
            dynamicRisk = C.dynamicRisk;
            penaltyTotal = C.penaltyTotal;
            feasible = C.feasible;
            planningTimeS = C.planningTimeS;
            evaluationTimeS = C.evaluationTimeS;
            pathReusedAcrossWeights = C.pathReusedAcrossWeights;
            runStatus = C.runStatus;
            completedEnv = C.completedEnv;
            fprintf('Resuming weight sensitivity analysis: %d/%d environments completed.\n', ...
                sum(completedEnv), nEnv);
        else
            warning('An incompatible sensitivity checkpoint was ignored.');
        end
    end

    fprintf('\n=== Cost-weight sensitivity analysis ===\n');
    fprintf('Design: %d fixed High-complexity environments x %d weight settings.\n', ...
        nEnv, nConfig);
    fprintf('RA-ALA is re-optimized; baseline paths are generated once per environment.\n');

    for ei = 1:nEnv
        if completedEnv(ei)
            continue;
        end

        envSeed = envSeeds(ei);
        fprintf('\nEnvironment %d/%d (seed=%d)\n', ei, nEnv, envSeed);
        rng(envSeed);
        env = CityEnvironment(S.mapSize, S.gridStep);
        env.generate('high', S.windLevel, S.riskLevel, envSeed);
        env.setTaskPoints(S.startPt, S.goalPt);

        % Only baselines whose search objective is independent of the reported
        % J weights are generated once. ST-EA* is replanned for every setting.
        cmBase = localCreateCostModel(env, weightConfigs(1));
        plannerBase = PathPlanners(env, cmBase);
        plannerBase.setBudget(15, 5000, 2000);
        baselinePaths = cell(nAlg,1);
        baselinePlanningTime = nan(nAlg,1);
        baselineStatus = repmat({'not_applicable'}, nAlg, 1);

        for ai = [2 3 5]
            algSeed = envSeed + 53 + ai * 11;
            rng(algSeed);
            timer = tic;
            try
                switch ai
                    case 2
                        [baselinePaths{ai},~,~] = plannerBase.energyAStar( ...
                            S.startPt, S.goalPt, tStart, hasPayload);
                    case 3
                        [baselinePaths{ai},~,~] = plannerBase.informedRRTStar( ...
                            S.startPt, S.goalPt, tStart, hasPayload, 1500);
                    case 5
                        [baselinePaths{ai},~,~] = plannerBase.greedyPlanner( ...
                            S.startPt, S.goalPt, tStart, hasPayload);
                end
                baselinePlanningTime(ai) = toc(timer);
                if isempty(baselinePaths{ai})
                    baselineStatus{ai} = 'empty_path';
                else
                    baselineStatus{ai} = 'ok';
                end
            catch ME
                baselinePlanningTime(ai) = toc(timer);
                baselineStatus{ai} = ['failed: ', ME.identifier];
                baselinePaths{ai} = [];
            end
        end

        for ci = 1:nConfig
            cm = localCreateCostModel(env, weightConfigs(ci));
            planner = PathPlanners(env, cm);
            planner.setBudget(15, 5000, 2000);

            for ai = 1:nAlg
                row = ((ci-1)*nEnv + (ei-1))*nAlg + ai;
                configId(row) = ci;
                configName{row} = configTable.ConfigName{ci};
                variedComponent{row} = configTable.VariedComponent{ci};
                multiplier(row) = configTable.Multiplier(ci);
                environmentId(row) = ei;
                environmentSeed(row) = envSeed;
                algorithmName{row} = algNames{ai};

                try
                    if ai == 1
                        algorithmSeed(row) = raSeeds(ei);
                        rng(raSeeds(ei));
                        timer = tic;
                        capturedText = evalc(['[pathRA,~,det,stage] = runRA_ALA(', ...
                            'planner,cm,env,S.startPt,S.goalPt,tStart,hasPayload,cfg);']); %#ok<NASGU>
                        planningTimeS(row) = toc(timer);
                        if isempty(pathRA)
                            error('WeightSensitivity:EmptyRAPath', 'RA-ALA returned an empty path.');
                        end
                        if isfield(stage,'timing') && isfield(stage.timing,'total_s')
                            planningTimeS(row) = stage.timing.total_s;
                        end
                        evaluationTimeS(row) = localNestedField(stage, ...
                            {'timing','topk_evaluation_s'}, NaN);
                        pathReusedAcrossWeights(row) = false;
                    elseif ai == 4
                        algorithmSeed(row) = envSeed + 53 + ai * 11;
                        rng(algorithmSeed(row));
                        timer = tic;
                        [pathST,~,infoST] = planner.timeExpandedEnergyAStar( ...
                            S.startPt,S.goalPt,tStart,hasPayload,2,300);
                        planningTimeS(row) = toc(timer);
                        if isempty(pathST) || ~infoST.reachedGoal
                            error('WeightSensitivity:EmptySTPath', ...
                                'ST-EA* did not reach the goal (%s).',infoST.stopReason);
                        end
                        det = infoST.details;
                        evaluationTimeS(row) = NaN;
                        pathReusedAcrossWeights(row) = false;
                    else
                        algorithmSeed(row) = envSeed + 53 + ai * 11;
                        if ~strcmp(baselineStatus{ai}, 'ok')
                            error('WeightSensitivity:BaselineFailure', '%s', baselineStatus{ai});
                        end
                        timer = tic;
                        [~,det] = cm.evaluatePath(baselinePaths{ai}, tStart, hasPayload);
                        evaluationTimeS(row) = toc(timer);
                        planningTimeS(row) = baselinePlanningTime(ai);
                        pathReusedAcrossWeights(row) = true;
                    end

                    J(row) = localField(det, 'J_final', NaN);
                    energyWh(row) = localField(det, 'E_total', NaN);
                    flightTimeS(row) = localField(det, 'T_total', NaN);
                    climbCost(row) = localField(det, 'C_climb', NaN);
                    dynamicRisk(row) = localField(det, 'R_dynamic', NaN);
                    penaltyTotal(row) = localField(det, 'penalty_total', NaN);
                    feasible(row) = logical(localField(det, 'feasible', ...
                        penaltyTotal(row) < 0.1));
                    runStatus{row} = 'ok';
                catch ME
                    feasible(row) = false;
                    runStatus{row} = ['failed: ', ME.identifier];
                    warning('Config %d, environment %d, algorithm %s failed: %s', ...
                        ci, ei, algNames{ai}, ME.message);
                end
            end
        end

        completedEnv(ei) = true;
        checkpointEnvSeeds = envSeeds;
        checkpointConfigTable = configTable;
        checkpointSpacingM = canonicalSpacingM;
        save(checkpointFile, 'checkpointEnvSeeds','checkpointConfigTable', ...
            'checkpointSpacingM', ...
            'configId','configName','variedComponent','multiplier', ...
            'environmentId','environmentSeed','algorithmName','algorithmSeed', ...
            'J','energyWh','flightTimeS','climbCost','dynamicRisk', ...
            'penaltyTotal','feasible','planningTimeS','evaluationTimeS', ...
            'pathReusedAcrossWeights','runStatus','completedEnv');
        fprintf('Completed environment %d/%d. Checkpoint saved.\n', ei, nEnv);
    end

    caseResults = table(configId,configName,variedComponent,multiplier, ...
        environmentId,environmentSeed,algorithmName,algorithmSeed,J,energyWh, ...
        flightTimeS,climbCost,dynamicRisk,penaltyTotal,feasible,planningTimeS, ...
        evaluationTimeS,pathReusedAcrossWeights,runStatus, ...
        'VariableNames', {'ConfigID','ConfigName','VariedComponent','Multiplier', ...
        'EnvironmentID','EnvironmentSeed','Algorithm','AlgorithmSeed','J', ...
        'Energy_Wh','FlightTime_s','ClimbCost','DynamicRisk','PenaltyTotal', ...
        'Feasible','PlanningTime_s','EvaluationTime_s','PathReusedAcrossWeights', ...
        'RunStatus'});
    writetable(caseResults, fullfile(outputDir, 'weight_sensitivity_case_results.csv'));

    summaryResults = localSummarize(caseResults, configTable, algNames, nEnv);
    writetable(summaryResults, fullfile(outputDir, 'weight_sensitivity_summary.csv'));

    [rankingResults, raBestCount] = localRankSettings(summaryResults, configTable);
    writetable(rankingResults, fullfile(outputDir, 'weight_sensitivity_rank_stability.csv'));

    baseRows = find(caseResults.ConfigID == 1 & strcmp(caseResults.Algorithm,'RA-ALA'));
    expectedBaseJ = S.stat_J(1, baseStatIndex)';
    actualBaseJ = caseResults.J(baseRows);
    validRepro = isfinite(expectedBaseJ) & isfinite(actualBaseJ);
    if any(validRepro)
        maxBaseDifference = max(abs(expectedBaseJ(validRepro) - actualBaseJ(validRepro)));
    else
        maxBaseDifference = NaN;
    end
    reproductionPassed = all(isnan(expectedBaseJ) == isnan(actualBaseJ)) && ...
        (isnan(maxBaseDifference) || maxBaseDifference <= 1e-9);
    if ~reproductionPassed
        warning('Baseline sensitivity runs did not exactly reproduce the selected Section 5.5 cases (max |dJ| = %.3g).', ...
            maxBaseDifference);
    end

    figureFilePng = fullfile(outputDir, 'fig_weight_sensitivity.png');

    localPlotSensitivity(summaryResults, configTable, algNames, figureFilePng);

    reportFile = fullfile(outputDir, 'weight_sensitivity_method_report.txt');
    localWriteReport(reportFile, dataFile, configTable, nEnv, cfg, ...
        reproductionPassed, maxBaseDifference, raBestCount, nConfig);

    save(fullfile(outputDir, 'weight_sensitivity_results.mat'), ...
        'caseResults','summaryResults','rankingResults','configTable', ...
        'weightConfigs','envSeeds','raSeeds','cfg','reproductionPassed', ...
        'maxBaseDifference','raBestCount');

    if isfile(checkpointFile)
        delete(checkpointFile);
    end

    outputs = struct();
    outputs.outputDir = outputDir;
    outputs.caseResults = caseResults;
    outputs.summaryResults = summaryResults;
    outputs.rankingResults = rankingResults;
    outputs.reproductionPassed = reproductionPassed;
    outputs.maxBaseDifference = maxBaseDifference;
    outputs.raBestCount = raBestCount;

    fprintf('\nWeight sensitivity analysis completed.\n');
    fprintf('Results: %s\n', outputDir);
    fprintf('Section 5.5 reproduction check: %d (max |dJ| = %.3g).\n', ...
        reproductionPassed, maxBaseDifference);
    fprintf('RA-ALA feasibility-first best settings: %d/%d.\n', raBestCount, nConfig);
end

function [configTable, configs] = localBuildWeightConfigs()
    base = struct('w_energy',1.0,'w_time',0.5,'w_climb',2.0, ...
        'w_risk',10.0,'lambda_penalty',100.0);
    configNames = {'Base','Energy 0.5x','Energy 1.5x','Time 0.5x','Time 1.5x', ...
        'Climb 0.5x','Climb 1.5x','Risk 0.5x','Risk 1.5x', ...
        'Penalty 0.5x','Penalty 1.5x'};
    components = {'None','Energy','Energy','Time','Time','Climb','Climb', ...
        'Risk','Risk','Penalty','Penalty'};
    multipliers = [1,0.5,1.5,0.5,1.5,0.5,1.5,0.5,1.5,0.5,1.5]';
    configs = repmat(base, numel(configNames), 1);
    fieldNames = {'','w_energy','w_energy','w_time','w_time','w_climb', ...
        'w_climb','w_risk','w_risk','lambda_penalty','lambda_penalty'};
    for ci = 2:numel(configNames)
        f = fieldNames{ci};
        configs(ci).(f) = base.(f) * multipliers(ci);
    end
    configTable = table((1:numel(configNames))', configNames', components', ...
        multipliers, [configs.w_energy]', [configs.w_time]', [configs.w_climb]', ...
        [configs.w_risk]', [configs.lambda_penalty]', ...
        'VariableNames', {'ConfigID','ConfigName','VariedComponent','Multiplier', ...
        'w_energy','w_time','w_climb','w_risk','lambda_penalty'});
end

function cm = localCreateCostModel(env, weights)
    constructorWeights = struct('w_energy',weights.w_energy, ...
        'w_time',weights.w_time,'w_climb',weights.w_climb, ...
        'w_risk',weights.w_risk);
    cm = UnifiedCostModel([], constructorWeights);
    cm.lambda_penalty = weights.lambda_penalty;
    cm.setEnvironment(env.windField, env.dynObstacles, env.heightMap);
end

function value = localField(S, name, defaultValue)
    if isstruct(S) && isfield(S, name) && ~isempty(S.(name))
        value = S.(name);
    else
        value = defaultValue;
    end
end

function value = localNestedField(S, names, defaultValue)
    value = defaultValue;
    current = S;
    for k = 1:numel(names)
        if ~isstruct(current) || ~isfield(current, names{k})
            return;
        end
        current = current.(names{k});
    end
    if ~isempty(current)
        value = current;
    end
end

function summary = localSummarize(results, configTable, algNames, nEnv)
    nConfig = height(configTable);
    nAlg = numel(algNames);
    nRows = nConfig * nAlg;
    ConfigID = zeros(nRows,1);
    ConfigName = cell(nRows,1);
    VariedComponent = cell(nRows,1);
    Multiplier = nan(nRows,1);
    Algorithm = cell(nRows,1);
    SuccessfulPaths = zeros(nRows,1);
    FeasibleCount = zeros(nRows,1);
    FeasibilityPct = zeros(nRows,1);
    MedianJ = nan(nRows,1);
    Q1J = nan(nRows,1);
    Q3J = nan(nRows,1);
    MedianEnergyWh = nan(nRows,1);
    MedianFlightTimeS = nan(nRows,1);
    MedianDynamicRisk = nan(nRows,1);
    MedianPenalty = nan(nRows,1);
    MedianPlanningTimeS = nan(nRows,1);

    row = 0;
    for ci = 1:nConfig
        for ai = 1:nAlg
            row = row + 1;
            mask = results.ConfigID == ci & strcmp(results.Algorithm, algNames{ai});
            valid = mask & isfinite(results.J);
            ConfigID(row) = ci;
            ConfigName{row} = configTable.ConfigName{ci};
            VariedComponent{row} = configTable.VariedComponent{ci};
            Multiplier(row) = configTable.Multiplier(ci);
            Algorithm{row} = algNames{ai};
            SuccessfulPaths(row) = sum(valid);
            FeasibleCount(row) = sum(results.Feasible(mask));
            FeasibilityPct(row) = 100 * FeasibleCount(row) / nEnv;
            MedianJ(row) = localMedian(results.J(valid));
            Q1J(row) = localPercentile(results.J(valid), 25);
            Q3J(row) = localPercentile(results.J(valid), 75);
            MedianEnergyWh(row) = localMedian(results.Energy_Wh(valid));
            MedianFlightTimeS(row) = localMedian(results.FlightTime_s(valid));
            MedianDynamicRisk(row) = localMedian(results.DynamicRisk(valid));
            MedianPenalty(row) = localMedian(results.PenaltyTotal(valid));
            MedianPlanningTimeS(row) = localMedian(results.PlanningTime_s(valid));
        end
    end

    summary = table(ConfigID,ConfigName,VariedComponent,Multiplier,Algorithm, ...
        SuccessfulPaths,FeasibleCount,FeasibilityPct,MedianJ,Q1J,Q3J, ...
        MedianEnergyWh,MedianFlightTimeS,MedianDynamicRisk,MedianPenalty, ...
        MedianPlanningTimeS);
end

function [ranking, raBestCount] = localRankSettings(summary, configTable)
    nConfig = height(configTable);
    BestByMedianJ = cell(nConfig,1);
    BestFeasibilityFirst = cell(nConfig,1);
    RARankByMedianJ = nan(nConfig,1);
    RABestFeasibilityFirst = false(nConfig,1);

    for ci = 1:nConfig
        rows = find(summary.ConfigID == ci);
        medJ = summary.MedianJ(rows);
        feas = summary.FeasibilityPct(rows);
        medJForSort = medJ;
        medJForSort(~isfinite(medJForSort)) = inf;
        [~,orderJ] = sort(medJForSort, 'ascend');
        BestByMedianJ{ci} = summary.Algorithm{rows(orderJ(1))};
        raLocal = find(strcmp(summary.Algorithm(rows),'RA-ALA'),1);
        RARankByMedianJ(ci) = find(orderJ == raLocal, 1);

        bestFeas = max(feas);
        candidates = find(feas == bestFeas);
        [~,tieOrder] = sort(medJForSort(candidates), 'ascend');
        winner = candidates(tieOrder(1));
        BestFeasibilityFirst{ci} = summary.Algorithm{rows(winner)};
        RABestFeasibilityFirst(ci) = strcmp(BestFeasibilityFirst{ci}, 'RA-ALA');
    end

    ranking = table(configTable.ConfigID,configTable.ConfigName, ...
        BestByMedianJ,BestFeasibilityFirst,RARankByMedianJ,RABestFeasibilityFirst, ...
        'VariableNames', {'ConfigID','ConfigName','BestByMedianJ', ...
        'BestFeasibilityFirst','RA_RankByMedianJ','RA_BestFeasibilityFirst'});
    raBestCount = sum(RABestFeasibilityFirst);
end

function localPlotSensitivity(summary, configTable, algNames, pngFile)
    nConfig = height(configTable);
    nAlg = numel(algNames);
    medJ = nan(nConfig,nAlg);
    feas = nan(nConfig,nAlg);
    for ci = 1:nConfig
        for ai = 1:nAlg
            row = find(summary.ConfigID == ci & strcmp(summary.Algorithm,algNames{ai}),1);
            medJ(ci,ai) = summary.MedianJ(row);
            feas(ci,ai) = summary.FeasibilityPct(row);
        end
    end
    relativeJ = medJ ./ medJ(:,1);
    logRelativeJ = log10(max(relativeJ, eps));

    fig = figure('Color','w','Units','centimeters','Position',[2 2 32 23]);
    tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');

    ax1 = nexttile;
    imagesc(ax1,logRelativeJ);
    set(ax1,'XTick',1:nAlg,'XTickLabel',algNames,'YTick',1:nConfig, ...
        'YTickLabel',configTable.ConfigName,'FontName','Times New Roman','FontSize',10);
    title(ax1,'(a) Median composite cost relative to RA-ALA within each weight setting', ...
        'FontWeight','bold');
    cb1 = colorbar(ax1);
    cb1.Label.String = 'log_{10}(median J / median J_{RA-ALA})';
    for ci = 1:nConfig
        for ai = 1:nAlg
            text(ax1,ai,ci,sprintf('%.2fx',relativeJ(ci,ai)), ...
                'HorizontalAlignment','center','FontSize',9,'FontWeight','bold', ...
                'Color',localContrastColor(logRelativeJ(ci,ai),ax1.CLim));
        end
    end

    ax2 = nexttile;
    imagesc(ax2,feas,[0 100]);
    set(ax2,'XTick',1:nAlg,'XTickLabel',algNames,'YTick',1:nConfig, ...
        'YTickLabel',configTable.ConfigName,'FontName','Times New Roman','FontSize',10);
    title(ax2,'(b) Feasibility rate across the same ten environments', ...
        'FontWeight','bold');
    cb2 = colorbar(ax2);
    cb2.Label.String = 'Feasibility (%)';
    for ci = 1:nConfig
        for ai = 1:nAlg
            text(ax2,ai,ci,sprintf('%.0f%%',feas(ci,ai)), ...
                'HorizontalAlignment','center','FontSize',9,'FontWeight','bold', ...
                'Color',localContrastColor(feas(ci,ai),[0 100]));
        end
    end

    colormap(fig,parula(256));
    exportPublicationFigure(fig,pngFile);
    close(fig);
end

function color = localContrastColor(value, limits)
    midpoint = mean(limits);
    if isfinite(value) && value < midpoint
        color = [1 1 1];
    else
        color = [0 0 0];
    end
end

function localWriteReport(reportFile, dataFile, configTable, nEnv, cfg, ...
        reproductionPassed, maxBaseDifference, raBestCount, nConfig)
    fid = fopen(reportFile,'w');
    if fid < 0
        warning('Could not write %s.', reportFile);
        return;
    end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid,'COST-WEIGHT SENSITIVITY ANALYSIS\n');
    fprintf(fid,'================================\n\n');
    fprintf(fid,'Source cohort: %s\n', dataFile);
    fprintf(fid,'Design: %d fixed High-complexity environments, one pre-specified seed per environment.\n', nEnv);
    fprintf(fid,'Settings: baseline plus 0.5x and 1.5x one-at-a-time perturbations of each reported weight.\n');
    fprintf(fid,'Base weights: w_energy=1, w_time=0.5, w_climb=2, w_risk=10, lambda_penalty=100.\n');
    fprintf(fid,'RA-ALA budget: popSize=%d, maxIter=%d.\n', cfg.popSize, cfg.maxIter);
    fprintf(fid,'The auxiliary search riskWeight remains fixed at %.6g; it is not treated as a reported J weight.\n\n', cfg.riskWeight);
    fprintf(fid,'Implementation rule:\n');
    fprintf(fid,'- RA-ALA and ST-EA* are re-optimized under every weight setting.\n');
    fprintf(fid,'- Energy-A*, Informed-RRT*, and Greedy paths are generated once per environment and rescored,\n');
    fprintf(fid,'  because their implemented path-generation objectives do not use the reported composite-cost weights.\n');
    fprintf(fid,'- Raw J values are not compared across different settings because each setting defines a different objective.\n');
    fprintf(fid,'  Interpretation uses within-setting algorithm ranks, feasibility, and physical outcome components.\n\n');
    fprintf(fid,'Section 5.5 reproduction passed: %d\n', reproductionPassed);
    fprintf(fid,'Maximum absolute baseline J difference: %.12g\n', maxBaseDifference);
    fprintf(fid,'RA-ALA feasibility-first best settings: %d/%d\n\n', raBestCount, nConfig);
    fprintf(fid,'This analysis assesses robustness to reasonable weight perturbations; it does not claim that the base weights are optimal.\n\n');
    fprintf(fid,'Configuration table:\n');
    for ci = 1:height(configTable)
        fprintf(fid,'%2d %-14s  wE=%g wt=%g wc=%g wr=%g lambda=%g\n', ...
            configTable.ConfigID(ci),configTable.ConfigName{ci}, ...
            configTable.w_energy(ci),configTable.w_time(ci), ...
            configTable.w_climb(ci),configTable.w_risk(ci), ...
            configTable.lambda_penalty(ci));
    end
end

function value = localMedian(x)
    x = x(isfinite(x));
    if isempty(x)
        value = NaN;
    else
        value = median(x);
    end
end

function value = localPercentile(x, p)
    x = sort(x(isfinite(x)));
    n = numel(x);
    if n == 0
        value = NaN;
        return;
    end
    if n == 1
        value = x(1);
        return;
    end
    position = 1 + (n-1) * p / 100;
    lowerIndex = floor(position);
    upperIndex = ceil(position);
    fraction = position - lowerIndex;
    value = x(lowerIndex) + fraction * (x(upperIndex) - x(lowerIndex));
end
