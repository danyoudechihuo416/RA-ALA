function tests = testValidationInterfaces
%TESTVALIDATIONINTERFACES Small interface tests; no full experiments.
    tests = functiontests(localfunctions);
end

function testDistinctSamplingSettings(t)
    S = localCohort();
    settings = resolveCohortEvaluationSettings(S);
    verifyEqual(t,settings.PlanningSpacingM,0.75);
    verifyEqual(t,settings.FinalSpacingM,0.75);
    S.main_collision_sample_spacing_m = 1.5;
    verifyError(t,@()resolveCohortEvaluationSettings(S),'Validation:ResolutionMetadata');
end

function testHistoricalAndChangedSourceRejected(t)
    S = localCohort();
    legacy = rmfield(S,'planning_collision_sample_spacing_m');
    verifyError(t,@()resolveCohortEvaluationSettings(legacy),'Validation:CohortSchema');
    S.validation_source.sha256{1} = 'different source';
    verifyError(t,@()resolveCohortEvaluationSettings(S),'Validation:SourceMismatch');
end

function testProvenanceTracksArchive(t)
    folder = localFolder(t);
    file = fullfile(folder,'cohort.mat');
    S = localCohort(); save(file,'-struct','S');
    first = validationProvenance(file);
    verifyEqual(t,first,validationProvenance(file));
    S.stat_J(1,1)=99; save(file,'-struct','S');
    changed = validationProvenance(file);
    verifyNotEqual(t,first.cohort_sha256,changed.cohort_sha256);
    verifyEqual(t,first.sha256,changed.sha256);
end

function testWeightPreflightDoesNotPlanOrWrite(t)
    folder = localFolder(t);
    S = localCohort(); file=fullfile(folder,'cohort.mat'); save(file,'-struct','S');
    outputDir=fullfile(folder,'weight_output');
    opts=struct('OutputDir',outputDir,'MakeFigure',false,'CheckOnly',true);
    R=runWeightSensitivityAnalysis(file,opts);
    verifyTrue(t,R.preflight_passed);
    verifyEqual(t,R.environment_count,1);
    verifyEqual(t,R.configuration_count,11);
    verifyFalse(t,isfolder(outputDir));
end

function testFixedPathWeightPreflightDoesNotWrite(t)
    folder=localFolder(t);
    S=localCohort(); file=fullfile(folder,'cohort.mat'); save(file,'-struct','S');
    outputDir=fullfile(folder,'fixed_weight_output');
    R=runFixedPathWeightSensitivityAnalysis(file,struct( ...
        'OutputDir',outputDir,'MakeFigure',false,'CheckOnly',true));
    verifyTrue(t,R.preflight_passed);
    verifyEqual(t,R.configuration_count,11);
    verifyFalse(t,isfolder(outputDir));
end

function testFixedPathsAndFailedTrialsAreNotReplanned(t)
    folder=localFolder(t);
    S=localCohort(); file=fullfile(folder,'cohort.mat'); save(file,'-struct','S');
    opts=struct('OutputDir',folder,'N_ENV',1,'N_SEED',2, ...
        'SpacingsM',[6 3 1.5 0.75 0.375]);
    R=runSpatialResolutionSensitivity(file,opts);
    verifyEqual(t,R.paths{1},S.stat_paths{1,1});
    verifyEmpty(t,R.paths{2});
    verifyEqual(t,R.planning_time_s,[0;0]);
    verifyEqual(t,R.planning_status{1},'reused_saved_path');
    verifyEqual(t,R.planning_status{2},'failed:ResolutionSensitivity:SavedPlannerFailure');
    verifyEqual(t,R.summary.N,repmat(2,5,1));
    verifyEqual(t,R.summary.evaluated_N,ones(5,1));
    verifyTrue(t,all(R.summary.feasible_rate<=0.5));
    opts.PlanningSpacingM=3;
    verifyError(t,@()runSpatialResolutionSensitivity(file,opts), ...
        'ResolutionSensitivity:SavedPathResolutionMismatch');
end

function testSummaryIncludesWaitAndUniqueCounts(t)
    folder=localFolder(t);
    S=localCohort();
    S.stat_paths(1,:)={S.stat_paths{1,1},S.stat_paths{1,1}};
    S.stat_paths(4,:)=S.stat_paths(1,:);
    S.stat_wait_schedules(4,:)={2,2};
    file=fullfile(folder,'cohort.mat'); save(file,'-struct','S');
    R=summarizeExperimentOutcomes(file,folder);
    verifyEqual(t,R.counts.N,[2;1;2;1;1]);
    ra=R.caseTable(strcmp(R.caseTable.Algorithm,'RA-ALA') & R.caseTable.UniqueTrial,:);
    st=R.caseTable(strcmp(R.caseTable.Algorithm,'ST-EA*') & R.caseTable.UniqueTrial,:);
    rng(S.env_seeds_used,'twister');
    env=CityEnvironment(S.mapSize,S.gridStep);
    env.generate('high',S.windLevel,S.riskLevel,S.env_seeds_used);
    env.setTaskPoints(S.startPt,S.goalPt);
    cm=UnifiedCostModel(); cm.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
    cm.setCollisionSampling(0.75,3);
    [~,expected]=cm.evaluatePath(S.stat_paths{4,1},0,true,2);
    % Waiting also changes subsequent wind exposure, not just total time.
    verifyEqual(t,expected.total_wait_time_s,2);
    verifyEqual(t,st.TimeS,expected.T_total,'AbsTol',1e-9);
    verifyEqual(t,st.EnergyWh,expected.E_total,'AbsTol',1e-9);
    verifyGreaterThan(t,st.EnergyWh,ra.EnergyWh(1));
    verifyTrue(t,ismember('Kinematic',R.counts.Properties.VariableNames));
end

function testValidationSuitePreservesExperimentOrder(t)
    suite=fileread('runAllValidationExperiments.m');
    pre=strfind(suite,'Main pre-cohort stage:');
    figure78=strfind(suite,'Running/resuming the authoritative 0.75 m Figure 7-8 cohort');
    post=strfind(suite,'Main post-cohort stage:');
    verifyNotEmpty(t,pre);
    verifyNotEmpty(t,figure78);
    verifyNotEmpty(t,post);
    verifyLessThan(t,pre(1),figure78(1));
    verifyLessThan(t,figure78(1),post(1));
    main=fileread('runMainExperiments.m');
    verifyNotEmpty(t,strfind(main,'''pre-cohort''')); %#ok<STRIFCND>
    verifyNotEmpty(t,strfind(main,'''post-cohort''')); %#ok<STRIFCND>
end

function testSuitePreflightDoesNotExecuteOrChangeBaseMode(t)
    assignin('base','RA_ALA_RUN_MODE','replot-path-comparison');
    cleanup=onCleanup(@()evalin('base','clear RA_ALA_RUN_MODE')); %#ok<NASGU>
    R=runAllValidationExperiments(struct('CheckOnly',true));
    verifyTrue(t,R.preflight_passed);
    verifyFalse(t,R.main_completed);
    verifyEqual(t,evalin('base','RA_ALA_RUN_MODE'),'replot-path-comparison');
    verifyTrue(t,R.options.RunWeights);
    verifyTrue(t,R.options.RunSummary);
    verifyError(t,@()runAllValidationExperiments(struct('RunWieghts',false)), ...
        'ValidationSuite:UnknownOption');
end

function testRemainingSuiteSkipsCompletedStages(t)
    remaining=fileread('runRemainingValidationExperiments.m');
    verifyNotEmpty(t,strfind(remaining,'''RunFigure78'',false')); %#ok<STRIFCND>
    verifyNotEmpty(t,strfind(remaining,'''RunMain'',false')); %#ok<STRIFCND>
    verifyNotEmpty(t,strfind(remaining,'''RunClusterStats'',false')); %#ok<STRIFCND>
end

function testWeightSmokeAtAuthoritativeResolution(t)
    folder=localFolder(t);
    S=localCohort();
    S.stat_J(1,1)=NaN;
    file=fullfile(folder,'cohort.mat'); save(file,'-struct','S');
    R=runWeightSensitivityAnalysis(file,struct('OutputDir',folder, ...
        'MakeFigure',false,'Resume',false));
    verifyEqual(t,height(R.caseResults),11);
    verifyEqual(t,unique(R.caseResults.AlgorithmSeed),S.stat_ra_seed(1));
    verifyFalse(t,any(strcmp(R.caseResults.RunStatus,'not_run')));
    verifyTrue(t,isfile(R.checkpointFile));
    archive=load(fullfile(folder,'ra_weight_sensitivity_results.mat'));
    verifyEqual(t,archive.sampling.FinalSpacingM,0.75);
    verifyEqual(t,archive.sampling.PlanningSpacingM,0.75);
end

function testBudgetSmokeRecordsReleaseSpacing(t)
    folder=localFolder(t);
    opts=struct('OutputDir',folder,'N_ENV',1,'N_SEED',2, ...
        'Start',[80 80 60],'Goal',[90 80 60],'TimeBudgetS',0.05, ...
        'NodeBudget',2,'RRTIterations',2,'EnvironmentSeeds',483, ...
        'ALAConfig',struct('popSize',4,'maxIter',1,'nWaypoints',2, ...
        'riskWeight',15,'windLookahead',3,'rescue_max_ins',0));
    R=runComputationalBudgetAnalysis(opts);
    verifyEqual(t,R.summary.N,[2;1;2;1;1]);
    verifyFalse(t,any(R.raw.exception));
    verifyEqual(t,R.budget_permissions.planning_spacing_m,repmat(0.75,5,1));
    verifyEqual(t,R.budget_permissions.final_spacing_m,repmat(0.75,5,1));
    verifyTrue(t,all(R.raw.final_evaluatePath_calls<=1));
    verifyEqual(t,R.raw.total_with_verification_s, ...
        R.raw.wall_clock_s+R.raw.final_evaluation_s,'AbsTol',1e-9);
end

function folder=localFolder(t)
    fixture=t.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
    folder=fixture.Folder;
end

function S=localCohort()
    S=struct('planning_collision_sample_spacing_m',0.75, ...
        'final_verification_spacing_m',0.75,'main_collision_sample_spacing_m',0.75, ...
        'main_min_collision_samples',3,'mapSize',1000,'gridStep',10, ...
        'windLevel','medium','riskLevel','dense','startPt',[80 80 60], ...
        'goalPt',[90 80 60],'env_seeds_used',483,'stat_env',[483 483], ...
        'stat_ra_seed',[547 600]);
    S.validation_source=validationProvenance();
    S.ala_cfg_stat=struct('popSize',4,'maxIter',1,'nWaypoints',2, ...
        'riskWeight',15,'windLookahead',3,'rescue_max_ins',0);
    S.algNames={'RA-ALA','Energy-A*','Informed-RRT*','ST-EA*','Greedy'};
    S.stat_J=ones(5,2);
    S.stat_paths=cell(5,2);
    S.stat_final_evaluation_details=cell(5,2);
    S.stat_paths{1,1}=[S.startPt;S.goalPt];
    S.stat_wait_schedules=cell(5,2);
    S.stat_is_unique_trial=true(5,2);
    S.stat_is_unique_trial([2 4 5],2)=false;
end
