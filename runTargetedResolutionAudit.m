function results = runTargetedResolutionAudit(resultsFile,userOpts)
%RUNTARGETEDRESOLUTIONAUDIT Re-evaluate selected fixed paths at 0.75 m.
%   By default, cases 1 and 18 are checked at 3, 1.5, and 0.75 m using
%   exactly the paths and environments saved by the resolution experiment.

    projectDir = fileparts(mfilename('fullpath'));
    if nargin < 1 || isempty(resultsFile)
        resultsFile = fullfile(projectDir,'spatial_resolution_output', ...
            'spatial_resolution_results.mat');
    elseif ~isfile(resultsFile)
        resultsFile = fullfile(projectDir,resultsFile);
    end
    if nargin < 2, userOpts = struct(); end
    opts = localDefaults(projectDir,userOpts);
    if ~exist(opts.OutputDir,'dir'), mkdir(opts.OutputDir); end

    L = load(resultsFile);
    required={'paths','envSeeds','algorithmSeeds','settings'};
    for i=1:numel(required)
        if ~isfield(L,required{i})
            error('TargetedResolution:MissingField', ...
                'Missing %s in %s.',required{i},resultsFile);
        end
    end

    nSeed=size(L.algorithmSeeds,2); nCase=numel(L.paths);
    caseIds=opts.CaseIDs(:)'; spacings=opts.SpacingsM(:)';
    nRows=numel(caseIds)*numel(spacings);
    case_id=zeros(nRows,1); environment_id=zeros(nRows,1);
    environment_seed=zeros(nRows,1); run_within_environment=zeros(nRows,1);
    algorithm_seed=zeros(nRows,1); spacing_m=zeros(nRows,1);
    J=nan(nRows,1); energy_Wh=nan(nRows,1); arrival_time_s=nan(nRows,1);
    dynamic_risk=nan(nRows,1); penalty_total=nan(nRows,1);
    penalty_height=nan(nRows,1); penalty_static=nan(nRows,1);
    penalty_dynamic=nan(nRows,1); penalty_nfz=nan(nRows,1);
    feasible=false(nRows,1); static_violation=false(nRows,1);
    dynamic_violation=false(nRows,1); nfz_violation=false(nRows,1);
    total_subsamples=nan(nRows,1); evaluation_time_s=nan(nRows,1);
    status=repmat({''},nRows,1); row=0;

    for ci=1:numel(caseIds)
        id=caseIds(ci);
        if id<1 || id>nCase || isempty(L.paths{id})
            error('TargetedResolution:InvalidCase','Case %d has no saved path.',id);
        end
        ei=ceil(id/nSeed); si=id-(ei-1)*nSeed;
        envSeed=L.envSeeds(ei); algSeed=L.algorithmSeeds(ei,si);
        rng(envSeed,'twister');
        env=CityEnvironment(L.settings.MapSize,L.settings.GridStep);
        env.generate('high',L.settings.WindLevel,L.settings.RiskLevel,envSeed);
        env.setTaskPoints(L.settings.Start,L.settings.Goal);
        path=L.paths{id};

        for ri=1:numel(spacings)
            row=row+1; case_id(row)=id; environment_id(row)=ei;
            environment_seed(row)=envSeed; run_within_environment(row)=si;
            algorithm_seed(row)=algSeed; spacing_m(row)=spacings(ri);
            try
                cm=UnifiedCostModel();
                cm.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
                cm.setCollisionSampling(spacings(ri),opts.MinSamples);
                timer=tic; [~,d]=cm.evaluatePath(path,0,true);
                evaluation_time_s(row)=toc(timer); J(row)=d.J_final;
                energy_Wh(row)=d.E_total; arrival_time_s(row)=d.T_total;
                dynamic_risk(row)=d.R_dynamic; penalty_total(row)=d.penalty_total;
                penalty_height(row)=d.penalty_height;
                penalty_static(row)=d.penalty_static_collision;
                penalty_dynamic(row)=d.penalty_dynamic_collision;
                penalty_nfz(row)=d.penalty_nfz; feasible(row)=logical(d.feasible);
                static_violation(row)=d.penalty_static_collision>0;
                dynamic_violation(row)=d.penalty_dynamic_collision>0;
                nfz_violation(row)=d.penalty_nfz>0;
                total_subsamples(row)=d.total_collision_subsamples;
                status{row}='ok';
            catch ME
                status{row}=['failed:',localExceptionId(ME)];
            end
        end
    end

    audit=table(case_id,environment_id,environment_seed,run_within_environment, ...
        algorithm_seed,spacing_m,J,energy_Wh,arrival_time_s,dynamic_risk, ...
        penalty_total,penalty_height,penalty_static,penalty_dynamic, ...
        penalty_nfz,feasible,static_violation,dynamic_violation,nfz_violation, ...
        total_subsamples,evaluation_time_s,status);
    decision=localDecision(audit,caseIds,opts);
    writetable(audit,fullfile(opts.OutputDir,'targeted_resolution_case_results.csv'));
    writetable(decision,fullfile(opts.OutputDir,'targeted_resolution_decision.csv'));
    localWriteReport(fullfile(opts.OutputDir,'targeted_resolution_report.txt'), ...
        resultsFile,opts,audit,decision);
    save(fullfile(opts.OutputDir,'targeted_resolution_results.mat'), ...
        'audit','decision','opts','caseIds','spacings');
    results=struct('audit',audit,'decision',decision,'options',opts);
    fprintf('Targeted resolution audit written to: %s\n',opts.OutputDir);
end

function opts=localDefaults(projectDir,u)
    opts=struct('CaseIDs',[1 18],'SpacingsM',[3 1.5 0.75], ...
        'MinSamples',3,'FeasibilityThreshold',0, ...
        'OutputDir',fullfile(projectDir,'targeted_resolution_output'));
    names=fieldnames(u);
    for i=1:numel(names), opts.(names{i})=u.(names{i}); end
    if ~any(abs(opts.SpacingsM-1.5)<eps) || ~any(abs(opts.SpacingsM-0.75)<eps)
        error('TargetedResolution:RequiredSpacings', ...
            'SpacingsM must include 1.5 and 0.75 m.');
    end
end

function D=localDecision(T,caseIds,opts)
    n=numel(caseIds); case_id=caseIds(:); agreement_1p5_vs_0p75=false(n,1);
    feasible_1p5=false(n,1); feasible_0p75=false(n,1);
    same_static_status=false(n,1); same_dynamic_status=false(n,1);
    same_height_status=false(n,1);
    same_nfz_status=false(n,1); relative_J_change=nan(n,1);
    recommendation=cell(n,1);
    for i=1:n
        a=T(T.case_id==caseIds(i) & abs(T.spacing_m-1.5)<eps,:);
        b=T(T.case_id==caseIds(i) & abs(T.spacing_m-0.75)<eps,:);
        if height(a)~=1 || height(b)~=1
            error('TargetedResolution:MissingComparison', ...
                'Case %d lacks a unique 1.5/0.75 m comparison.',caseIds(i));
        end
        feasible_1p5(i)=a.feasible; feasible_0p75(i)=b.feasible;
        same_height_status(i)=(a.penalty_height>0)==(b.penalty_height>0);
        same_static_status(i)=a.static_violation==b.static_violation;
        same_dynamic_status(i)=a.dynamic_violation==b.dynamic_violation;
        same_nfz_status(i)=a.nfz_violation==b.nfz_violation;
        agreement_1p5_vs_0p75(i)=feasible_1p5(i)==feasible_0p75(i) && ...
            same_height_status(i) && same_static_status(i) && ...
            same_dynamic_status(i) && same_nfz_status(i);
        relative_J_change(i)=abs(a.J-b.J)/max(abs(b.J),eps);
        if agreement_1p5_vs_0p75(i)
            recommendation{i}='1.5 m classification confirmed by 0.75 m';
        else
            recommendation{i}='sampling classification unresolved; use continuous checks';
        end
    end
    D=table(case_id,feasible_1p5,feasible_0p75,agreement_1p5_vs_0p75, ...
        same_height_status,same_static_status,same_dynamic_status, ...
        same_nfz_status,relative_J_change,recommendation);
    D.Properties.UserData=struct('all_confirmed',all(agreement_1p5_vs_0p75), ...
        'feasibility_threshold',opts.FeasibilityThreshold);
end

function localWriteReport(file,source,opts,T,D)
    fid=fopen(file,'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'TARGETED 0.75 M RESOLUTION AUDIT\n\nSource: %s\n',source);
    fprintf(fid,'Fixed cases: %s\nSpacings: %s m\n\n', ...
        mat2str(opts.CaseIDs),mat2str(opts.SpacingsM));
    for i=1:height(T)
        fprintf(fid,'case=%d spacing=%.3g m J=%.8g penalty=%.8g feasible=%d height/static/dynamic/NFZ=%d/%d/%d/%d subsamples=%g\n', ...
            T.case_id(i),T.spacing_m(i),T.J(i),T.penalty_total(i),T.feasible(i), ...
            T.penalty_height(i)>0,T.static_violation(i),T.dynamic_violation(i), ...
            T.nfz_violation(i),T.total_subsamples(i));
    end
    fprintf(fid,'\nDECISION\n');
    for i=1:height(D)
        fprintf(fid,'case=%d: 1.5 m feasible=%d, 0.75 m feasible=%d, confirmed=%d, relative J change=%.6g; %s\n', ...
            D.case_id(i),D.feasible_1p5(i),D.feasible_0p75(i), ...
            D.agreement_1p5_vs_0p75(i),D.relative_J_change(i),D.recommendation{i});
    end
    if all(D.agreement_1p5_vs_0p75)
        fprintf(fid,'\nOverall recommendation: adopt 1.5 m as the final sampled standard; targeted 0.75 m confirms the limiting classifications.\n');
    else
        fprintf(fid,'\nOverall recommendation: 1.5 m is not converged against 0.75 m; use 0.75 m for the main experiments.\n');
    end
end

function id=localExceptionId(ME)
    id=ME.identifier; if isempty(id), id='unidentified_error'; end
end
