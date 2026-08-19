function results = runClusterAwareStatistics(dataFile,userOpts)
%RUNCLUSTERAWARESTATISTICS Environment-level inference for Section 5.5.
%   The three algorithmic seeds within each urban environment are first
%   aggregated. The ten environments, rather than the 30 runs, are then
%   treated as independent blocks for inference.

    projectDir = fileparts(mfilename('fullpath'));
    if nargin < 1 || isempty(dataFile)
        dataFile = fullfile(projectDir,'main_experiment_cohort.mat');
    elseif ~isfile(dataFile)
        dataFile = fullfile(projectDir,dataFile);
    end
    if nargin < 2, userOpts = struct(); end
    opts = localDefaults(projectDir,userOpts);
    if ~exist(opts.OutputDir,'dir'), mkdir(opts.OutputDir); end

    S = load(dataFile);
    required = {'stat_env','stat_J','stat_E','stat_P','algNames','N_SEED', ...
        'main_collision_sample_spacing_m'};
    for i=1:numel(required)
        if ~isfield(S,required{i})
            error('ClusterStats:MissingField','Missing %s in %s.',required{i},dataFile);
        end
    end
    probe=UnifiedCostModel();
    expectedSpacing=probe.collision_sample_spacing;
    if abs(double(S.main_collision_sample_spacing_m)-expectedSpacing)>=eps
        error('ClusterStats:StaleMainCohort', ...
            'Cohort spacing is %.3g m; rerun main experiments at %.3g m.', ...
            S.main_collision_sample_spacing_m,expectedSpacing);
    end


    algNames = S.algNames(:)';
    nAlg = numel(algNames);
    envSeeds = unique(S.stat_env(:)','stable');
    nEnv = numel(envSeeds);
    if nEnv < 5
        error('ClusterStats:TooFewEnvironments', ...
            'At least five independent environments are required.');
    end

    envJ = nan(nEnv,nAlg); envE = nan(nEnv,nAlg);
    envP = nan(nEnv,nAlg); envF = nan(nEnv,nAlg);
    envSuccess = zeros(nEnv,nAlg); envRuns = zeros(nEnv,nAlg);
    for e=1:nEnv
        idx = find(S.stat_env==envSeeds(e));
        for a=1:nAlg
            j = S.stat_J(a,idx); en = S.stat_E(a,idx); p = S.stat_P(a,idx);
            if isfield(S,'stat_feasible')
                feasible = logical(S.stat_feasible(a,idx));
            else
                feasible = p <= opts.PenaltyTolerance;
            end
            ok = isfinite(j) & isfinite(en) & isfinite(p);
            envRuns(e,a) = sum(ok);
            if any(ok)
                envJ(e,a) = median(j(ok));
                envE(e,a) = median(en(ok));
                envP(e,a) = median(p(ok));
                envSuccess(e,a) = sum(feasible(ok));
                envF(e,a) = envSuccess(e,a)/sum(ok);
            end
        end
    end

    rng(opts.BootstrapSeed,'twister');
    bootIdx = randi(nEnv,nEnv,opts.BootstrapReplicates);

    envTable = localEnvironmentTable(envSeeds,algNames,envJ,envE,envP, ...
        envF,envSuccess,envRuns);
    continuous = localContinuousTests(algNames,envJ,envE,envP,bootIdx,opts);
    feasibilityRates = localFeasibilityRates(algNames,envF,envSuccess, ...
        envRuns,bootIdx,opts);
    feasibilityPairs = localFeasibilityPairs(algNames,envF,bootIdx,opts);
    omnibus = localOmnibus(algNames,envJ,envE,envP,envF);

    writetable(envTable,fullfile(opts.OutputDir,'environment_level_summary.csv'));
    writetable(continuous,fullfile(opts.OutputDir,'continuous_pairwise_tests.csv'));
    writetable(feasibilityRates,fullfile(opts.OutputDir,'feasibility_rates_cluster_ci.csv'));
    writetable(feasibilityPairs,fullfile(opts.OutputDir,'feasibility_pairwise_cluster_tests.csv'));
    writetable(omnibus,fullfile(opts.OutputDir,'omnibus_block_tests.csv'));
    localWriteReport(fullfile(opts.OutputDir,'cluster_aware_statistics_report.txt'), ...
        dataFile,opts,nEnv,algNames,continuous,feasibilityRates, ...
        feasibilityPairs,omnibus);
    save(fullfile(opts.OutputDir,'cluster_aware_statistics.mat'), ...
        'envTable','continuous','feasibilityRates','feasibilityPairs', ...
        'omnibus','envJ','envE','envP','envF','envSeeds','algNames','opts');

    results = struct('environment_summary',envTable,'continuous',continuous, ...
        'feasibility_rates',feasibilityRates, ...
        'feasibility_pairwise',feasibilityPairs,'omnibus',omnibus, ...
        'options',opts);
    fprintf('Cluster-aware statistics written to: %s\n',opts.OutputDir);
end

function opts = localDefaults(projectDir,u)
    opts = struct('PenaltyTolerance',1e-12,'Alpha',0.05, ...
        'BootstrapReplicates',20000,'BootstrapSeed',20260808, ...
        'OutputDir',fullfile(projectDir,'cluster_statistics_output'));
    names=fieldnames(u);
    for i=1:numel(names), opts.(names{i})=u.(names{i}); end
end

function T = localEnvironmentTable(envSeeds,algNames,J,E,P,F,S,N)
    nEnv=numel(envSeeds); nAlg=numel(algNames); rows=nEnv*nAlg;
    environment_id=zeros(rows,1); environment_seed=zeros(rows,1);
    algorithm=cell(rows,1); runs=zeros(rows,1); success_count=zeros(rows,1);
    feasibility_proportion=nan(rows,1); median_J=nan(rows,1);
    median_energy_Wh=nan(rows,1); median_penalty=nan(rows,1);
    r=0;
    for e=1:nEnv
        for a=1:nAlg
            r=r+1; environment_id(r)=e; environment_seed(r)=envSeeds(e);
            algorithm{r}=algNames{a}; runs(r)=N(e,a); success_count(r)=S(e,a);
            feasibility_proportion(r)=F(e,a); median_J(r)=J(e,a);
            median_energy_Wh(r)=E(e,a); median_penalty(r)=P(e,a);
        end
    end
    T=table(environment_id,environment_seed,algorithm,runs,success_count, ...
        feasibility_proportion,median_J,median_energy_Wh,median_penalty);
end

function T = localContinuousTests(algNames,J,E,P,bootIdx,opts)
    metricNames={'Composite cost','Energy','Hard-constraint penalty'};
    units={'cost units','Wh','penalty units'}; X={J,E,P};
    nBase=numel(algNames)-1; nRows=numel(X)*nBase;
    metric=cell(nRows,1); unit=cell(nRows,1); baseline=cell(nRows,1);
    environment_N=zeros(nRows,1); median_RA=nan(nRows,1);
    median_baseline=nan(nRows,1); median_difference_baseline_minus_RA=nan(nRows,1);
    ci_low=nan(nRows,1); ci_high=nan(nRows,1); W_plus=nan(nRows,1);
    W_minus=nan(nRows,1); nonzero_N=zeros(nRows,1); p_raw=nan(nRows,1);
    p_holm=nan(nRows,1); rank_biserial=nan(nRows,1);
    effect_direction=repmat({'positive favors RA-ALA'},nRows,1);
    calculation_method=repmat({['exact signed-rank enumeration; average ranks ', ...
        'for ties; zero differences discarded']},nRows,1);
    r=0;
    for m=1:numel(X)
        metricRows=zeros(nBase,1);
        for b=2:numel(algNames)
            r=r+1; metricRows(b-1)=r; x=X{m};
            ok=isfinite(x(:,1)) & isfinite(x(:,b));
            d=x(ok,b)-x(ok,1);
            metric{r}=metricNames{m}; unit{r}=units{m}; baseline{r}=algNames{b};
            environment_N(r)=sum(ok); median_RA(r)=median(x(ok,1));
            median_baseline(r)=median(x(ok,b));
            median_difference_baseline_minus_RA(r)=median(d);
            [ci_low(r),ci_high(r)]=localBootstrapCI(d,bootIdx(:,1:opts.BootstrapReplicates),@median,opts.Alpha);
            [p_raw(r),W_plus(r),W_minus(r),nonzero_N(r),rank_biserial(r)] = ...
                localExactSignedRank(d);
        end
        p_holm(metricRows)=localHolm(p_raw(metricRows));
    end
    T=table(metric,unit,baseline,environment_N,median_RA,median_baseline, ...
        median_difference_baseline_minus_RA,ci_low,ci_high,W_plus,W_minus, ...
        nonzero_N,p_raw,p_holm,rank_biserial,effect_direction,calculation_method);
end

function T = localFeasibilityRates(algNames,F,S,N,bootIdx,opts)
    nAlg=numel(algNames); algorithm=algNames(:); environment_N=repmat(size(F,1),nAlg,1);
    successes=sum(S,1)'; total_runs=sum(N,1)'; feasibility_rate=successes./total_runs;
    cluster_ci_low=nan(nAlg,1); cluster_ci_high=nan(nAlg,1);
    for a=1:nAlg
        [cluster_ci_low(a),cluster_ci_high(a)] = ...
            localBootstrapCI(F(:,a),bootIdx,@mean,opts.Alpha);
    end
    ci_method=repmat({sprintf('percentile cluster bootstrap (%d resamples)', ...
        opts.BootstrapReplicates)},nAlg,1);
    T=table(algorithm,environment_N,successes,total_runs,feasibility_rate, ...
        cluster_ci_low,cluster_ci_high,ci_method);
end

function T = localFeasibilityPairs(algNames,F,bootIdx,opts)
    nBase=numel(algNames)-1; baseline=algNames(2:end)'; environment_N=zeros(nBase,1);
    mean_rate_difference_RA_minus_baseline=nan(nBase,1);
    ci_low=nan(nBase,1); ci_high=nan(nBase,1); p_raw=nan(nBase,1);
    p_holm=nan(nBase,1); nonzero_clusters=zeros(nBase,1);
    effect_direction=repmat({'positive favors RA-ALA'},nBase,1);
    test_method=repmat({'exact cluster-level sign-flip test on environment feasibility proportions'},nBase,1);
    for b=2:numel(algNames)
        r=b-1; ok=isfinite(F(:,1)) & isfinite(F(:,b)); d=F(ok,1)-F(ok,b);
        environment_N(r)=sum(ok); mean_rate_difference_RA_minus_baseline(r)=mean(d);
        [ci_low(r),ci_high(r)]=localBootstrapCI(d,bootIdx,@mean,opts.Alpha);
        [p_raw(r),nonzero_clusters(r)]=localExactSignFlip(d);
    end
    p_holm=localHolm(p_raw);
    T=table(baseline,environment_N,mean_rate_difference_RA_minus_baseline, ...
        ci_low,ci_high,nonzero_clusters,p_raw,p_holm,effect_direction,test_method);
end

function T = localOmnibus(~,J,E,P,F)
    outcomes={'Composite cost';'Energy';'Hard-constraint penalty';'Feasibility proportion'};
    X={J,E,P,F}; statistic=nan(4,1); degrees_of_freedom=nan(4,1);
    p_value=nan(4,1); environment_N=zeros(4,1); algorithms_N=zeros(4,1);
    test=repmat({'Friedman block test with environment as block'},4,1);
    for i=1:4
        [statistic(i),degrees_of_freedom(i),p_value(i),environment_N(i),algorithms_N(i)] = ...
            localFriedman(X{i});
    end
    T=table(outcomes,test,environment_N,algorithms_N,statistic,degrees_of_freedom,p_value);
end

function [p,Wp,Wm,nz,r] = localExactSignedRank(d)
    d=d(isfinite(d)); d=d(abs(d)>1e-12); nz=numel(d);
    if nz==0, p=1; Wp=0; Wm=0; r=0; return; end
    ranks=localTiedRank(abs(d)); total=sum(ranks);
    Wp=sum(ranks(d>0)); Wm=total-Wp; obs=abs(Wp-total/2);
    values=0:(2^nz-1); permWp=zeros(numel(values),1);
    for i=1:nz
        permWp=permWp+ranks(i)*double(bitget(values,i))';
    end
    p=mean(abs(permWp-total/2)>=obs-1e-12);
    r=(Wp-Wm)/total;
end

function [p,nz] = localExactSignFlip(d)
    d=d(isfinite(d)); d=d(abs(d)>1e-12); nz=numel(d);
    if nz==0, p=1; return; end
    obs=abs(mean(d)); values=0:(2^nz-1); stats=zeros(numel(values),1);
    for i=1:nz
        signs=2*double(bitget(values,i))-1;
        stats=stats+signs'*d(i)/nz;
    end
    p=mean(abs(stats)>=obs-1e-12);
end

function adj = localHolm(p)
    adj=nan(size(p)); ok=find(isfinite(p));
    if isempty(ok), return; end
    [ps,ord]=sort(p(ok)); m=numel(ps); vals=zeros(m,1); running=0;
    for i=1:m
        running=max(running,(m-i+1)*ps(i)); vals(i)=min(1,running);
    end
    tmp=zeros(m,1); tmp(ord)=vals; adj(ok)=tmp;
end

function [lo,hi] = localBootstrapCI(x,bootIdx,fun,alpha)
    x=x(:); x=x(isfinite(x)); n=numel(x);
    if n==0, lo=NaN; hi=NaN; return; end
    idx=bootIdx;
    if size(idx,1)~=n
        B=size(idx,2); idx=randi(n,n,B);
    end
    B=size(idx,2); vals=zeros(B,1);
    for b=1:B, vals(b)=fun(x(idx(:,b))); end
    lo=localQuantile(vals,alpha/2); hi=localQuantile(vals,1-alpha/2);
end

function q = localQuantile(x,p)
    x=sort(x(isfinite(x))); n=numel(x);
    if n==0, q=NaN; return; end
    h=1+(n-1)*p; lo=floor(h); hi=ceil(h);
    if lo==hi, q=x(lo); else, q=x(lo)+(h-lo)*(x(hi)-x(lo)); end
end

function ranks = localTiedRank(x)
    [s,ord]=sort(x(:)); n=numel(s); ranks=zeros(n,1); i=1;
    while i<=n
        j=i; while j<n && abs(s(j+1)-s(i))<=1e-12, j=j+1; end
        ranks(i:j)=(i+j)/2; i=j+1;
    end
    tmp=ranks; ranks(ord)=tmp;
end

function [Q,df,p,n,k] = localFriedman(X)
    ok=all(isfinite(X),2); X=X(ok,:); [n,k]=size(X); df=k-1;
    if n==0 || k<2, Q=NaN; p=NaN; return; end
    R=zeros(n,k); tieSum=0;
    for i=1:n
        R(i,:)=localTiedRank(X(i,:))';
        [~,~,g]=unique(X(i,:)); counts=accumarray(g(:),1);
        tieSum=tieSum+sum(counts.^3-counts);
    end
    col=sum(R,1); Q=12/(n*k*(k+1))*sum(col.^2)-3*n*(k+1);
    correction=1-tieSum/(n*(k^3-k));
    if correction>0, Q=Q/correction; else, Q=0; end
    p=gammainc(max(Q,0)/2,df/2,'upper');
end

function localWriteReport(file,dataFile,opts,nEnv,algNames,C,F,FP,O)
    fid=fopen(file,'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'CLUSTER-AWARE SECTION 5.5 STATISTICS\n\n');
    fprintf(fid,'Source: %s\nIndependent unit: urban environment (N=%d)\n',dataFile,nEnv);
    fprintf(fid,'Within-environment aggregation: median of three seeds for continuous outcomes.\n');
    fprintf(fid,'Feasibility criterion: no hard penalty (numerical tolerance %.3g).\n',opts.PenaltyTolerance);
    fprintf(fid,'Algorithms: %s\n\n',strjoin(algNames,', '));
    fprintf(fid,'OMNIBUS TESTS\n');
    for i=1:height(O)
        fprintf(fid,'%s: Friedman Q=%.6g, df=%g, p=%.6g\n', ...
            O.outcomes{i},O.statistic(i),O.degrees_of_freedom(i),O.p_value(i));
    end
    fprintf(fid,'\nFEASIBILITY RATES WITH CLUSTER BOOTSTRAP CI\n');
    for i=1:height(F)
        fprintf(fid,'%s: %d/%d = %.3f, 95%% CI [%.3f, %.3f]\n', ...
            F.algorithm{i},F.successes(i),F.total_runs(i),F.feasibility_rate(i), ...
            F.cluster_ci_low(i),F.cluster_ci_high(i));
    end
    fprintf(fid,'\nPAIRWISE FEASIBILITY CONTRASTS (RA-ALA minus baseline)\n');
    for i=1:height(FP)
        fprintf(fid,'%s: diff=%.3f [%.3f, %.3f], p=%.6g, Holm p=%.6g\n', ...
            FP.baseline{i},FP.mean_rate_difference_RA_minus_baseline(i), ...
            FP.ci_low(i),FP.ci_high(i),FP.p_raw(i),FP.p_holm(i));
    end
    fprintf(fid,'\nCONTINUOUS OUTCOMES (baseline minus RA-ALA; positive effect favors RA-ALA)\n');
    for i=1:height(C)
        fprintf(fid,'%s vs %s: median diff=%.6g [%.6g, %.6g], W+=%.6g, W-=%.6g, p=%.6g, Holm p=%.6g, rank-biserial=%.6g\n', ...
            C.metric{i},C.baseline{i},C.median_difference_baseline_minus_RA(i), ...
            C.ci_low(i),C.ci_high(i),C.W_plus(i),C.W_minus(i), ...
            C.p_raw(i),C.p_holm(i),C.rank_biserial(i));
    end
end
