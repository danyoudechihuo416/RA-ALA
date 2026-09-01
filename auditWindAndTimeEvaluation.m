function audit = auditWindAndTimeEvaluation(cohortFile,outputDir)
%AUDITWINDANDTIMEEVALUATION Rescore saved RA-ALA geometry; never run a planner.
%   This diagnostic deliberately evaluates historical geometry in the CURRENT
%   wind/evaluator. It is not a replacement for newly optimized experiments.
%   An explicit, new output directory is required to protect archived results.
    if nargin ~= 2 || isfolder(outputDir) || isfile(outputDir)
        error('WindTimeAudit:OutputDirectory','Provide a new diagnostic output directory.');
    end
    S = load(cohortFile);
    before = validationProvenance(cohortFile);
    assert(strcmp(S.windLevel,'medium'),'WindTimeAudit:Preset','This audit is for the medium preset.');
    mkdir(outputDir);
    caseRows = {};
    windRows = {};
    cm = UnifiedCostModel();
    xy = 0:50:S.mapSize;
    heights = [30 60 90 120];
    times = 0:5:900;
    for seed = S.env_seeds_used(:)'
        rng(seed,'twister'); env = CityEnvironment(S.mapSize,S.gridStep);
        env.generate('high',S.windLevel,S.riskLevel,seed);
        env.setTaskPoints(S.startPt,S.goalPt);
        cm.setEnvironment(env.windField,env.dynObstacles,env.heightMap);
        p = env.windField.params;
        a = p.turbulence*(1+2*exp(-cm.H_min/50))*p.v_base;
        h = min(2,max(0.3,log(max(cm.H_max,1.1))/log(80)));
        horizontal = (p.v_base*p.urban_gain_cap*h+sqrt(2)*a)* ...
            (1+p.time_speed_amplitude)+p.v_base*p.gust_factor;
        vertical = 0.3*a+0.15*p.v_base;
        bound = hypot(horizontal,vertical);
        speeds = zeros(numel(xy)^2*numel(heights)*numel(times),1);
        n = 0;
        for x = xy
            for y = xy
                ix = max(1,min(size(env.heightMap,1),round(x)));
                iy = max(1,min(size(env.heightMap,2),round(y)));
                minHeight = max(cm.H_min,env.heightMap(ix,iy)+cm.H_clearance);
                for z = heights
                    if z < minHeight, continue; end
                    for t = times
                        n = n+1;
                        speeds(n) = norm(env.windField.getWind(x,y,z,t));
                    end
                end
            end
        end
        speeds = speeds(1:n);
        windRows(end+1,:) = {seed,n,median(speeds),prctile(speeds,95),max(speeds),bound}; %#ok<AGROW>
        fprintf('WIND seed=%d samples=%d max=%.4f bound=%.4f m/s\n',seed,n,max(speeds),bound);
        for c = find(S.stat_env == seed)
            pts = S.stat_paths{1,c};
            for spacing = [1.5 0.75]
                cm.setCollisionSampling(spacing,3);
                [j,d] = cm.evaluatePath(pts,0,true);
                q = d.time_solver;
                caseRows(end+1,:) = {c,seed,spacing,size(pts,1),d.evaluation_status, ...
                    d.numerically_valid,d.feasible,j,d.E_total,d.T_total,d.R_dynamic, ...
                    d.penalty_total,d.penalty_kinematic,d.total_collision_subsamples, ...
                    q.fixed_point_failures,q.root_attempts,q.root_successes,q.subdivisions, ...
                    q.wind_evaluations,q.max_accepted_residual_s}; %#ok<AGROW>
                fprintf('PATH case=%d spacing=%.2f status=%s feasible=%d fallback=%d split=%d\n', ...
                    c,spacing,d.evaluation_status,d.feasible,q.root_successes,q.subdivisions);
            end
        end
    end
    audit.cases = cell2table(caseRows,'VariableNames',{'Case','Environment','Spacing_m', ...
        'Waypoints','EvaluationStatus','NumericallyValid','Feasible','J','Energy_Wh', ...
        'Time_s','Risk','Penalty','KinematicPenalty','AcceptedSamples', ...
        'FixedPointFailures','RootAttempts','RootSuccesses','Subdivisions', ...
        'WindEvaluations','MaxAcceptedResidual_s'});
    audit.wind = cell2table(windRows,'VariableNames',{'Environment','Samples', ...
        'Median_mps','P95_mps','Max_mps','AnalyticalBound_mps'});
    audit.purpose = 'Diagnostic only: historical fixed geometry, current wind/evaluator; no replanning.';
    audit.spatial_grid_m = xy;
    audit.heights_m = heights;
    audit.times_s = times;
    audit.current_source = before;
    if isfield(S,'validation_source'), audit.archived_source = S.validation_source; end
    after = validationProvenance(cohortFile);
    assert(strcmp(before.cohort_sha256,after.cohort_sha256), ...
        'WindTimeAudit:ArchiveChanged','Input archive changed during the audit.');
    assert(isequal(before.sha256,after.sha256), ...
        'WindTimeAudit:SourceChanged','Scientific source changed during the audit.');
    writetable(audit.cases,fullfile(outputDir,'fixed_geometry_time_audit.csv'));
    writetable(audit.wind,fullfile(outputDir,'wind_domain_audit.csv'));
    save(fullfile(outputDir,'wind_time_audit.mat'),'audit');
    fprintf('DIAGNOSTIC_OUTPUT %s\n',outputDir);
end
