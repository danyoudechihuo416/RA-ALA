function [samples, info] = solveArrivalTimeStep(p1,p2,t0,windField,airspeed,speedFloor,opts)
%SOLVEARRIVALTIMESTEP Bounded midpoint propagation on one spatial subsegment.
%   Fixed-point iteration is followed by bracketed root finding and, if
%   necessary, spatial bisection. Every accepted leaf satisfies the original
%   time equation at the SAME midpoint used for its kinematics. A failed
%   step returns no samples; callers must not propagate its last iterate.
%   Kinematic infeasibility remains separate: the existing speed-floor
%   continuation is diagnostic only and cannot make a route feasible.

    samples = struct('point',{},'start_time',{},'mid_time',{},'duration',{}, ...
        'length',{},'kinematics',{},'residual_s',{});
    info = struct('converged',false,'reason','not_started', ...
        'fixed_point_failures',0,'root_attempts',0,'root_successes',0, ...
        'subdivisions',0,'max_depth_used',0,'wind_evaluations',0, ...
        'max_fixed_point_iterations',0,'max_accepted_residual_s',0, ...
        'last_residual_s',NaN,'failure_point',[NaN NaN NaN]);
    validateattributes(opts.tolerance,{'numeric'},{'scalar','positive','finite'});
    validateattributes(opts.max_iterations,{'numeric'},{'scalar','integer','positive'});
    validateattributes(opts.max_root_evaluations,{'numeric'},{'scalar','integer','nonnegative'});
    validateattributes(opts.max_depth,{'numeric'},{'scalar','integer','nonnegative'});
    validateattributes(opts.max_evaluations,{'numeric'},{'scalar','integer','positive'});
    direction = p2-p1;
    direction = direction/norm(direction);
    fatalReason = '';
    try
        [samples,ok] = propagate(p1,p2,t0,0);
        info.converged = ok;
        if ok, info.reason = 'converged'; end
    catch err
        if ~startsWith(err.identifier,'ArrivalTime:')
            rethrow(err);
        end
        info.reason = err.identifier;
        info.converged = false;
    end
    if ~info.converged, samples = samples([]); end

    function [leaves,ok] = propagate(a,b,ta,depth)
        leaves = samples([]);
        info.max_depth_used = max(info.max_depth_used,depth);
        mid = (a+b)/2;
        ds = norm(b-a);
        dt = ds/airspeed;
        kin = [];
        ok = false;
        for it = 1:opts.max_iterations
            [res,kin] = residual(dt,mid,ta,ds);
            info.max_fixed_point_iterations = max(info.max_fixed_point_iterations,it);
            if abs(res) <= opts.tolerance
                ok = true;
                break;
            end
            dt = ds/kin.groundSpeed;
        end
        if ~ok
            info.fixed_point_failures = info.fixed_point_failures+1;
            if opts.max_root_evaluations > 0
                info.root_attempts = info.root_attempts+1;
                rootOptions = optimset('Display','off','FunValCheck','on', ...
                    'TolX',opts.tolerance/10, ...
                    'MaxIter',opts.max_root_evaluations, ...
                    'MaxFunEvals',opts.max_root_evaluations);
                try
                    % The current kinematic model never returns a speed
                    % below speedFloor, so this bounds diagnostic travel time.
                    [root,~,exitflag] = fzero(@(d) residual(d,mid,ta,ds), ...
                        [0 ds/speedFloor],rootOptions);
                    [rootResidual,rootKin] = residual(root,mid,ta,ds);
                    if exitflag > 0 && root > 0 && abs(rootResidual) <= opts.tolerance
                        dt = root; res = rootResidual; kin = rootKin; ok = true;
                        info.root_successes = info.root_successes+1;
                    end
                catch err
                    if ~isempty(fatalReason)
                        error(fatalReason,'Arrival-time evaluation cannot continue.');
                    end
                    if startsWith(err.identifier,'ArrivalTime:'), rethrow(err); end
                    % A failed bracket/solver is retried only by bounded bisection.
                end
            end
        end
        if ok
            info.max_accepted_residual_s = max(info.max_accepted_residual_s,abs(res));
            leaves = struct('point',mid,'start_time',ta,'mid_time',ta+dt/2, ...
                'duration',dt,'length',ds,'kinematics',kin,'residual_s',abs(res));
            return;
        end
        if depth >= opts.max_depth || all(mid == a) || all(mid == b)
            info.reason = 'subdivision_limit';
            info.failure_point = mid;
            return;
        end
        info.subdivisions = info.subdivisions+1;
        [left,ok] = propagate(a,mid,ta,depth+1);
        if ~ok, return; end
        nextTime = left(end).start_time+left(end).duration;
        [right,ok] = propagate(mid,b,nextTime,depth+1);
        if ok, leaves = [left right]; end
    end

    function [r,kin] = residual(dt,point,startTime,ds)
        info.failure_point = point;
        if info.wind_evaluations >= opts.max_evaluations
            fatalReason = 'ArrivalTime:EvaluationLimit';
            error(fatalReason,'Subsegment evaluation budget exhausted.');
        end
        info.wind_evaluations = info.wind_evaluations+1;
        w = [0 0 0];
        if ~isempty(windField)
            try
                w = windField.getWind(point(1),point(2),point(3),startTime+dt/2);
            catch
                fatalReason = 'ArrivalTime:WindQueryFailed';
                error(fatalReason,'Wind query failed; no zero-wind substitution is permitted.');
            end
        end
        if ~isnumeric(w) || numel(w) ~= 3 || any(~isfinite(w(:)))
            fatalReason = 'ArrivalTime:InvalidWind';
            error(fatalReason,'Wind query returned invalid values.');
        end
        kin = computeTrackHoldingKinematics(direction,w,airspeed,speedFloor);
        r = dt-ds/kin.groundSpeed;
        info.last_residual_s = abs(r);
    end
end
