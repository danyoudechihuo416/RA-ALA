function kin = computeTrackHoldingKinematics(trackDirection,windVelocity,airspeed,speedFloor)
%COMPUTETRACKHOLDINGKINEMATICS Prescribed-airspeed 3-D track holding.
%   The vehicle cancels the wind component normal to the requested track
%   and uses the remaining airspeed along the track. The resulting ground
%   velocity is parallel to trackDirection. A negative or vanishing signed
%   progress speed, or a crosswind not smaller than the prescribed
%   airspeed, is reported as kinematically infeasible.

    if nargin < 4 || isempty(speedFloor), speedFloor = 0.5; end
    validateattributes(trackDirection,{'numeric'},{'vector','numel',3,'finite'});
    validateattributes(windVelocity,{'numeric'},{'vector','numel',3,'finite'});
    validateattributes(airspeed,{'numeric'},{'scalar','positive','finite'});
    validateattributes(speedFloor,{'numeric'},{'scalar','positive','finite'});

    e = reshape(double(trackDirection),1,3);
    eNorm = norm(e);
    if eNorm <= eps
        error('TrackKinematics:ZeroDirection','Track direction must be nonzero.');
    end
    e = e/eNorm;
    w = reshape(double(windVelocity),1,3);

    windAlong = dot(w,e);
    windNormal = w-windAlong*e;
    crosswind = norm(windNormal);
    margin = airspeed^2-crosswind^2;
    feasible = margin > 0;
    if feasible
        airAlong = sqrt(margin);
        signedGroundSpeed = windAlong+airAlong;
        feasible = signedGroundSpeed > speedFloor;
    else
        airAlong = 0;
        signedGroundSpeed = -inf;
    end

    if feasible
        groundSpeed = signedGroundSpeed;
        airVelocity = groundSpeed*e-w;
    else
        % A finite speed is retained only so diagnostic energy/time values
        % remain representable; the caller must mark the sample infeasible.
        groundSpeed = speedFloor;
        airVelocity = airAlong*e-windNormal;
    end

    kin = struct('feasible',logical(feasible), ...
        'trackUnit',e,'windAlong',windAlong,'windNormal',windNormal, ...
        'crosswindMagnitude',crosswind,'airAlong',airAlong, ...
        'signedGroundSpeed',signedGroundSpeed,'groundSpeed',groundSpeed, ...
        'groundVelocity',groundSpeed*e,'airVelocity',airVelocity, ...
        'airspeedMagnitude',norm(airVelocity));
end
