function tests = testArrivalTimeSolver
%TESTARRIVALTIMESOLVER Numerical-failure and shared-timeline regression tests.
    tests = functiontests(localfunctions);
end

function opts = options
    opts = struct('tolerance',1e-6,'max_iterations',8, ...
        'max_root_evaluations',64,'max_depth',6,'max_evaluations',512);
end

function testConstantWindAnalyticTime(testCase)
    cm = UnifiedCostModel();
    cm.setEnvironment(struct('getWind',@(x,y,z,t)[-4 0 0]),[],[]);
    [j,d] = cm.evaluatePath([0 0 60;110 0 60],0,true);
    verifyTrue(testCase,isfinite(j) && d.feasible && d.time_converged);
    verifyEqual(testCase,d.T_total,10,'AbsTol',1e-10);
    verifyEqual(testCase,d.time_solver.fixed_point_failures,0);
end

function testBracketedFallbackAndAcceptedMidpoint(testCase)
    opt = options(); opt.max_iterations = 1;
    wf = struct('getWind',@(x,y,z,t)[-5+2*t 0 0]);
    [s,d] = solveArrivalTimeStep([0 0 60],[15 0 60],0,wf,15,0.5,opt);
    verifyTrue(testCase,d.converged);
    verifyEqual(testCase,d.root_successes,1);
    verifyEqual(testCase,sum([s.duration]),(-10+sqrt(160))/2,'AbsTol',1e-6);
    k = computeTrackHoldingKinematics([1 0 0],wf.getWind(7.5,0,60,s.mid_time),15,0.5);
    verifyEqual(testCase,s.kinematics.airVelocity,k.airVelocity,'AbsTol',1e-12);
    verifyLessThanOrEqual(testCase,abs(s.duration-s.length/k.groundSpeed),opt.tolerance);
end

function testSubdivisionWithoutRootSolver(testCase)
    opt = options(); opt.max_iterations = 3; opt.max_root_evaluations = 0;
    wf = struct('getWind',@(x,y,z,t)[-5+2*t 0 0]);
    [s,d] = solveArrivalTimeStep([0 0 60],[15 0 60],0,wf,15,0.5,opt);
    verifyTrue(testCase,d.converged);
    verifyGreaterThan(testCase,d.subdivisions,0);
    verifyEqual(testCase,sum([s.length]),15,'AbsTol',1e-12);
    verifyLessThanOrEqual(testCase,max([s.residual_s]),opt.tolerance);
    for i = 2:numel(s)
        verifyEqual(testCase,s(i).start_time,s(i-1).start_time+s(i-1).duration);
    end
end

function testDiscontinuousSignChangeIsNotAcceptedAsRoot(testCase)
    opt = options(); opt.max_depth = 0;
    wf = struct('getWind',@(x,y,z,t)[-10*double(t<1) 0 0]);
    [s,d] = solveArrivalTimeStep([0 0 60],[15 0 60],0,wf,15,0.5,opt);
    verifyFalse(testCase,d.converged);
    verifyEmpty(testCase,s);
    verifyEqual(testCase,d.root_successes,0);
end

function testDiscontinuousCaseCanBeSubdivided(testCase)
    wf = struct('getWind',@(x,y,z,t)[-10*double(t<1) 0 0]);
    [s,d] = solveArrivalTimeStep([0 0 60],[15 0 60],0,wf,15,0.5,options());
    verifyTrue(testCase,d.converged);
    verifyGreaterThan(testCase,d.subdivisions,0);
    verifyLessThanOrEqual(testCase,max([s.residual_s]),1e-6);
end

function testEvaluationLimitIsEnforced(testCase)
    opt = options(); opt.max_evaluations = 1;
    wf = struct('getWind',@(x,y,z,t)[-4 0 0]);
    [s,d] = solveArrivalTimeStep([0 0 60],[15 0 60],0,wf,15,0.5,opt);
    verifyFalse(testCase,d.converged);
    verifyEmpty(testCase,s);
    verifyEqual(testCase,d.wind_evaluations,1);
    verifyEqual(testCase,d.reason,'ArrivalTime:EvaluationLimit');
end

function testFailedTimeStopsWholePathAndGuidance(testCase)
    cm = UnifiedCostModel();
    cm.time_iteration_max = 1;
    cm.time_root_max_evaluations = 0;
    cm.time_max_subdivision_depth = 0;
    cm.setEnvironment(struct('getWind',@(x,y,z,t)[-4 0 0]),[],[]);
    pts = [0 0 60;15 0 60;30 0 60];
    [j,d] = cm.evaluatePath(pts,0,true);
    verifyEqual(testCase,j,Inf);
    verifyFalse(testCase,d.feasible || d.numerically_valid || d.time_converged);
    verifyEqual(testCase,d.evaluation_status,'time_solver_failure');
    verifyTrue(testCase,all(isnan(d.t_arrivals(2:end))));
    verifyTrue(testCase,isnan(d.penalty_static_collision));
    fitness = evaluateRAALASearchFitness([],@(x)pts,0,true,cm,struct(),struct());
    verifyEqual(testCase,fitness,Inf);
end

function testInvalidWindIsNotReplacedByCalmWind(testCase)
    cm = UnifiedCostModel();
    cm.setEnvironment(struct('getWind',@(x,y,z,t)[NaN 0 0]),[],[]);
    [j,d] = cm.evaluatePath([0 0 60;15 0 60]);
    verifyEqual(testCase,j,Inf);
    verifyEqual(testCase,d.time_failure.reason,'ArrivalTime:InvalidWind');
end

function testWindExceptionIsNotReplacedByCalmWind(testCase)
    cm = UnifiedCostModel();
    cm.setEnvironment(struct('getWind',@(x,y,z,t)error('Test:Wind','Unavailable')),[],[]);
    [j,d] = cm.evaluatePath([0 0 60;15 0 60]);
    verifyEqual(testCase,j,Inf);
    verifyEqual(testCase,d.time_failure.reason,'ArrivalTime:WindQueryFailed');
end

function testAllInvalidCandidatesDoNotTriggerRescue(testCase)
    rng(483,'twister'); env = CityEnvironment(1000,10);
    env.generate('high','medium','dense',483);
    start = [80 80 60]; goal = [90 80 60];
    env.setTaskPoints(start,goal);
    cm = UnifiedCostModel();
    cm.setEnvironment(struct('getWind',@(x,y,z,t)[NaN 0 0]),env.dynObstacles,env.heightMap);
    planner = PathPlanners(env,cm); planner.setBudget(0.01,2,2);
    cfg = struct('popSize',4,'maxIter',1,'nWaypoints',2, ...
        'riskWeight',15,'windLookahead',3,'rescue_max_ins',0);
    plannedPath = []; j = []; d = [];
    evalc('[plannedPath,j,d]=runRA_ALA(planner,cm,env,start,goal,0,true,cfg);');
    verifyEqual(testCase,j,Inf);
    verifyFalse(testCase,d.numerically_valid || d.feasible);
    verifyEqual(testCase,d.evaluation_status,'time_solver_failure');
    verifyFalse(testCase,d.rescueA_stats.triggered || d.rescueB_stats.triggered);
end

function testPhysicalViolationIsNotNumericalFailure(testCase)
    cm = UnifiedCostModel();
    cm.setEnvironment(struct('getWind',@(x,y,z,t)[0 16 0]),[],[]);
    [~,d] = cm.evaluatePath([0 0 60;15 0 60]);
    verifyTrue(testCase,d.numerically_valid && d.time_converged);
    verifyFalse(testCase,d.feasible);
    verifyGreaterThan(testCase,d.penalty_kinematic,0);
end

function testWaitDoesNotShiftNFZMotionToBeforeDeparture(testCase)
    nfz = struct('center',[7.5 0],'radius',1,'height',[0 120], ...
        't_start',10,'t_end',11,'active',true);
    dyn = struct('tempNFZ',nfz,'getAllPositions',@(t)zeros(0,4), ...
        'checkCollision',@(x,y,z,t)false);
    cm = UnifiedCostModel(); cm.setEnvironment([],dyn,[]);
    [~,d] = cm.evaluatePath([0 0 60;15 0 60],0,true,10);
    verifyEqual(testCase,d.T_total,11,'AbsTol',1e-12);
    verifyGreaterThan(testCase,d.penalty_nfz,0);
    verifyEqual(testCase,d.total_nfz_subsamples,d.total_collision_subsamples);
end

function testSubdivisionDoesNotMultiplyPhysicalPenalty(testCase)
    cm = UnifiedCostModel(); cm.setCollisionSampling(15,1);
    cm.time_iteration_max = 3; cm.time_root_max_evaluations = 0;
    cm.setEnvironment(struct('getWind',@(x,y,z,t)[-5+2*t 0 0]),[],70*ones(30));
    [~,d] = cm.evaluatePath([1 1 60;16 1 60]);
    verifyTrue(testCase,d.time_converged);
    verifyGreaterThan(testCase,d.time_solver.subdivisions,0);
    verifyEqual(testCase,d.penalty_static_collision,1,'AbsTol',1e-12);
end

function testMediumWindBoundAndContinuity(testCase)
    rng(483,'twister'); env = CityEnvironment(1000,10);
    env.generate('high','medium','dense',483);
    wf = env.windField; p = wf.params;
    verifyEqual(testCase,p.v_base,3);
    verifyEqual(testCase,p.gust_factor,0.5);
    verifyLessThanOrEqual(testCase,max(wf.windMap.canyonGain(:)),1.3);
    % Triangle-inequality bound for every point/time in the 30--120 m band.
    h = log(120)/log(80);
    a = p.turbulence*(1+2*exp(-30/50))*p.v_base;
    horizontal = (p.v_base*p.urban_gain_cap*h+sqrt(2)*a)*1.2+p.v_base*p.gust_factor;
    vertical = 0.3*a+0.15*p.v_base;
    verifyLessThan(testCase,hypot(horizontal,vertical),10);
    for t = 10:10:100
        left = wf.getWind(50,60,60,t-1e-7);
        right = wf.getWind(50,60,60,t+1e-7);
        verifyLessThan(testCase,norm(left-right),1e-5);
    end
    verifyEqual(testCase,wf.getWind(123,456,70,15),wf.getWind(123,456,70,15));
end
