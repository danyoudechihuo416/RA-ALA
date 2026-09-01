function tests = testTrackHoldingKinematics
%TESTTRACKHOLDINGKINEMATICS Unit tests for the shared velocity convention.
    tests = functiontests(localfunctions);
end

function testNoWind(testCase)
    k = computeTrackHoldingKinematics([1 0 0],[0 0 0],15,0.5);
    verifyTrue(testCase,k.feasible);
    verifyEqual(testCase,k.groundSpeed,15,'AbsTol',1e-12);
    verifyEqual(testCase,k.airspeedMagnitude,15,'AbsTol',1e-12);
end

function testHeadwindAndTailwind(testCase)
    kh = computeTrackHoldingKinematics([1 0 0],[-4 0 0],15,0.5);
    kt = computeTrackHoldingKinematics([1 0 0],[4 0 0],15,0.5);
    verifyEqual(testCase,kh.groundSpeed,11,'AbsTol',1e-12);
    verifyEqual(testCase,kt.groundSpeed,19,'AbsTol',1e-12);
end

function testCrosswindTrackHolding(testCase)
    k = computeTrackHoldingKinematics([1 0 0],[0 9 0],15,0.5);
    verifyTrue(testCase,k.feasible);
    verifyEqual(testCase,k.groundVelocity(2:3),[0 0],'AbsTol',1e-12);
    verifyEqual(testCase,k.airspeedMagnitude,15,'AbsTol',1e-12);
    verifyEqual(testCase,k.groundSpeed,12,'AbsTol',1e-12);
end

function testExcessiveCrosswindIsInfeasible(testCase)
    k = computeTrackHoldingKinematics([1 0 0],[0 16 0],15,0.5);
    verifyFalse(testCase,k.feasible);
end
