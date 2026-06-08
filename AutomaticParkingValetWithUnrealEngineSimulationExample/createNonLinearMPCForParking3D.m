%% Design MPC controllers for tracking

% Copyright 2021 The MathWorks, Inc.

%% Reference trajectory
Xref = ParkingLotManager.createReferenceTrajectory(Ts,Tf);

%% Common tracking parameters
pTracking = 10;

x = egoInitialPose';
u = [0; 0];

%% Adaptive MPC for tracking
[Ad,Bd,Cd,Dd,U0,Y0,X0,DX0] = vehicleStateJacobianFcnDT(Ts,x,u);
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);

mpcobj = createAdaptiveMPC(pTracking,dsys,Ts);

%% Nonlinear MPC for tracking
nlobjTracking = createNonlinearMPC(pTracking,Ts);

% Optional but recommended: check that the nonlinear model is consistent
validateFcns(nlobjTracking,x,u);

%% ============================================================
% Local function: Design Adaptive MPC Tracking Controller
% ============================================================
function mpcobj = createAdaptiveMPC(pTracking,dsys,Ts)

mpcverbosity('off');

mpcobj = mpc(dsys);

% MPC settings
mpcobj.Ts = Ts;
mpcobj.PredictionHorizon = pTracking;
mpcobj.ControlHorizon = pTracking;

% Manipulated variables:
% u(1) = v      longitudinal velocity
% u(2) = delta  steering angle
mpcobj.MV(1).Min = -5;
mpcobj.MV(1).Max = 5;

mpcobj.MV(2).Min = -pi/4;
mpcobj.MV(2).Max = pi/4;

% Weights
mpcobj.Weights.OutputVariables = [2, 2, 3];
mpcobj.Weights.ManipulatedVariablesRate = [0.1, 0.2];

% Optional absolute input penalty
% mpcobj.Weights.ManipulatedVariables = [0, 0.01];

% Custom estimator
setoutdist(mpcobj,'model',tf(ones(3,1)));
setEstimator(mpcobj,'custom');

end

%% ============================================================
% Local function: Design Nonlinear MPC Tracking Controller
% ============================================================
function nlobjTracking = createNonlinearMPC(pTracking,Ts)

% Nonlinear bicycle model:
%
% x = [X; Y; theta]
% u = [v; delta]
%
% X_dot     = v cos(theta)
% Y_dot     = v sin(theta)
% theta_dot = v / L tan(delta)
%
% This is implemented in vehicleStateFcn.m

nx = 3;   % states: X, Y, theta
ny = 3;   % outputs: X, Y, theta
nu = 2;   % inputs: v, delta

nlobjTracking = nlmpc(nx,ny,nu);

%% Sampling and horizons
nlobjTracking.Ts = Ts;
nlobjTracking.PredictionHorizon = pTracking;
nlobjTracking.ControlHorizon = pTracking;

%% Nonlinear prediction model
nlobjTracking.Model.StateFcn = "vehicleStateFcn";
nlobjTracking.Model.IsContinuousTime = true;

% Since ny = nx = 3, the default output is:
% y = x = [X; Y; theta]
% Therefore no explicit OutputFcn is strictly necessary.

%% State names
nlobjTracking.States(1).Name = "X";
nlobjTracking.States(2).Name = "Y";
nlobjTracking.States(3).Name = "theta";

%% Manipulated variable names
nlobjTracking.MV(1).Name = "v";
nlobjTracking.MV(2).Name = "delta";

%% Output names
nlobjTracking.OutputVariables(1).Name = "X";
nlobjTracking.OutputVariables(2).Name = "Y";
nlobjTracking.OutputVariables(3).Name = "theta";

%% Physical constraints
% u(1) = v
nlobjTracking.MV(1).Min = -5;
nlobjTracking.MV(1).Max = 5;

% u(2) = delta
nlobjTracking.MV(2).Min = -pi/4;
nlobjTracking.MV(2).Max = pi/4;

%% Weights equivalent to the Adaptive MPC
nlobjTracking.Weights.OutputVariables = [2, 2, 3];
nlobjTracking.Weights.ManipulatedVariablesRate = [0.1, 0.2];

% Optional absolute input penalty, equivalent to the commented line
% in your Adaptive MPC.
% nlobjTracking.Weights.ManipulatedVariables = [0, 0.01];

%% Solver configuration
nlobjTracking.Optimization.SolverOptions.MaxIterations = 50;
nlobjTracking.Optimization.SolverOptions.ConstraintTolerance = 1e-3;
nlobjTracking.Optimization.SolverOptions.OptimalityTolerance = 1e-3;

end