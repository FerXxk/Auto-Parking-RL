%% Parameters used by Auto Parking Valet 3D using MPC and RL example

% Copyright 2021 The MathWorks, Inc.

% Ego vehicle dimension
centerToRear  = 1.343;      % distance from center to rear axle (m)

% Lidar parameters
maxLidarDist = 20;          % maximum distance that can be measured by lidar (m)
lidarHorizontalFOV = 360;   % Horizontal field of view (deg)
lidarHorizontalRes = 1;     % Horizontal resolution (deg)
lidarVerticalFOV = 80;      % Vertical field of view (deg)
lidarVerticalRes = 1;       % Horizontal resolution (deg)
numSensors   = [lidarHorizontalFOV/lidarHorizontalRes ...
    lidarVerticalFOV/lidarVerticalRes];    % dimensions of lidar readings
lidarTol    = 0.2;          % minimum distance measured by lidar (m)

% Error tolerances with target pose
xyerrTol    = 0.75;         % position error tolerance w.r.t. target pose (m)
terrTol     = deg2rad(10);  % orientation error tolerance w.r.t. target pose (m)

% Camera parameters
cameraDepth = 10;               % camera depth (m)
cameraViewAngle = deg2rad(120); % camera field of view (rad)

% Parameters for training
speedMax = 2;               % maximum speed of ego vehicle (m/s)
steerMax = pi/4;            % maximum steering angle (rad)
trainXBounds  = info.TrainingZoneXLimits;
trainYBounds  = info.TrainingZoneYLimits;
trainTBounds  = [-2*pi 2*pi];

% Parameters for simulation
xBounds  = info.XLimits;
yBounds  = info.YLimits;
tBounds  = [-Inf Inf];