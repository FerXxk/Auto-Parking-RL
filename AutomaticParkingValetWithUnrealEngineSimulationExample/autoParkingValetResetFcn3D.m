function in = autoParkingValetResetFcn3D(in)
% Reset function for auto parking valet example

% Copyright 2021-2023 The MathWorks, Inc.
    
    % Determine which spot to set as target
    spotSelector = rand;
    if spotSelector < 0.25
        freeSpotIndex = 18;
    elseif spotSelector < 0.5
        freeSpotIndex = 6;
    elseif spotSelector < 0.75
        freeSpotIndex = 10;
    else
        freeSpotIndex = 21;
    end

    % Setup/move actor vehicles
    setupActorVehicles("rlAutoParkingValet3D",freeSpotIndex);
    info = ParkingLotManager.getInfo();
    egoTargetPose = localFindGoalPose(freeSpotIndex,info);

    % Set training bounds
    trainXBounds = [(info.SpotLocations(freeSpotIndex,1) - 6.8125) ...
            (info.SpotLocations(freeSpotIndex,1) + 6.8125)];
    if freeSpotIndex >= 15 % lower row of middle aisle
        trainYBounds = [(info.SpotLocations(freeSpotIndex,2) - 2.74) ...
            (info.SpotLocations(freeSpotIndex,2) + 9.56)];
    else % upper row of middle aisle
        trainYBounds = [(info.SpotLocations(freeSpotIndex,2) - 9.56) ...
            (info.SpotLocations(freeSpotIndex,2) + 2.74)];
    end

    % Set initial pose
    choice = rand;
    if choice <= 0.35
        y = -32.5;
        x = info.SpotLocations(freeSpotIndex,1) - 5.6125;
        t = deg2rad(315 + 2*45*rand);
    elseif choice <= 0.70
        y = -32.5;
        x = info.SpotLocations(freeSpotIndex,1) + 5.5875;
        t = deg2rad(135 + 2*45*rand);
    else
        zone = rand;
        if zone <= 0.4
            x = (info.SpotLocations(freeSpotIndex,1) - 4.6125) + 4.5*rand;
            y = -33 + 2*rand;
            t = deg2rad(315 + 2*45*rand);
        elseif zone <= 0.8
            x = (info.SpotLocations(freeSpotIndex,1) + 0.1125) + 4.5*rand;
            y = -33 + 2*rand;
            t = deg2rad(135 + 2*45*rand);
        else
            x = (info.SpotLocations(freeSpotIndex,1) - 1.125) + 2.25*rand;
            if freeSpotIndex <= 14
                y = -34 + 3*rand;
                t = deg2rad(45 + 2*45*rand);
            else
                y = -30 - 3*rand;
                t = deg2rad(225 + 2*45*rand);      
            end
        end
    end
    
    pose = [x,y,t];
    speed = 1 + 3*rand;
    in = setVariable(in,'egoInitialPose',pose);
    in = setVariable(in,'egoInitialSpeed',speed);
    in = setVariable(in,'egoTargetPose',egoTargetPose);
    in = setVariable(in,'trainXBounds',trainXBounds);
    in = setVariable(in,'trainYBounds',trainYBounds);
    in = setVariable(in,'freeSpotIndex',freeSpotIndex);

    % Initialize 2D visualization
    viz = evalin('base','visualizer');
    viz.resetVehicle(pose,freeSpotIndex,1,[trainXBounds trainYBounds])

end

function pose = localFindGoalPose(freeloc,info)
    isNorthLoc = freeloc <= 14;

    % parking location xy
    locXYs = info.SpotLocations();

    % goal xy
    goalXY = locXYs(freeloc,:);

    if isNorthLoc
        pose = [goalXY, pi/2];
    else
        pose = [goalXY, -pi/2];
    end
end