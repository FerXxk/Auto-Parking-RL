classdef ParkingLotManager < handle
    % ParkingLotManager is a static class containing helper methods for a
    % parking lot scene.
    %
    % Static methods:
    %
    % 1. findGoalPose - searches for a goal pose within a specified distance.
    % 2. getInfo - returns parking lot parameters in a struct.
    % 3. createReferenceTrajectory - creates a reference trajectory.

    % Copyright 2023 The MathWorks, Inc.

    %% Public static methods
    
    methods (Static)
        function pose = findGoalPose(vehiclePose, freeloc, searchDist)
            arguments
                vehiclePose (1,3) double
                freeloc (1,1) double {mustBePositive,mustBeInteger} = 10
                searchDist (1,1) double {mustBePositive} = 10
            end

            isNorthLoc = freeloc < 15;

            % parking location xy
            info = ParkingLotManager.getInfo();
            locXYs = info.SpotLocations();

            % vehicle xy
            vehicleXY = vehiclePose(1:2);

            % goal xy
            goalXY = locXYs(freeloc,:);

            % distance to parking location
            distToLoc = sqrt(sum((goalXY-vehicleXY).^2));

            % compute the pose
            if distToLoc < searchDist
                if isNorthLoc
                    pose = [goalXY, pi/2];
                else
                    pose = [goalXY, -pi/2];
                end
            else
                pose = [0,0,0];
            end
        end

        function info = getInfo()
            info.XLimits = [-9, 50];
            info.YLimits = [-60,-12];
            info.VehicleDimension = [3.864 1.653];
            info.CenterToFrontDist = 1.104;
            info.CenterToRearDist = 1.343;
            info.TrainingZoneXLimits = [-1.2 12.425];
            info.TrainingZoneYLimits = [-41.34 -29];
            info.SpotDimension = [2.725 5.56];
            info.SpotLocations = [ ...
                30.1375  -25.9250;
                27.4125  -25.9250;
                24.6875  -25.9250;
                21.9625  -25.9250;
                19.2375  -25.9250;
                16.5125  -25.9250;
                13.7875  -25.9250;
                11.0625  -25.9250;
                8.3375  -25.9250;
                5.6125  -25.9250;
                2.8875  -25.9250;
                0.1625  -25.9250;
                -2.5625  -25.9250;
                -5.2875  -25.9250;
                -2.5625  -38.5600;
                0.1625  -38.5600;
                2.8875  -38.5600;
                5.6125  -38.5600;
                8.3375  -38.5600;
                11.0625  -38.5600;
                13.7875  -38.5600;
                16.5125  -38.5600;
                19.2375  -38.5600 ];
        end

        function Xref = createReferenceTrajectory(ts,tf)
            % Get x,y, and theta reference path
            % 3 Sections: North, Left Turn, East

            % North Section (91 points, 0.2 dist.)
            northY = (-55:0.2:-37)';
            northX = 40*ones(length(northY),1);
            northT = 90*ones(length(northY),1);
            northSection = [northX northY northT];

            % Turn Section (50->48 points, 0.1443 dist.)
            th = linspace( 0, pi/2, 50);
            R = 4.5;
            turnX = (R*cos(th)+35.5)';
            turnY = (R*sin(th)-37)';
            turnT = (rad2deg(th)+90)';
            turnSection = [turnX(2:end-1) turnY(2:end-1) turnT(2:end-1)]; % Remove overlapping points

            % West Section (193 points, 0.2 dist.)
            westX = (35.5:-0.2:-3)';
            westY = -32.5*ones(length(westX),1);
            westT = 180*ones(length(westX),1);
            westSection = [westX westY westT];

            % Combine 3 sections into single path
            refPath = [northSection; turnSection; westSection];

            Tsteps = tf/ts; %Add round or ceil
            xRef = [refPath(:,1), refPath(:,2), deg2rad(refPath(:,3))];
            p = size(xRef,1);
            Xref = [xRef(1:p,:);repmat(xRef(end,:),Tsteps-p,1)];
        end
    end
end