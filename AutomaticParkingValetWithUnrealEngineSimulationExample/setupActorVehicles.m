function setupActorVehicles(mdl,idxs)
%SETUPACTORVEHICLES Place Unreal Engine actor vehicles in occupied spots
%and clear free spots.
    % Set up static actors and remove any actors from target 
    % parking spot.
    ActorSys = strcat(mdl, ...
        "/Vehicle Dynamics and Sensing/Unreal Engine Visualization and Sensing/Static Vehicles");
    load_system(mdl)

    occupiedSpots = ones(1,23);
    
    % Set free spots
    localValidateIdx(idxs)
    occupiedSpots(idxs) = 0;

    % Find what actors exist in the static vehicle subsystem
    vehicleBlocks = find_system(ActorSys, ...
        IncludeCommented="on", BlockType="SubSystem");
    
    % Place actors in occupied spots, move actor in open spot out of scene
    for i=1:length(occupiedSpots)
        vehNum = num2str(i);
        if any(contains(vehicleBlocks, strcat("Vehicle ",vehNum)))
            % vehBlk = strcat(ActorSys, "/Vehicle ", vehNum);
            XBlk = strcat(ActorSys, "/X", vehNum);
            YBlk = strcat(ActorSys, "/Y", vehNum);
            thetaBlk = strcat(ActorSys, "/theta", vehNum);
            if occupiedSpots(i) == 1
                % set_param(vehBlk, ...
                %     InitialPos="[info.SpotLocations(" + ...
                %     vehNum + ",1), info.SpotLocations(" + ...
                %     vehNum + ",2), 0]");
                set_param(XBlk, Value="info.SpotLocations(" + ...
                    vehNum + ",1)");
                set_param(YBlk, Value="info.SpotLocations(" + ...
                    vehNum + ",2)");
                if vehNum >= 15 % bottom row
                    % set_param(vehBlk, InitialRot="[0 0 -pi/2]");
                    set_param(thetaBlk, Value="-pi/2");
                else % top row
                    % set_param(vehBlk, InitialRot="[0 0 pi/2]");
                    set_param(thetaBlk, Value="pi/2");
                end
            elseif occupiedSpots(i) == 0
                % set_param(vehBlk, InitialPos="[16, -17, 0]");
                set_param(XBlk, Value="16");
                set_param(YBlk, Value="17");
                if vehNum >= 15 % bottom row
                    % set_param(vehBlk, InitialRot="[0 0 -pi/2]");
                    set_param(thetaBlk, Value="-pi/2");
                else
                    % set_param(vehBlk, InitialRot="[0 0 pi/2]");
                    set_param(thetaBlk, Value="pi/2");
                end
            end
        end
    end
end

function localValidateIdx(idx)
    if any(idx<0) || any(idx>23) || any(~isnumeric(idx)) || any((idx-floor(idx))~=0)
        error('Index values must be integers between 1 and 23 (inclusive).');
    end
end