function plotVehicle(pose,steer,isParking,doTraining)
    if isParking
        status = "PARKING";
    else
        status = "SEARCHING";
    end
    
    % Try to update visualizer if available (skip in parallel workers)
    try
        viz = evalin('base','visualizer');
        viz.plot(pose,steer,status,doTraining);
    catch
        % Visualizer not available in parallel worker environment
    end
end
