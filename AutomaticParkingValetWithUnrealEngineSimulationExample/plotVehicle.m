function plotVehicle(pose,steer,isParking,doTraining)
    if isParking
        status = "PARKING";
    else
        status = "SEARCHING";
    end
    viz = evalin('base','visualizer');
    viz.plot(pose,steer,status,doTraining);
end
