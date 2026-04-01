classdef ParkingLotVisualizer < handle
    % ParkingLotVisualizer

    % Copyright 2023 The MathWorks, Inc.

    properties (Transient, Access = private)
        Figure
        Ax

        SpotRect            = gobjects()
        EgoVehicleRect      = gobjects()
        StaticVehicleRect   = gobjects()
        
        VehicleBodyRect
        VehicleAxleRects
        VehicleWheelRects
        VehiclePath
        VehicleStatusText

        TrainZoneRect
    end

    properties (Access = private)
        Info
        Ts
        Tf
    end

    methods
        function this = ParkingLotVisualizer(freeLoc,Ts,Tf)
            this.Info = ParkingLotManager.getInfo();
            this.Ts = Ts;
            this.Tf = Tf;

            buildFigure_(this);
            setFree(this,freeLoc);
        end
        function setFree(this,freeloc)
            arguments
                this
                freeloc (1,1) double {mustBePositive,mustBeInteger}
            end
            if isgraphics(this.Ax)
                for idx = 1:numel(this.StaticVehicleRect)
                    if idx==freeloc
                        this.StaticVehicleRect(idx).Visible = "off";
                        this.SpotRect(idx).FaceColor = "g";
                    else
                        this.StaticVehicleRect(idx).Visible = "on";
                        this.SpotRect(idx).FaceColor = "r";
                    end
                end
            end
        end
        function plot(this,pose,steer,status,tzone)
            if isgraphics(this.Ax)
                plotVehicle_(this,pose,steer);
                plotVehiclePath_(this,pose);
                setVehicleStatus_(this,status);
                info = this.Info;
                toggleTrainingZone_(this,tzone,...
                    [info.TrainingZoneXLimits info.TrainingZoneYLimits]);
            end
        end
        function resetVehicle(this,pose,spotID,isTraining,trainBounds)
            % Remove any existing vehicle path object
            this.VehiclePath = [];
            pathObj = findobj(this.Ax,Tag="Vehicle Path");
            delete(pathObj);

            % Remove any existing training zone drawing
            this.TrainZoneRect = [];
            trainZoneObj = findobj(this.Ax,Tag="Training Zone");
            delete(trainZoneObj);

            % Initialize free spot
            this.setFree(spotID);
            
            if ~isTraining
                % Set vehicle to initial position
                this.plotVehicle_(pose,0)
            end

            if isTraining
                % Re-initialize training zone
                toggleTrainingZone_(this,true,trainBounds);
            end
        end
    end

    methods (Access = private)
        function buildFigure_(this)
            % Build the figure
            f = figure( ...
                'Position',[435 200 800 600], ...
                'Name','Parking Lot', ...
                'NumberTitle','off', ...
                'MenuBar','none');
            ax = gca(f);

            legend(ax,'off');
            hold(ax, 'on');

            ax.XLim = this.Info.XLimits;
            ax.YLim = this.Info.YLimits;

            this.Figure = f;
            this.Ax = ax;

            % Plot parking lot environment image
            img = imread('ParkingLotEnvironmentSubsection.jpg');
            image(ax,'CData',img,'XData',[-9, 50],'YData',[-12,-60]);

            % Plot reference path
            xRef = ParkingLotManager.createReferenceTrajectory(this.Ts,this.Tf);
            plot(this.Ax, xRef(:,1), xRef(:,2), '-.m', 'LineWidth', 1.5)

            % Plot rectangles
            plotCarRectangles_(this);
            plotSpotRectangles_(this);
            plotLocationText_(this);
        end

        function plotSpotRectangles_(this)
            % Plot location rectangles and numbers
            locXYs = this.Info.SpotLocations();
            numSpots = size(locXYs,1);
            yoffset = 4.5;
            spotSize = [1,1];
            for idx = 1:numSpots
                rx = locXYs(idx,1);
                ry = locXYs(idx,2);
                % spot markers
                if idx < 15
                    spotRectPos = [rx-0.5*spotSize(1), ry-yoffset-0.5*spotSize(2), spotSize(1), spotSize(2)];
                else
                    spotRectPos = [rx-0.5*spotSize(1), ry+yoffset-0.5*spotSize(2), spotSize(1), spotSize(2)];
                end
                this.SpotRect(idx) = rectangle(this.Ax,'Position',spotRectPos,'FaceColor','r');
            end
        end

        function plotCarRectangles_(this)
            % Plot car rectangles
            locXYs = this.Info.SpotLocations();
            numSpots = size(locXYs,1);
            vehicleDim = this.Info.VehicleDimension;
            for idx = 1:numSpots
                rx = locXYs(idx,1);
                ry = locXYs(idx,2);
                vehicleRectPos = [rx-0.5*vehicleDim(2), ry-0.5*vehicleDim(1), vehicleDim(2), vehicleDim(1)];
                this.StaticVehicleRect(idx) = rectangle(this.Ax,'Position',vehicleRectPos,'FaceColor','k');
            end
        end

        function plotLocationText_(this)
            % Plot location rectangles and numbers
            locXYs = this.Info.SpotLocations();
            numSpots = size(locXYs,1);
            txtoffset = 2.5;
            for idx = 1:numSpots
                rx = locXYs(idx,1);
                ry = locXYs(idx,2);
                if idx < 15
                    txtxy = [rx-0.5, ry-txtoffset-0.5];
                else
                    txtxy = [rx-0.5, ry+txtoffset+0.5];
                end
                text(this.Ax, txtxy(1), txtxy(2), sprintf('%02u',idx), 'Color','w','FontSize',7);
            end
        end

        function plotVehicle_(this,pose,steer)
            ax = this.Ax;
            
            centerToFrontDist = this.Info.CenterToFrontDist;
            centerToRearDist = this.Info.CenterToRearDist;
            vdim = this.Info.VehicleDimension;
            
            % extract data and translate pose to center of vehicle
            th = pose(3);
            xc = pose(1) + centerToRearDist*cos(th);
            yc = pose(2) + centerToRearDist*sin(th);
            
            % corners of the vehicle rectangle in the order LU LD RD RU
            cornersx = xc + 0.5*vdim(1)*[-1 -1 1 1];
            cornersy = yc + 0.5*vdim(2)*[1 -1 -1 1];
            vbody = rotate( polyshape(cornersx,cornersy), rad2deg(th), [xc yc] );
            
            % prepare axles
            axwid = vdim(1)/50; % axle width
            faxlex = xc + centerToFrontDist*[1 1 1 1]+ 0.5*axwid*[-1 -1 1 1];
            raxlex = xc + centerToRearDist*[-1 -1 -1 -1]+ 0.5*axwid*[-1 -1 1 1];
            faxley = yc + 0.5*vdim(2)*[1 -1 -1 1];
            raxley = faxley;
            axles(1) = rotate( polyshape(faxlex,faxley), rad2deg(th), [xc yc] );
            axles(2) = rotate( polyshape(raxlex,raxley), rad2deg(th), [xc yc] );
            
            % prepare wheels
            whlen = vdim(1)/8;  % wheel rectangle length
            whwid = vdim(1)/30; % wheel rectangle width
            whx = 0.5*whlen*[-1 -1 1 1]; % wheel rectangle corners x
            why = 0.5*whwid*[1 -1 -1 1]; % wheel rectangle corners y
            wheels0(1) = rotate( polyshape(xc-centerToRearDist+whx, yc+0.5*vdim(2)+why), rad2deg(th), [xc yc] );
            wheels0(2) = rotate( polyshape(xc-centerToRearDist+whx, yc-0.5*vdim(2)+why), rad2deg(th), [xc yc] );
            wheels0(3) = rotate( polyshape(xc+centerToRearDist+whx, yc-0.5*vdim(2)+why), rad2deg(th), [xc yc] );
            wheels0(4) = rotate( polyshape(xc+centerToRearDist+whx, yc+0.5*vdim(2)+why), rad2deg(th), [xc yc] );
            wheels(1) = wheels0(1);
            wheels(2) = wheels0(2);
            wheels(3) = rotate( wheels0(3), rad2deg(steer), [axles(1).Vertices(3,1), axles(1).Vertices(3,2)+0.5*axwid] );
            wheels(4) = rotate( wheels0(4), rad2deg(steer), [axles(1).Vertices(2,1), axles(1).Vertices(2,2)+0.5*axwid] );
            
            if isempty(this.VehiclePath)
                this.VehiclePath = animatedline(ax,pose(1),pose(2),'LineWidth',3,'Color','g');
                this.VehiclePath.Tag = "Vehicle Path";
            end
            if isempty(this.VehicleBodyRect) || any(~isvalid(this.VehicleBodyRect))
                this.VehicleBodyRect = plot(ax,vbody,'FaceColor','r','FaceAlpha',0.25);
                this.VehicleAxleRects = plot(ax,axles,'FaceColor','k','FaceAlpha',1);
                this.VehicleWheelRects = plot(ax,wheels,'FaceColor','k','FaceAlpha',1);
            else
                this.VehicleBodyRect.Shape = vbody;
                for i = 1:2
                    this.VehicleAxleRects(i).Shape = axles(i);
                end
                for i = 1:4
                    this.VehicleWheelRects(i).Shape = wheels(i);
                end
            end
        end

        function plotVehiclePath_(this,pose)
            addpoints(this.VehiclePath,pose(1),pose(2));
        end

        function toggleTrainingZone_(this,toggle,trainBounds)
            trainingZoneXLimits = trainBounds(1:2);
            trainingZoneYLimits = trainBounds(3:4);
            if toggle && (isempty(this.TrainZoneRect) || ~isvalid(this.TrainZoneRect))
                ax = this.Ax;
                x = [trainingZoneXLimits(1) trainingZoneXLimits(2) trainingZoneXLimits(2) trainingZoneXLimits(1)];
                y = [trainingZoneYLimits(1) trainingZoneYLimits(1) trainingZoneYLimits(2) trainingZoneYLimits(2)];
                this.TrainZoneRect = fill(ax,x,y,'r','FaceAlpha',0.1,Tag="Training Zone");
            elseif ~toggle
                this.TrainZoneRect.Visible = "off";
            end
        end

        function setVehicleStatus_(this,txt)
            ax = gca(this.Figure);
            if isempty(this.VehicleStatusText) || ~isvalid(this.VehicleStatusText)
                this.VehicleStatusText = text(ax, 40, -15, txt);
                this.VehicleStatusText.Color = "y";
            else
                this.VehicleStatusText.String = txt;
            end
        end
    end
end

