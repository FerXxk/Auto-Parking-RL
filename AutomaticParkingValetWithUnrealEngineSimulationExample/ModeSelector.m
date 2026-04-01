classdef ModeSelector < matlab.System 
    % Mode Selector

    % Copyright 2021 The MathWorks, Inc.

    properties
        MaxDepth = 10 % Maximum distance of detection
        MaxViewAngle = 2*pi/3 % Angle of detection cone
        FreeSpotIndex = 18 % Free parking spot index
    end
    
    properties (Nontunable)
        % Parking lot object name
        ParkingLot ParkingLotManager;
    end

    properties(DiscreteState)
        Status
    end

    methods(Access = protected)
        function setupImpl(obj)
        end

        function [isFree,targetPose] = stepImpl(obj,vehiclePose)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            % Extract ParkingLotManager object
            % parkingLot = evalin('base', obj.ParkingLot);
            parkingLot = obj.ParkingLot;

            % Goal XY position
            goalPose = parkingLot.findGoalPose(vehiclePose, ...
                obj.FreeSpotIndex, obj.MaxDepth);

            if all(goalPose==0)
                
            else
                obj.Status = [1 reshape(goalPose, 1, 3)];
            end

            % Output whether free spot is found
            isFree = obj.Status(1);
            targetPose = obj.Status(2:4);
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.Status = [0 0 0 0];
        end

        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds.Status  = obj.Status;
        end

        function validateInputsImpl(obj,vehiclePose)
            % Validate inputs to the step method at initialization
            validateattributes(vehiclePose,{'double'},{'numel',3});
        end

        function flag = isInputSizeMutableImpl(obj,index)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function flag = isInputComplexityMutableImpl(obj,index)
            % Return false if input complexity cannot change
            % between calls to the System object
            flag = false;
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 1;
        end

        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional
            % outputs
            num = 2;
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Mode", "Selector"];
            % icon = ["My","System"]; % Example: multi-line text icon
        end

        function name = getInputNamesImpl(obj)
            % Return input port names for System block
            name = 'Ego Pose';
        end

        function [name,name2] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name = 'Spot Found';
            name2 = 'Target Spot Pose';
        end

        function [out,out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];
            out2 = [1 3];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out,out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out,out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            out2 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out,out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            sz = [1 4];
            dt = "double";
            cp = false;
        end
    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"),...
           'Title','Mode Selector',...
           'Text', 'This system object identifies free parking spots and manages controller mode switching.');
        end
        
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end
