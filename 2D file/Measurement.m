classdef Measurement
    properties
        type 
        i 
        j
        R
        t
    end

    methods 
        function obj = Measurement(type, i, j, R, t)
            if nargin > 0
                obj.type = type;
                obj.i = i;
                obj.j = j;
                obj.R = R;
                obj.t = t;
            end
        end

        function RotationMatrix = generateRotationMatrix(obj)
            RotationMatrix = so2(obj.R, "theta").rotm;
        end

        function displayInfo(obj)
            disp(['Type: ', obj.type, '; i: ', num2str(obj.i), '; j: ', num2str(obj.j), '; R: ', num2str(obj.R), '; t: ', num2str(obj.t)]);
        end
    end

    methods (Static)
        function adjustedAngle = adjustAngle(angle)
            while angle <= -pi
                angle = angle + 2*pi;
            end

            while angle > pi
                angle = angle - 2*pi;
            end

            adjustedAngle = angle;
        end
    end
end
