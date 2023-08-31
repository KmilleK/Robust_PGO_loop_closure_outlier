classdef Measurement3D
    properties
        type 
        i 
        j
        R
        t
    end

    methods 
        function obj = Measurement3D(type, i, j, R, t)
            if nargin > 0
                obj.type = type;
                obj.i = i;
                obj.j = j;
                obj.R = R;
                obj.t = t;
            end
        end

        function displayInfo(obj)
            disp(['Type: ', obj.type, '; i: ', num2str(obj.i), '; j: ', num2str(obj.j), '; R: ', num2str(obj.R), '; t: ', num2str(obj.t)]);
        end
    end

end
