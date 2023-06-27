classdef DataType
    enumeration
        Cube,
        Sphere_g2o,
        Parking,
        Smaller_parking
    end
    
    methods
        function disp(obj)
            fprintf('%s\n', char(obj));
        end
    end
end