classdef DataType
    enumeration
        Cube,
        Parking,
        Smaller_parking,
        Sphere_g2o
    end
    
    methods
        function disp(obj)
            fprintf('%s\n', char(obj));
        end
    end
end