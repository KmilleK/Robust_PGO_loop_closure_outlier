classdef DataType
    enumeration
        Erdos,
        Geometric,
        Cube
    end
    
    methods
        function disp(obj)
            fprintf('%s\n', char(obj));
        end
    end
end