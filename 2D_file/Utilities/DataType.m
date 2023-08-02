classdef DataType
    enumeration
        Erdos,
        Geometric,
        Grid
    end
    
    methods
        function disp(obj)
            fprintf('%s\n', char(obj));
        end
    end
end