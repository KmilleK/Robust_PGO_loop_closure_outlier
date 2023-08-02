classdef KernelFunction
    enumeration
        Identity,
        L1,
        L2,
        Huber
    end
    
    methods
        function disp(obj)
            fprintf('%s\n', char(obj));
        end

        function [color,name] = info_plot(obj)
            name=char(obj);
            switch name
                case 'Identity'
                    color='black-';
                case 'L1'
                    color='blue-';
                case 'L2'
                    color='red-';
                case 'Huber'
                    color='green-';
            end 
        end
    end 

     methods (Static)
         function count=KernelFunctionNumber()
            enumerationClass = meta.class.fromName('KernelFunction');
            enumerationMembers = enumerationClass.EnumerationMemberList;
            count = numel(enumerationMembers);
         end


         function members=KernelFunctionName(i)
            enumerationClass = meta.class.fromName('KernelFunction');
            enumerationMembers = enumerationClass.EnumerationMemberList;
            % Check if the index is valid
            if i >= 1 && i <= numel(enumerationMembers)
                members = enumerationMembers(i).Name;
            else
                error('Invalid index');
            end
         end
    end
end