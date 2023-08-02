classdef Pose 
    properties
        i 
        R            % rotation matrix R in SO2 
        t            % translation vector 2D
    end

    methods 
        function obj=Pose(i,R,t)
            if nargin ==3
                obj.i=i;
                obj.R=R;
                obj.t=t;
            elseif nargin==2
                obj.i=i;
                obj.R=R;
                obj.t=[0;0];
            end


        end

        

        function displayInfo(obj)
            disp(['; i: ' ,num2str(obj.i) ,'; R: ',num2str(obj.R) ,'; t: ' ,num2str(obj.t)]);
        end

        
    end

   
end