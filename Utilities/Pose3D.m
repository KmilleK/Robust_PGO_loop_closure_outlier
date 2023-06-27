classdef Pose3 
    properties
        i 
        R            % rotation matrix 3*3 
        t            % translation vector 3D
    end

    methods 
        function obj=Pose3(i,R,t)
            disp(nargin)
            if nargin ==3
                obj.i=i;
                obj.R=R;
                obj.t=t;
            elseif nargin==2
                obj.i=i;
                obj.R=R;
                obj.t=[0;0;0];
            end

        end
       
        % function displayRotationMatrix(obj)
        %     disp(so2(obj.R,"theta").rotm);
        % end

        function displayInfo(obj)
            disp(['; i: ' ,num2str(obj.i) ,'; R: ',num2str(obj.R) ,'; t: ' ,num2str(obj.t)]);
        end

        
    end

   
end