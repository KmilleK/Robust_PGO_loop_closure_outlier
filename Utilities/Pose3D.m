classdef Pose3D 
    properties
        i 
        R            % rotation matrix 3*3 
        t            % translation vector 3D
    end

    methods 
        function obj=Pose3D(i,R,t)
                obj.i=i;
                obj.R=R;
                obj.t=t;
        end
       
        % function displayRotationMatrix(obj)
        %     disp(so2(obj.R,"theta").rotm);
        % end

        function displayInfo(obj)
            disp(['; i: ' ,num2str(obj.i) ,'; R: ',num2str(obj.R) ,'; t: ' ,num2str(obj.t)]);
        end

        
    end

   
end