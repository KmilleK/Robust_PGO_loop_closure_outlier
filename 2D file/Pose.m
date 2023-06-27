classdef Pose 
    properties
        i 
        R            % rotation angle in radian 
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

        function RotationMatrix =generateRotationMatrix(obj)
            RotationMatrix=so2(obj.R,"theta").rotm;
        end

        function displayRotationMatrix(obj)
            disp(so2(obj.R,"theta").rotm);
        end

        function displayInfo(obj)
            disp(['; i: ' ,num2str(obj.i) ,'; R: ',num2str(obj.R) ,'; t: ' ,num2str(obj.t)]);
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