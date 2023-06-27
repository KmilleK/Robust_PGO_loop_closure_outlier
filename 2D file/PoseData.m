function poses = PoseData(data_type)
    % Select the accurate function for the type of pose data create
     if data_type == 0
        poses=BasicSyntheticGroundTruth();
     elseif data_type==1
         poses=GridGroundTruth();
     elseif data_type==2
         poses=BasicSyntheticGroundTruth3();
     elseif data_type==3
         poses=GridGroundTruth3D();
     elseif data_type==4
         poses=RandomGraph3D();
     else
        error('Wrong number of input arguments.');
     end


end


function poses=BasicSyntheticGroundTruth()


%Ground truth

X_gt = [0, 5,  7, 6,   3,   0.5];
Y_gt = [0, 2,  4,  7,   5,   1]; 
R_gt = [0, 30, 90, 190, 210, 260];    %degree angle
R_gt_rad=R_gt*pi/180;


%Poses information storage 
poses= Pose.empty(6,0);

poses(1)=Pose(1,R_gt_rad(1),[X_gt(1);Y_gt(1)]);
poses(2)=Pose(2,R_gt_rad(2),[X_gt(2);Y_gt(2)]);
poses(3)=Pose(3,R_gt_rad(3),[X_gt(3);Y_gt(3)]);
poses(4)=Pose(4,R_gt_rad(4),[X_gt(4);Y_gt(4)]);
poses(5)=Pose(5,R_gt_rad(5),[X_gt(5);Y_gt(5)]);
poses(6)=Pose(6,R_gt_rad(6),[X_gt(6);Y_gt(6)]);


% %Plot of the pose data 
% for i=1:n
%     poses(i).displayRotationMatrix();
% end

%plotRobot(poses,'Robot Position and Orientation ground truth');



end

function poses=GridGroundTruth()

poses= Pose.empty(100,0);
pose_id=1;
for row=0:9
    for col=0:9
        if(mod(row,2)==0)
            poses(pose_id)=Pose(pose_id,0+randn(1,1),[col;row]);
        else 
            poses(pose_id)=Pose(pose_id,pi+randn(1,1),[9-col;row]);
        end

        pose_id=pose_id+1;
            
    end
end
poses(1).R=0;
plotRobot(poses,'Robot Position and Orientation ground truth');

end

function poses=BasicSyntheticGroundTruth3()

%Ground truth
X_gt = [0, 5,  7, 6,   3,   0.5];
Y_gt = [0, 2,  4,  7,   5,   1]; 
Z_gt = [0,0,0,0,0,0];
R_gt = [0, 0, 0;
     pi/4, 0, 0;
     0, pi/6, 0;
     0, 0, pi/3;
     pi/3, pi/6, 0;
     pi/4, pi/6, pi/3];    %euler radian angle 


%Poses information storage 
poses= Pose.empty(6,0);
for i=1:6
    poses(i)=Pose(i,eul2rotm(R_gt(i,:)) ,[X_gt(i);Y_gt(i);Z_gt(i)]);
end 

% %Plot of the pose data 
% for i=1:n
%     poses(i).displayRotationMatrix();
% end

plotRobot3D(poses,'Robot Position and Orientation ground truth');



end


function poses=GridGroundTruth3D()
cube_dim=5;
%Cube of 5*5*5
poses= Pose.empty(cube_dim^3,0);
pose_id=1;
for z=0:cube_dim-1
    for y=0:cube_dim-1
        for x=0:cube_dim-1
            if(mod(z,2)==0)
                if(mod(y,2)==0)
                    poses(pose_id)=Pose(pose_id,eul2rotm(generateRandomEulerAngles()),[x;y;z]);
                else 
                    poses(pose_id)=Pose(pose_id,eul2rotm(generateRandomEulerAngles()),[cube_dim-1-x;y;z]);
                end
            else
                if(mod(y,2)==1)
                    poses(pose_id)=Pose(pose_id,eul2rotm(generateRandomEulerAngles()),[x;cube_dim-1-y;z]);
                else 
                    poses(pose_id)=Pose(pose_id,eul2rotm(generateRandomEulerAngles()),[cube_dim-1-x;cube_dim-1-y;z]);
                end

            end
    
            pose_id=pose_id+1;
        end
            
    end
end
poses(1).R=eye(3);
plotRobot3D(poses,'Robot Position and Orientation ground truth');

end

function poses=RandomGraph3D()
cube_dim=4;
%Cube of 5*5*5
nb_draw=cube_dim^2;

poses= Pose.empty(nb_draw,0);

lowerBoundTranslation= 0;
upperBoundTranslation= cube_dim; 

lowerBoundRotation=-pi;
upperBoundRotation=pi;


for i=1:nb_draw
    R=eul2rotm(lowerBoundRotation*ones(1,3) + (upperBoundRotation - lowerBoundRotation) * rand(1,3));
    t= lowerBoundTranslation*ones(3,1) + (upperBoundTranslation - lowerBoundTranslation) * rand(3,1);
    poses(i)=Pose(i,R,t);
end

poses(1).R=eye(3);
plotRobot3D(poses,'Robot Position and Orientation ground truth');

end


function randomAngles = generateRandomEulerAngles()
    % Generate random angles between 0 and 2*pi for each axis
    roll = 2*pi*rand();
    pitch = 2*pi*rand();
    yaw = 2*pi*rand();

    randomAngles = [roll, pitch, yaw];
end