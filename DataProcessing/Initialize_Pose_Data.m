function [poses,measurements,n] = Initialize_Pose_Data(varargin)
% Initialize_Pose_Data format poses and measurement for optimization 
%
% poses : array of Pose3D data 
% measurements : array of Measurement3D data 
% n : integer with the number of robot pose in poses
%
% Initialize_Pose_Data(Data_type) return data with the wanted type and 
% default parameters for synthetic datasets 
%   Data_type: element of the DataType enumeration class
%
% Initialize_Pose_Data(Data_type,n,n_lc,w_r,w_t) return data with the
% wanted type and if the data is synthetic the extra parameters are used
% for creation
%   n: minimum number of poses 
%   n_lc: number of loop closure 
%   w_r: covariance of rotation noise 
%   w_t: covariance of translation noise

%%%% Author: KESSELER CAMILLE 
    
% Checking input of the function and initialize the parameters if needed 
    numArgs=nargin;
    if numArgs==1 
        Data_type=varargin{1};
        n= 100;                        
        w_r=0.01;                         
        w_t=0.1;                        
        n_lc=20;                        
    elseif numArgs==5
        Data_type=varargin{1};
        n=varargin{2};
        n_lc=varargin{3};
        w_r=varargin{4};
        w_t=varargin{5};
    else
        error('Initialise pose Data function do not have the correct number of input');
    end
    
    d=3;                                                                   % 3D testing     

    switch Data_type 
        case DataType.Cube
            [poses,measurements,n] =Cube_Data_creation(d,n,n_lc,w_r,w_t);
        case DataType.Sphere_g2o
            [poses,measurements,n] =Data_reading('sphere_bignoise_vertex3.g2o');
        case DataType.Parking
            [poses,measurements,n] =Data_reading('parking.g2o');
        case DataType.Smaller_parking
            [poses,measurements,n] =Data_reading('Small_parking.g2o');
        otherwise
            error('Wrong Data type, impossible to create the pose and measurement');
    end
end


function [poses,measurements,n] =Cube_Data_creation(d,n,n_lc,w_r,w_t)
    
    
    % Cube dimension in function of the number of node wanted (round to the
    % superior dimension)
            
    cube_dim=1;
    while cube_dim^3 < n
        cube_dim=cube_dim+1;
    end 

    % Poses data creation

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
    
    % Number of poses
    n=length(poses);

    % Measurements data creation
    
    measurements= Measurement.empty(n-1+n_lc,0);
   
    for i = 1:n-1                                                          % vector of measurement for odometry
        R_noise=eul2rotm(w_r^2*randn(1,3));
        R_m_gt=poses(i).R'*poses(i+1).R*R_noise;
        t_m_gt=poses(i).R'*(poses(i+1).t-poses(i).t)+w_t^2*randn(3, 1); 
        measurements(i)=Measurement('Odometry',i,i+1,R_m_gt,t_m_gt);
    end
    
    for i=n:n+n_lc-1                                                       % vector of measurement for loop closure        
        
        % Index selection
        ini=0;
        DoubleLoopClosure=true;
        while ini<=0 || DoubleLoopClosure
            ini= randi([1, n]);
            DoubleLoopClosure=false;
            for j=n+1:i-1
                if measurements(j).i==ini
                    DoubleLoopClosure=true;
                end
            end
        end

        fin=0;
        while fin==ini || fin<=0 || fin>n
            fin=randi([1,n]);
        end

        % loop closure noise creation
        R_noise=eul2rotm(w_r^2*randn(1,d));
        R_m_gt=poses(ini).R'*poses(fin).R*R_noise;
        t_m_gt=poses(ini).R'*(poses(fin).t-poses(ini).t)+w_t^2*randn(d, 1);
        loop_closure=Measurement('LoopClosure',ini,fin,R_m_gt,t_m_gt);
        measurements(i)=loop_closure;
    end

end 

function randomAngles = generateRandomEulerAngles()
    % Generate random angles between 0 and 2*pi for each axis
    roll = 2*pi*rand();
    pitch = 2*pi*rand();
    yaw = 2*pi*rand();

    randomAngles = [roll, pitch, yaw];
end

function [poses,measurements,n] =Data_reading(filename)

    % Open and read the G2O file
    fid = fopen(filename, 'r');
    data = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);

    % Initialize counters for vertices and edges
    vertex_count = 0;
    edge_count = 0;
    
    % Process the lines in the G2O file
    for i = 1:numel(data{1})
        line = data{1}{i};
        if startsWith(line, 'VERTEX_SE3:QUAT')
            vertex_count = vertex_count + 1;
        elseif startsWith(line, 'EDGE_SE3:QUAT')
            edge_count = edge_count + 1;
        end
    end
    


    %Poses information storage 
    poses= Pose.empty(vertex_count,0);

    %measurement information storage 
    measurements= Measurement.empty(edge_count,0);
    
    vertex_index = 1;
    edge_index = 1;
    % Process the lines in the G2O file
    for i = 1:numel(data{1})
        line = data{1}{i};
        if startsWith(line, 'VERTEX_SE3:QUAT')
            % Extract vertex information
            parts = strsplit(line);
            vertex_id = str2double(parts{2});
            x = str2double(parts{3});
            y = str2double(parts{4});
            z= str2double(parts{5});
            qx = str2double(parts{6});
            qy = str2double(parts{7});
            qz = str2double(parts{8});
            qr = str2double(parts{9});
                       
            % Store vertex information
            poses(vertex_index)=Pose(vertex_id+1,quat2rotm([qr,qx,qy,qz]),[x;y;z]);
            vertex_index = vertex_index+1;
 
        elseif startsWith(line, 'EDGE_SE3:QUAT')
            % Extract edge information
            parts = strsplit(line);
            source_id = str2double(parts{2});
            target_id = str2double(parts{3});
            x = str2double(parts{4});
            y = str2double(parts{5});
            z = str2double(parts{6});
            qx = str2double(parts{7});
            qy = str2double(parts{8});
            qz = str2double(parts{9});
            qr = str2double(parts{10});
            
            % Store edge information
            if (source_id+1==target_id)
                measurements(edge_index)=Measurement('Odometry',source_id+1,target_id+1,quat2rotm([qr,qx,qy,qz]),[x;y;z]);
            else
                measurements(edge_index)=Measurement('LoopClosure',source_id+1,target_id+1,quat2rotm([qr,qx,qy,qz]),[x;y;z]);
            end
            
           edge_index=edge_index+1;
        end
    end

    n=length(poses);

    disp('the actual pose display are noisy initialisation not the ground truth !!! be careful')

end 