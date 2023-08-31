function [poses,measurements] = Initialize_Pose_Data(Data_type,n,n_lc,w_r,w_t,p_out)
% Creata data poses and measurement for optimization testing in 3D  
%
% OUTPUT
% poses : array of Pose3D data 
% measurements : array of Measurement3D data 
% n : integer with the number of robot pose in poses
%
% INPUT 
% Initialize_Pose_Data(Data_type,n,n_lc,w_r,w_t) return data with the
% wanted type and if the data is synthetic the extra parameters are used
% for creation
%   Data_type: the type of graph to create
%   n: minimum number of poses 
%   n_lc: number of loop closure 
%   w_r: covariance of rotation noise 
%   w_t: covariance of translation noise
%   p_out: percentage of outlier in the whole dataset  

%%%% Author: KESSELER CAMILLE 
    
% Checking input of the function and initialize the parameters if needed 
      
    d=3;                                                                   % 3D testing     

    switch Data_type 
        case DataType.Erdos
            [poses,measurements] =Erdos_random_graph_creation(n,n_lc,w_r,w_t,p_out);
        case DataType.Geometric
            [poses,measurements] =Geometric_random_graph_creation(n,n_lc,w_r,w_t,p_out);
        case DataType.Cube
            [poses,measurements] =Cube_Data_creation(d,n,n_lc,w_r,w_t,p_out);
        % case DataType.Sphere_g2o
        %     [poses,measurements,n] =Data_reading('sphere_bignoise_vertex3.g2o');
        % case DataType.Parking
        %     [poses,measurements,n] =Data_reading('parking.g2o');
        % case DataType.Smaller_parking
        %     [poses,measurements,n] =Data_reading('Small_parking.g2o');
        otherwise
            error('Wrong Data type, impossible to create the pose and measurement');
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ERDOS RENYI GRAPH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function [poses,measurements] =Erdos_random_graph_creation(n,n_lc,w_r,w_t,p_out)

    d=3;
    Delta = 10;    % Size of the environment square of size delta*delta 
       
    % Poses creation
    poses= Pose3D.empty(n,0);
    
    lowerBoundTranslation= 0;
    upperBoundTranslation= Delta; 
    
    lowerBoundRotation=0;
    upperBoundRotation=2*pi;
    
    for i=1:n
        R=eul2rotm(generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation));
        t= lowerBoundTranslation*ones(3,1) + (upperBoundTranslation - lowerBoundTranslation) * rand(3,1);
        poses(i)=Pose3D(i,R,t);
    end
    

    % Connected edge set creation 
    Disconnect=1;
    edges_set=permPairs(n);
    % trial=0;
    while Disconnect

        % trial=trial+1;
        % disp('trial at created the erdos graph:');
        % disp(trial);

        disconnection=(rand(length(edges_set),1)<=0.5);  % Create edge connection with probability of 0.5
        connected_edges=[];
        iter=1;
        connected_edges_out=[];
        iter_out=1;
        for i=1:length(disconnection)
            if disconnection(i)==1
                connected_edges(:,iter)=edges_set(i,:)';
                iter=iter+1;
            else 
                connected_edges_out(:,iter_out)=edges_set(i,:)';
                iter_out=iter_out+1;
            end
        end
        
        Disconnect=~IsConnected(connected_edges');

        
    end

    connected_edges=connected_edges';
    connected_edges_out=connected_edges_out';
    
    % Measurements odometry inliers creation
    
    measurements= Measurement3D.empty(length(connected_edges)+n_lc,0);

    for i = 1:length(connected_edges)

        start_pose=connected_edges(i,1);
        end_pose=connected_edges(i,2);

        R_in=poses(start_pose).R'*poses(end_pose).R*eul2rotm(w_r^2*randn(1,d));
        t_in=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+w_t^2*randn(3, 1);     
       
        measurements(i)=Measurement3D('Odometry',start_pose,end_pose,R_in,t_in);

    end
    
    % Measurements loop closure inliers and outliers creation
    
    loop_closure_index=randsample(length(connected_edges_out),n_lc);
   
    for i = 1:length(loop_closure_index)

        start_pose=connected_edges_out(loop_closure_index(i),1);
        end_pose=connected_edges_out(loop_closure_index(i),2);

        %bernouilli variable for the edges outlier/inliers selection  
            
        b=rand<p_out;

        R_in=poses(start_pose).R'*poses(end_pose).R*eul2rotm(w_r^2*randn(1,d));
        t_in=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+w_t^2*randn(3, 1);     

        
        R_out=poses(start_pose).R'*poses(end_pose).R*eul2rotm(generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation));
        t_out=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+ (-Delta/4)*ones(3,1) + ((Delta/4) - (-Delta/4)) * rand(3,1);   
    
        R=(1-b)*R_in+b*R_out;
        t=(1-b)*t_in+b*t_out;

        measurements(length(connected_edges)+i)=Measurement3D('LoopClosure',start_pose,end_pose,R,t);

    end

end
    
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GEOMETRIC GRAPH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function [poses,measurements,n] =Geometric_random_graph_creation(n,n_lc,w_r,w_t,p_out)

     d=3;
    Delta = 10;    % Size of the environment square of size delta*delta 
       

    
    lowerBoundTranslation= 0;
    upperBoundTranslation= Delta; 
    
    lowerBoundRotation=0;
    upperBoundRotation=2*pi;
    
    

    % Connected edge set creation 
    Disconnect=1;
    edges_set=permPairs(n);
   
    
    % trial=0;
    while Disconnect

           % Poses creation
        poses= Pose3D.empty(n,0);
        for i=1:n
            R=eul2rotm(generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation));
            t= lowerBoundTranslation*ones(3,1) + (upperBoundTranslation - lowerBoundTranslation) * rand(3,1);
            poses(i)=Pose3D(i,R,t);
        end

        disconnection=connect_check_distance(poses,Delta,edges_set);  % Create edge if close by
        connected_edges=[];
        iter=1;
        connected_edges_out=[];
        iter_out=1;
        for i=1:length(disconnection)
            if disconnection(i)==1
                connected_edges(:,iter)=edges_set(i,:)';
                iter=iter+1;
            else 
                connected_edges_out(:,iter_out)=edges_set(i,:)';
                iter_out=iter_out+1;
            end
        end
        
        Disconnect=~IsConnected(connected_edges');
    
    end

    connected_edges=connected_edges';
    connected_edges_out=connected_edges_out';
    
    % Measurements odometry inliers creation
    
    measurements= Measurement3D.empty(length(connected_edges)+n_lc,0);

    for i = 1:length(connected_edges)

        start_pose=connected_edges(i,1);
        end_pose=connected_edges(i,2);

        R_in=poses(start_pose).R'*poses(end_pose).R*eul2rotm(w_r^2*randn(1,d));
        t_in=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+w_t^2*randn(3, 1);     
       
        measurements(i)=Measurement3D('Odometry',start_pose,end_pose,R_in,t_in);

    end
    
    % Measurements loop closure inliers and outliers creation
    
    loop_closure_index=randsample(length(connected_edges_out),n_lc);
   
    for i = 1:length(loop_closure_index)

        start_pose=connected_edges_out(loop_closure_index(i),1);
        end_pose=connected_edges_out(loop_closure_index(i),2);

        %bernouilli variable for the edges outlier/inliers selection  
            
        b=rand<p_out;

        R_in=poses(start_pose).R'*poses(end_pose).R*eul2rotm(w_r^2*randn(1,d));
        t_in=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+w_t^2*randn(3, 1);     

        
        R_out=poses(start_pose).R'*poses(end_pose).R*eul2rotm(generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation));
        t_out=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+ (-Delta/4)*ones(3,1) + ((Delta/4) - (-Delta/4)) * rand(3,1);   
    
        R=(1-b)*R_in+b*R_out;
        t=(1-b)*t_in+b*t_out;

        measurements(length(connected_edges)+i)=Measurement3D('LoopClosure',start_pose,end_pose,R,t);

    end
    
end

function connection=connect_check_distance(poses,Delta,edges_set)
    
    connection=[];
    for i=1:length(edges_set)
        if norm(poses(edges_set(i,1)).t - poses(edges_set(i,2)).t) <Delta/2.
            connection(i)=1;
        else 
            connection(i)=0;
        end

    end 
end

function P=permPairs(N)
    %Produces all pairs of pairs from 1 to N.
    %It is ~30% to 50% faster that nchoosek(1:N,2).
    %Created by Pablo 02/12/2003
    ini_i=1;
    ini_j=ini_i+1;
    r0=1;
    P=[];
    for i=ini_i:N-1
        lj=N-ini_i;
        P(:,r0:lj+r0-1)=[ones(1,lj)*i;ini_j:N];
        r0=r0+lj;
        ini_i=ini_i+1;
        ini_j=ini_i+1;
    end
    P=P';
end

function res=IsConnected(edges)
    
    % Convert the edges and connections into a graph object
    G = graph(edges(:, 1), edges(:, 2));

    % % Find the connected components
    component_ids = conncomp(G);
    
    % Get the number of connected components
    num_components = max(component_ids);

    res=(num_components == 1);  % if 1 then all the nodes are connected in the same graph
    

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Cube_Data_creation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function [poses,measurements,n] =Cube_Data_creation(d,n,n_lc,w_r,w_t,p_out)
    
    
    % Cube dimension in function of the number of node wanted (round to the
    % superior dimension)
    D=10;
    cube_dim=n^(1/3);
    % if ~isinteger(cube_dim)
    %     error('Cube data type need a number of poses which can be decomposed as n=d^3');
    % end
    
    delta_d=D/(cube_dim-1);

    lowerBoundRotation=0;
    upperBoundRotation=2*pi;

    
   
    % Poses data creation

    poses= Pose3D.empty(n,0);
    pose_id=1;
    for z=0:delta_d:D
        for y=0:delta_d:D
            for x=0:delta_d:D
                if(mod(z,2)==0)
                    if(mod(y,2)==0)
                        poses(pose_id)=Pose3D(pose_id,eul2rotm(generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation)),[x;y;z]);
                    else 
                        poses(pose_id)=Pose3D(pose_id,eul2rotm(generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation)),[D-x;y;z]);
                    end
                else
                    if(mod(y,2)==1)
                        poses(pose_id)=Pose3D(pose_id,eul2rotm(generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation)),[x;D-y;z]);
                    else 
                        poses(pose_id)=Pose3D(pose_id,eul2rotm(generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation)),[D-x;D-y;z]);
                    end
    
                end
        
                pose_id=pose_id+1;
            end
                
        end
    end
    
    % Number of poses
    n=length(poses);

    % Measurements data creation
    
    measurements= Measurement3D.empty(n-1+n_lc,0);
   
    for i = 1:n-1                                                          % vector of measurement for odometry
        R_noise=eul2rotm(w_r^2*randn(1,3));
        R_m_gt=poses(i).R'*poses(i+1).R*R_noise;
        t_m_gt=poses(i).R'*(poses(i+1).t-poses(i).t)+w_t^2*randn(3, 1); 
        measurements(i)=Measurement3D('Odometry',i,i+1,R_m_gt,t_m_gt);
    end
    
   
    edges_set=permPairs(n);
    connected_edges_out=[];
    iter_out=1;
    for i=1:length(edges_set)
        if edges_set(i,1)~=edges_set(i,2)+1 && edges_set(i,2)~=edges_set(i,1)+1
            connected_edges_out(:,iter_out)=edges_set(i,:)';
            iter_out=iter_out+1;
        end
    end
    
    connected_edges_out=connected_edges_out';
    loop_closure_index=randsample(length(connected_edges_out),n_lc);
    
    
    for i = 1:length(loop_closure_index)

        start_pose=connected_edges_out(loop_closure_index(i),1);
        end_pose=connected_edges_out(loop_closure_index(i),2);

        %bernouilli variable for the edges outlier/inliers selection  
            
        b=rand<p_out;

        R_in=poses(start_pose).R'*poses(end_pose).R*eul2rotm(w_r^2*randn(1,d));
        t_in=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+w_t^2*randn(3, 1);     

        
        R_out=poses(start_pose).R'*poses(end_pose).R*eul2rotm(generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation));
        t_out=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+ (-D/4)*ones(3,1) + ((D/4) - (-D/4)) * rand(3,1);   
    
        R=(1-b)*R_in+b*R_out;
        t=(1-b)*t_in+b*t_out;

        measurements(n-1+i)=Measurement3D('LoopClosure',start_pose,end_pose,R,t);

    end

end 

function randomAngles = generateRandomEulerAngles(lowerBoundRotation,upperBoundRotation)
    % Generate random angles between 0 and 2*pi for each axis
    roll = lowerBoundRotation + (upperBoundRotation - lowerBoundRotation) * rand();
    pitch = lowerBoundRotation + (upperBoundRotation - lowerBoundRotation) * rand();
    yaw = lowerBoundRotation + (upperBoundRotation - lowerBoundRotation) * rand();

    randomAngles = [roll, pitch, yaw];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% G2o_creation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% function [poses,measurements,n] =Data_reading(filename)
% 
%     % Open and read the G2O file
%     fid = fopen(filename, 'r');
%     data = textscan(fid, '%s', 'Delimiter', '\n');
%     fclose(fid);
% 
%     % Initialize counters for vertices and edges
%     vertex_count = 0;
%     edge_count = 0;
% 
%     % Process the lines in the G2O file
%     for i = 1:numel(data{1})
%         line = data{1}{i};
%         if startsWith(line, 'VERTEX_SE3:QUAT')
%             vertex_count = vertex_count + 1;
%         elseif startsWith(line, 'EDGE_SE3:QUAT')
%             edge_count = edge_count + 1;
%         end
%     end
% 
% 
% 
%     %Poses information storage 
%     poses= Pose.empty(vertex_count,0);
% 
%     %measurement information storage 
%     measurements= Measurement.empty(edge_count,0);
% 
%     vertex_index = 1;
%     edge_index = 1;
%     % Process the lines in the G2O file
%     for i = 1:numel(data{1})
%         line = data{1}{i};
%         if startsWith(line, 'VERTEX_SE3:QUAT')
%             % Extract vertex information
%             parts = strsplit(line);
%             vertex_id = str2double(parts{2});
%             x = str2double(parts{3});
%             y = str2double(parts{4});
%             z= str2double(parts{5});
%             qx = str2double(parts{6});
%             qy = str2double(parts{7});
%             qz = str2double(parts{8});
%             qr = str2double(parts{9});
% 
%             % Store vertex information
%             poses(vertex_index)=Pose(vertex_id+1,quat2rotm([qr,qx,qy,qz]),[x;y;z]);
%             vertex_index = vertex_index+1;
% 
%         elseif startsWith(line, 'EDGE_SE3:QUAT')
%             % Extract edge information
%             parts = strsplit(line);
%             source_id = str2double(parts{2});
%             target_id = str2double(parts{3});
%             x = str2double(parts{4});
%             y = str2double(parts{5});
%             z = str2double(parts{6});
%             qx = str2double(parts{7});
%             qy = str2double(parts{8});
%             qz = str2double(parts{9});
%             qr = str2double(parts{10});
% 
%             % Store edge information
%             if (source_id+1==target_id)
%                 measurements(edge_index)=Measurement('Odometry',source_id+1,target_id+1,quat2rotm([qr,qx,qy,qz]),[x;y;z]);
%             else
%                 measurements(edge_index)=Measurement('LoopClosure',source_id+1,target_id+1,quat2rotm([qr,qx,qy,qz]),[x;y;z]);
%             end
% 
%            edge_index=edge_index+1;
%         end
%     end
% 
%     n=length(poses);
% 
%     disp('the actual pose display are noisy initialisation not the ground truth !!! be careful')
% 
% end 