function [poses,measurements] = CreateData(Data_type,n,w_r,w_t,p_out)
% CreateData creates the random graph data for a run of the Monte Carlo
%
% poses : array of Pose3D data 
% measurements : array of Measurement3D data 
% n : integer with the number of robot pose in poses
%
% CreateData(Data_type) return data with the wanted type and 
% default parameters for synthetic datasets 
%   Data_type: element of the DataType enumeration class
%   n: number of poses
%   w_r: covariance of rotation noise 
%   w_t: covariance of translation noise
%   p_out: probability of outlier 


%%%% Author: KESSELER CAMILLE 
    
    switch Data_type 
        case DataType.Erdos
            [poses,measurements] =Erdos_random_graph_creation(n,w_r,w_t,p_out);
        case DataType.Geometric
            [poses,measurements] =Geometric_random_graph_creation(n,w_r,w_t,p_out);
        case DataType.Grid
            [poses,measurements] =Grid_graph_creation(d,n,w_r,w_t,p_out);
        otherwise
            error('Wrong Data type, impossible to create the pose and measurement');
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ERDOS RENYI GRAPH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function [poses,measurements] =Erdos_random_graph_creation(n,w_r,w_t,p_out)

    Delta = 10;    % Size of the environment square of size delta*delta 
       
    % Poses creation
    poses= Pose.empty(n,0);
    
    lowerBoundTranslation= 0;
    upperBoundTranslation= Delta; 
    
    lowerBoundRotation=-pi;
    upperBoundRotation=pi;
    
    %poses(1)=Pose(1,eye(2),[0;0]);
    for i=1:n
        angle_rand=lowerBoundRotation + (upperBoundRotation - lowerBoundRotation) * rand;
        while angle_rand==lowerBoundRotation
            angle_rand=lowerBoundRotation + (upperBoundRotation - lowerBoundRotation) * rand;
        end
        R=so2(angle_rand,"theta").rotm;
        t= lowerBoundTranslation*ones(2,1) + (upperBoundTranslation - lowerBoundTranslation) * rand(2,1);
        poses(i)=Pose(i,R,t);
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
        for i=1:length(disconnection)
            if disconnection(i)==1
                connected_edges(:,iter)=edges_set(i,:)';
                iter=iter+1;
            end
        end
        
        Disconnect=~IsConnected(connected_edges');

        
    end

    connected_edges=connected_edges';
    
    % Measurements creation

    measurements= Measurement.empty(length(connected_edges),0);

    for i = 1:length(connected_edges)
        start_pose=connected_edges(i,1);
        end_pose=connected_edges(i,2);

        %bernouilli variable for the edges outlier/inliers selection  
            
        b=rand<p_out;

        R_in=poses(start_pose).R'*poses(end_pose).R*so2(w_r^2*randn(1),"theta").rotm;
        t_in=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+w_t^2*randn(2, 1);     
        
        uni_angle=lowerBoundRotation + (upperBoundRotation - lowerBoundRotation) * rand;
        R_out=poses(start_pose).R'*poses(end_pose).R*so2(uni_angle,"theta").rotm;
        t_out=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+ (-Delta/4)*ones(2,1) + ((Delta/4) - (-Delta/4)) * rand(2,1);   
    
        R=(1-b)*R_in+b*R_out;
        t=(1-b)*t_in+b*t_out;

        if b==0
            measurements(i)=Measurement('Inlier',start_pose,end_pose,R,t);
        else 
            measurements(i)=Measurement('Outlier',start_pose,end_pose,R,t);

        end
    end
    
end 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GEOMETRIC GRAPH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function [poses,measurements,n] =Geometric_random_graph_creation(n,w_r,w_t,p_out)

    Delta = 10;    % Size of the environment square of size delta*delta
    
    lowerBoundTranslation= 0;
    upperBoundTranslation= Delta; 
    
    lowerBoundRotation=-pi;
    upperBoundRotation=pi;

    % Connected edge set creation 
    Disconnect=1;
    edges_set=permPairs(n);
   
    
    % trial=0;
    while Disconnect

        poses= Pose.empty(n,0);
        for i=1:n
            angle_rand=lowerBoundRotation + (upperBoundRotation - lowerBoundRotation) * rand;
            while angle_rand==lowerBoundRotation
                angle_rand=lowerBoundRotation + (upperBoundRotation - lowerBoundRotation) * rand;
            end
            R=so2(angle_rand,"theta").rotm;
            t= lowerBoundTranslation*ones(2,1) + (upperBoundTranslation - lowerBoundTranslation) * rand(2,1);
            poses(i)=Pose(i,R,t);
        end
        % 
        % trial=trial+1;
        % disp('trial at created the geometric graph:');
        % disp(trial);

        disconnection=connect_check_distance(poses,Delta,edges_set);  % Create edge if close by
        connected_esdges=[];
        iter=1;
        for i=1:length(disconnection)
            if disconnection(i)==1
                connected_edges(:,iter)=edges_set(i,:)';
                iter=iter+1;
            end
        end
        
        Disconnect=~IsConnected(connected_edges');
    
    end

    connected_edges=connected_edges';
    
    % Measurements creation

    measurements= Measurement.empty(length(connected_edges),0);

    for i = 1:length(connected_edges)
        start_pose=connected_edges(i,1);
        end_pose=connected_edges(i,2);

        %bernouilli variable for the edges outlier/inliers selection  
            
        b=rand<p_out;

        R_in=poses(start_pose).R'*poses(end_pose).R*so2(w_r^2*randn(1),"theta").rotm;
        t_in=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+w_t^2*randn(2, 1);     
        
        uni_angle=lowerBoundRotation + (upperBoundRotation - lowerBoundRotation) * rand;
        R_out=poses(start_pose).R'*poses(end_pose).R*so2(uni_angle,"theta").rotm;
        t_out=poses(start_pose).R'*(poses(end_pose).t-poses(start_pose).t)+ (-Delta/4)*ones(2,1) + ((Delta/4) - (-Delta/4)) * rand(2,1);   
    
        R=(1-b)*R_in+b*R_out;
        t=(1-b)*t_in+b*t_out;

        if b==0
            measurements(i)=Measurement('Inlier',start_pose,end_pose,R,t);
        else 
            measurements(i)=Measurement('Outlier',start_pose,end_pose,R,t);

        end
    end
    
end

function connection=connect_check_distance(poses,Delta,edges_set)
    
    connection=[];
    for i=1:length(edges_set)
        if norm(poses(edges_set(i,1)).t - poses(edges_set(i,2)).t) <Delta/3.
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRID GRAPH %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function [poses,measurements,n] =Grid_graph_creation(d,n,w_r,w_t,p_out)
    
end 