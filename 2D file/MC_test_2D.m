%% test cvx basic solution of the PGO 


%%%%%%%%%%%%%%%%%%%%%Initialisation 
clear
folderPath = 'C:\Users\camille\Documents\graduated_thesis\5_6';
subfolders = genpath(folderPath);  % Generate a string containing all subfolders recursively
addpath(subfolders);               % Add the subfolders to the MATLAB path

global n d w_r w_t;

% constant definition

d=2;            % dimension of the Orthogonal and Special Orthogonal group   
w_r=0.01;       % covariance of the rotation noise  
w_t=0.1;         %covariance of the translation noise

N_MC=10;        % Monte carlo number of run 

Noise_addition=true;
Loop_closure=true;

%Pose data creation 

poses = PoseData(0);       % call with 0 basic synthetic dataset  with 1 grid dataset  
n=length(poses);              % number of poses 


ARE_vector=zeros(1,N_MC);
ATE_vector=zeros(1,N_MC);
time_vector=zeros(2,N_MC);   %coordinate 1 of rotation and 2 for translation 
stable_rank_vector=zeros(1,N_MC);

folderName = 'TestingResults';
if ~exist(folderName, 'dir')
    mkdir(folderName);
end

%Measurements data 

for i=1:N_MC

    measurements = PoseToMeasurement(poses,Noise_addition,Loop_closure);
    
    %%%%%%%%%%%%%%%%%%%%% Pose graph optimization process 
    
    %Rotation computation first step 
    
    [poses_inter,rank,timeR]=RotationCVXProblem(measurements);
    
    %Translation computation second step 
    
    [FinalPoses,timeT]=TranslationCVXProblem(poses_inter,measurements);
     
    stable_rank_vector(i)=rank;
    time_vector(1,i)=timeR;
    time_vector(2,i)=timeT;

    % ARE computation 
    ARE_vector(i)=ARE(poses,FinalPoses);
    % ATE computation 
    ATE_vector(i)=ATE(poses,FinalPoses);

end 

%Average of the Monte Carlo run 
Ave_ARE=mean(ARE_vector);
Ave_ATE=mean(ATE_vector);
Ave_rank=mean(stable_rank_vector);
Ave_timeT=mean(time_vector(2,:));
Ave_timeR=mean(time_vector(1,:));

% ARE computation 
disp('Average ARE:');
disp(Ave_ARE);

% ATE computation 

disp('Average ATE:');
disp(Ave_ATE);


%plot results 
%plotRobot(FinalPoses,"Final position after optimization")
