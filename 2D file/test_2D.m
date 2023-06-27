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

Noise_addition=true;
Loop_closure=true;

%Pose data creation 

poses = PoseData(1);       % call with 0 basic synthetic dataset  with 1 grid dataset  
n=length(poses);              % number of poses 

%Measurements data 

measurements = PoseToMeasurement(poses,Noise_addition,Loop_closure);

%%%%%%%%%%%%%%%%%%%%% Pose graph optimization process 

%Rotation computation first step 

poses_inter=RotationCVXProblem(measurements);

%Translation computation second step 

FinalPoses=TranslationCVXProblem(poses_inter,measurements);

% Analysis the result 

% ARE computation 
MyARE=ARE(poses,FinalPoses);
disp('ARE:');
disp(MyARE);

% ATE computation 
MyATE=ATE(poses,FinalPoses);
disp('ATE:');
disp(MyATE);


%plot results 
plotRobot(FinalPoses,"Final position after optimization")

save('workspace_All.mat');
