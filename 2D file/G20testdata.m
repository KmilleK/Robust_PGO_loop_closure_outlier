%% test cvx g2o solution of the PGO 


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



[poses,measurements] = g20Data('input_M3500.g2o');

n=length(poses);              % number of poses 

%%%%%%%%%%%%%%%%%%%%% Pose graph optimization process 

%Rotation computation first step 
disp('start rotation solver');
poses_inter=RotationCVXProblem(measurements);

disp('finish rotation solver');
%Translation computation second step 

disp('start translation solver');
FinalPoses=TranslationCVXProblem(poses_inter,measurements);

disp('finish translation solver');

%plot results
plotRobot(FinalPoses,"Final position after optimization")

