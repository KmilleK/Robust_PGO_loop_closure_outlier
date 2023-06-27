%% test cvx basic solution of the PGO 

%%%%%%%%%%%%%%%%%%%%% INITIALISATION 

% Matlab workspace and working folder initialisation
clear           
folderPath = 'C:\Users\camille\Documents\graduated_thesis\26_6'; 
subfolders_full = genpath(folderPath); 

excludedFolder = 'C:\Users\camille\Documents\graduated_thesis\26_6';  % Specify the folder to exclude
excludedSubfolders = genpath(excludedFolder);  

% Remove the excluded subfolders from the generated path
subfolders = strrep(subfolders_full, [excludedSubfolders ';'], '');% Generate a string containing all subfolders recursively

% Change to your actual folder
addpath(subfolders);                                                       % Add the subfolders to the MATLAB path

% Constant definition

n= 100;                         % Number of node 
w_r=0.01;                       % Covariance of the rotation noise  
w_t=0.1;                        % Covariance of the translation noise
n_lc=20;                        % Number of additional loop closure in the dataset;

% Choose the robust loss function   Possible chooice
robust_loss_function=KernelFunction.L2;
percent_outlier=0.05;

%% Data initialisation 

Data_type=DataType.Cube;        % Decide the type of data for the testing 

[poses,measurements,n]=Initialize_Pose_Data(Data_type,n,n_lc,w_r,w_t);

% Measurement update with additional outliers 
Infected_measurements=Update_measurements(poses,measurements,percent_outlier,n,n_lc,w_r,w_t);


 %%%%%%%%%%%%%%%%%%%%% Pose graph optimization process 
    

% Rotation computation first step 
            
[poses_inter,rank,timeR]=RotationCVXProblem3D(Infected_measurements,robust_loss_function,n,w_r);
                    
% Translation computation second step 
            
[FinalPoses,timeT]=TranslationCVXProblem3D(poses_inter,Infected_measurements,robust_loss_function,n,w_t);
                  
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

plot_visual_data(poses,Infected_measurements);

plot_visual_solution(FinalPoses);

