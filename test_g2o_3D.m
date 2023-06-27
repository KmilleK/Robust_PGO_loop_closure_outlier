%% test cvx for g2o data solution of the PGO 

%%%%%%%%%%%%%%%%%%%%% INITIALISATION 

% Matlab workspace and working folder initialisation
clear           
folderPath = 'D:\document\MATLAB\26_6'; 
subfolders_full = genpath(folderPath); 

excludedFolder = 'D:\document\MATLAB\26_6\cvx';  % Specify the folder to exclude
excludedSubfolders = genpath(excludedFolder);  

% Remove the excluded subfolders from the generated path
subfolders = strrep(subfolders_full, [excludedSubfolders ';'], '');% Generate a string containing all subfolders recursively

% Change to your actual folder
addpath(subfolders);                                                       % Add the subfolders to the MATLAB path

% Constant definition

w_r=0.01;                       % Covariance of the rotation noise  
w_t=0.1;                        % Covariance of the translation noise
n_lc=20;                        % Number of additional loop closure in the dataset;

% Choose the robust loss function   Possible chooice
robust_loss_function=KernelFunction.Identity;
percent_outlier=0.00;

%% Data initialisation from g2o

Data_type=DataType.Sphere_g2o;        % Decide the type of data for the testing 

[poses,measurements,n]=Initialize_Pose_Data(Data_type);  

% Measurement update with additional outliers 
Infected_measurements=Update_measurements(poses,measurements,percent_outlier); 

%%%%%%%%%%%%%%%%%%%%% Pose graph optimization process 
    
disp('start optimization');
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
disp('start visualization');

plot_visual_data(poses,Infected_measurements);

plot_visual_solution(FinalPoses);

