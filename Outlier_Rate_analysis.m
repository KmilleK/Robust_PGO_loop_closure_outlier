%%%% Script for comparison of the impact of outlier rate on dataset 
%%%%
%%%% Author: KESSELER CAMILLE 
%%%%
%%%% 16/06/2023

%%%%%%%%%%%%%%%%%%%%% INITIALISATION 

% Matlab workspace and working folder initialisation
clear           
folderPath = 'C:\Users\camille\Documents\graduated_thesis\26_6'; 
subfolders_full = genpath(folderPath); 

excludedFolder = 'C:\Users\camille\Documents\graduated_thesis\26_6\cvx';  % Specify the folder to exclude
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

% Data initialisation 

Data_type=DataType.Cube;        % Decide the type of data for the testing 

[poses,measurements,n]=Initialize_Pose_Data(Data_type,n,n_lc,w_r,w_t);


%%%%%%%%%%%%%%%%%%%%% LOOP CLOSURE OUTLIER RATE IMPACT 

% Setup for all results
percent_outlier=0.00:0.05:0.3;

Results=cell(5,KernelFunction.KernelFunctionNumber()+1);
Results{1,1}="Kernel Function";
Results{2,1}="Rank";
Results{3,1}="Time";
Results{4,1}="ARE";
Results{5,1}="ATE";

for i=2:KernelFunction.KernelFunctionNumber()+1
    Results{1,i}=KernelFunction.KernelFunctionName(i-1);
end


% Loop over the loop closure outlier rate
 
for ind_out=1:length(percent_outlier)
    disp("index outlier percentage");
    disp(ind_out);
    %loop over each kernel function 
    for ind_ker=1:KernelFunction.KernelFunctionNumber()
        
        robust_loss_function=KernelFunction.KernelFunctionName(ind_ker);

        %initialisation for 10 Monte Carlo runs 
        Sum_rank=0;
        Sum_time=0;
        Sum_ARE=0;
        Sum_ATE=0;
    
        for ind_MC_run=1:10
    
            % Measurement update with additional outliers 
            Infected_measurements=Update_measurements(poses,measurements,percent_outlier(ind_out),n,n_lc,w_r,w_t);

            %%%%%%%%%%%%%%%%%%%%% Pose graph optimization process 
    
            % Rotation computation first step 
            
            [poses_inter,rank,timeR]=RotationCVXProblem3D(Infected_measurements,robust_loss_function,n,w_r);
                    
            % Translation computation second step 
            
            [FinalPoses,timeT]=TranslationCVXProblem3D(poses_inter,Infected_measurements,robust_loss_function,n,w_t);
                  
            % Error computation 
            ARE_val=ARE(poses,FinalPoses);
            ATE_val=ATE(poses,FinalPoses);

            % Addition of the results 
            Sum_rank=Sum_rank+rank;
            Sum_time=Sum_time+timeR+timeT;
            Sum_ARE=Sum_ARE+ARE_val;
            Sum_ATE=Sum_ATE+ATE_val;  
        
        end                                                                % End loop for Monte Carlo Run 
        
        % Averaging of the results 
        Average_rank=Sum_rank/10;
        Average_time=Sum_time/10;
        Average_ARE=Sum_ARE/10;
        Average_ATE=Sum_ATE/10;

        Results{2,ind_ker+1}=[Results{2,ind_ker+1},Average_rank];                 % Rank information storage 
        Results{3,ind_ker+1}=[Results{3,ind_ker+1},Average_time];                 % Time information storage                  
        Results{4,ind_ker+1}=[Results{4,ind_ker+1},Average_ARE];                  % ARE information storage   
        Results{5,ind_ker+1}=[Results{5,ind_ker+1},Average_ATE];                  % ATE information storage
        

    end                                                                    % End loop for kernel function 

    
end                                                                        % End loop for loop closure outlier rate 



%%%%%%%%%%%%%%%%%%%%% PLOT RESULTS 

plot_visual_data(poses,Infected_measurements);                              % Plot the ground truth data set used for comparison and the measurement without the additional outliers

plot_Results(percent_outlier,Results);                                     % Plot the rank, time, absolute rotational error and average translational error

