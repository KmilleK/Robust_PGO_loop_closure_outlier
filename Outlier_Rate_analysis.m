%%%% Script for comparison of the impact of outlier rate on dataset 
%%%%
%%%% Author: KESSELER CAMILLE 
%%%%
%%%% 16/06/2023

%%%%%%%%%%%%%%%%%%%%% INITIALISATION 

% Matlab workspace and working folder initialisation
clear           
folderPath = 'C:\Users\camil\Documents\MATLAB\Image_thesis\Robust_PGO_loop_closure_outlier'; 
subfolders_full = genpath(folderPath); 

% Change to your actual folder
addpath(subfolders_full);                                                       % Add the subfolders to the MATLAB path

% Constant definition

n= 27;                         % Number of node 
nb_MC=10;                       % Number of Monte Carlo run 
w_r=0.01;                       % Covariance of the rotation noise  
w_t=0.1;                        % Covariance of the translation noise
n_lc=10;                        % Number of additional loop closure in the dataset;

% Choose if edge separation 
edge_separation=true;

% Data initialisation 

Data_type=DataType.Geometric;        % Decide the type of data for the testing 

%%%%%%%%%%%%%%%%%%%%% LOOP CLOSURE OUTLIER RATE IMPACT 

% Setup for all results
percent_outlier=0.00:0.05:0.3;

Results=cell(5,KernelFunction.KernelFunctionNumber()+1);
Results{1,1}="Kernel Function";
Results{2,1}="Rank";
Results{3,1}="Relaxation time";
Results{4,1}="ARE";
Results{5,1}="ATE";

for i=2:KernelFunction.KernelFunctionNumber()+1
    Results{1,i}=KernelFunction.KernelFunctionName(i-1);
    Results{2,i}=zeros(nb_MC,length(percent_outlier));
    Results{3,i}=zeros(nb_MC,length(percent_outlier));                         
    Results{4,i}=zeros(nb_MC,length(percent_outlier));                   
    Results{5,i}=zeros(nb_MC,length(percent_outlier));
end

% Loop over the loop closure outlier rate
 
for ind_out=1:length(percent_outlier)

    p_out=percent_outlier(ind_out);
   
    %loop over each kernel function 
    for ind_ker=1:KernelFunction.KernelFunctionNumber()
        
        robust_loss_function=KernelFunction.KernelFunctionName(ind_ker);

    
        for ind_MC_run=1:nb_MC
       
            %%%%%%%%%%%%%%%%%%%%% Pose graph optimization process 
            
            [poses,measurements]=Initialize_Pose_Data(Data_type,n,n_lc,w_r,w_t,p_out);
            
            %plot_data(poses,measurements);
            %Rotation computation first step 
            
            [poses_inter,rank,Relaxation_gap,timeR]=RotationCVXProblem3D(measurements,edge_separation,robust_loss_function,n,w_r);
                    
            % Translation computation second step 
            
            [FinalPoses,timeT]=TranslationCVXProblem3D(poses_inter,measurements,edge_separation,robust_loss_function,n,w_t);
                  
            % Error computation 
            ARE_val=ARE_3D(poses,FinalPoses);
            ATE_val=ATE_3D(poses,FinalPoses);

            Results{2,ind_ker+1}(ind_MC_run,ind_out)=rank;                     % Rank information storage 
            Results{3,ind_ker+1}(ind_MC_run,ind_out)=Relaxation_gap;           % relaxation gap information storage                  
            Results{4,ind_ker+1}(ind_MC_run,ind_out)=ARE_val;                  % ARE information storage   
            Results{5,ind_ker+1}(ind_MC_run,ind_out)=ATE_val;                  % ATE information storage
     
        end                                                                % End loop for Monte Carlo Run 
               

    end                                                                    % End loop for kernel function 

    
end                                                                        % End loop for loop closure outlier rate 



%%%%%%%%%%%%%%%%%%%%% RESULTS ANALYSIS  

fig_nam=strcat('results_cell.mat');
filename = fullfile(pwd, 'Results','current',fig_nam );
save(filename,"Results");
plot_Results(percent_outlier,Results);                                     % Plot the rank, time, absolute rotational error and average translational error


