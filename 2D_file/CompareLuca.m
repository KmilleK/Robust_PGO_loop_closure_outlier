%% test cvx comparison with Luca 2D article of the PGO 

%%%%%%%%%%%%%%%%%%%%% INITIALISATION 

% Matlab workspace and working folder initialisation
clear           
folderPath = 'C:\Users\camil\Documents\MATLAB\Image_thesis\Robust_PGO_loop_closure_outlier\2D_file'; 
subfolders_full = genpath(folderPath); 

% Change to your actual folder
addpath(subfolders_full);                                                       % Add the subfolders to the MATLAB path

% % Constant definition
% 
n= 20;                         % Number of node 
nb_MC=1;                       % Number of Monte Carlo run 
w_r=0.01;                       % Covariance of the rotation noise  
w_t=0.1;                        % Covariance of the translation noise

%% Data initialisation 

Data_type=DataType.Geometric;        % Decide the type of data for the testing 

%%%%%%%%%%%%%%%%%%%%% LOOP CLOSURE OUTLIER RATE IMPACT 

% Setup for all results
percent_outlier=0.2:0.05:0.3;

Results=cell(5,KernelFunction.KernelFunctionNumber()+1);
Results{1,1}="Kernel Function";
Results{2,1}="Rank";
Results{3,1}="Relaxation Gap";
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

        %initialisation for 10 Monte Carlo runs 

        for ind_MC_run=1:nb_MC
                
            % Creation of the dataset and the noisy measurements
            [poses,measurements]=CreateData(Data_type,n,w_r,w_t,p_out);       
            
            %%%%%%%%%%%%%%%%%%%%% Pose graph optimization process 
            
            % Rotation computation first step 
            [poses_inter,rank,Relaxation_gap,timeR]=RotationCVXProblem2D(measurements,robust_loss_function,n,w_r);
          
            
            % Translation computation second step 
            [FinalPoses,timeT]=TranslationCVXProblem2D(poses_inter,measurements,robust_loss_function,n,w_t);
            
            %%%%%%%%%%%%%%%%%%%%% Analysis the result 
            
            % Error computation 
            ARE_val=ARE(poses,FinalPoses);
            ATE_val=ATE(poses,FinalPoses);

            Results{2,ind_ker+1}(ind_MC_run,ind_out)=rank;                     % Rank information storage 
            Results{3,ind_ker+1}(ind_MC_run,ind_out)=Relaxation_gap;           % relaxation gap information storage                  
            Results{4,ind_ker+1}(ind_MC_run,ind_out)=ARE_val;                  % ARE information storage   
            Results{5,ind_ker+1}(ind_MC_run,ind_out)=ATE_val;                  % ATE information storage
     

        end                                         % End loop for Monte Carlo Run 


    end                                                             % End loop for kernel function 

end                                                                 % End loop for loop closure outlier rate 

%%%%%%%%%%%%%%%%%%%%% RESULTS ANALYSIS  

fig_nam=strcat('results_cell.mat');
filename = fullfile(pwd, 'Results','current',fig_nam );
save(filename,"Results");

%plot_Results(percent_outlier,Results);                                     % Plot the rank, time, absolute rotational error and average translational error


