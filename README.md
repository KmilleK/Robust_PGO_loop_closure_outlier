# Robust_PGO_loop_closure_outlier

1) Cvx installation procedure 

- Change directories to the top of the CVX distribution, and run the cvx_setup command. 

	cd C:\personal\cvx
	cvx_setup

- Change the solver: the free one are SDPT3 and SeDuMi. You can get a academic MOSEK license online if you want 

	cvx_solver SDPT3
	cvx_save_prefs

2) Change the folderPath and the excludeFolder in the main script to fit your actual file localisation.

	folderPath = 'D:\document\MATLAB\24_6'; 
	subfolders_full = genpath(folderPath); 

	excludedFolder = 'D:\document\MATLAB\24_6\cvx';  % Specify the folder to exclude
	excludedSubfolders = genpath(excludedFolder);  
