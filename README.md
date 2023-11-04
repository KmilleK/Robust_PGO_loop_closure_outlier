<h1 align="center">Robust_PGO_loop_closure_outlier</h1>


Code associated with the master thesis: robust pose graph optimization with loop closure outliers. 
It tests different robust cost functions applied to the edge measurement of a pose graph.


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">TO DO</a></li>
  </ol>
</details>

---
## About the project 

This project is the code associated with a graduated thesis, for the pose graph optimization 2 main choices have been made: 
- convex relaxation of the initial problem in order to obtain a global solution (estimation contracts on the quality of the solution)
- loss functions apply to the measurement error terms to mitigate the effect of outliers in the pose graph.

### Built with

Matlab and its toolbox (if the toolbox are not installed on your computer, when running you will be prompted to install it). The optimization solver used is cvx. 

## Getting started

### Installation

1) Cvx installation procedure (reference: http://cvxr.com/cvx/doc/install.html)

- Retrieve cvx latest version on the website: http://cvxr.com/cvx/download/
- Unpack the cvx directory in this folder
- Change directories to the top of the CVX distribution, and run the cvx_setup command. 
	cd C:\personal\cvx
	cvx_setup
- Change the solver: the free ones are SDPT3 and SeDuMi. You can get an academic MOSEK license online if you want 
	cvx_solver SDPT3
	cvx_save_prefs
- Add additional function (huber_fro) loss function into cvx function folder (\cvx\functions\@cvx)

 
2) Change the folderPath and the exclude folder in the main script to fit your actual file localization.

	folderPath = 'D:\document\MATLAB\24_6'; 
	subfolders_full = genpath(folderPath); 

	excludedFolder = 'D:\document\MATLAB\24_6\cvx';  % Specify the folder to exclude
	excludedSubfolders = genpath(excludedFolder); 

## Usage 

You can just run the Outlier_Rate_analysis.m file to compare the impact of the outlier rate on the different datasets. 
