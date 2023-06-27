function plot_Results(percent_outlier, Results)
    % percent_outlier is the vector with the percentage of outlier
    % Results is a cell matrix which contains the results of all the
    % computation on the form 
    % Kernel Function | ...   |  ...
    % Rank            | [...] |  [...]
    % Time            | [...] |  [...]
    % ARE            | [...] |  [...]
    % ATE            | [...] |  [...]

    plot_rank(percent_outlier,Results(1,:),Results(2,:));
    plot_time(percent_outlier,Results(1,:),Results(3,:));
    plot_ARE(percent_outlier,Results(1,:),Results(4,:));
    plot_ATE(percent_outlier,Results(1,:),Results(5,:));

end

function plot_rank(percent_outlier,kernel_name,Rank_data)
    % percent_outlier is the vector with the percentage of outlier
    % kernel name contains the info about the tested kernel name
    % Rank_data contains the rank data to be plot  
   
    
    % Number of kernel function 
    Nb_kernel=length(kernel_name)-1;
    Legend_labels=cell(1,Nb_kernel);
    % Create a figure
    figure;
    
    for i=2:Nb_kernel+1
        KernelName = kernel_name{i};
        [color,name]=KernelFunction.(KernelName).info_plot();
        plot(percent_outlier, Rank_data{i}, color);  % Blue line
        hold on;
        Legend_labels{1,i-1}= name;

    end

    % Add labels and title
    xlabel('Percentage of outlier');
    ylabel('Stable rank');
    title('Comparison of the stable rank of the rotation matrix');

    % Add a legend
    legend(Legend_labels);

    % Adjust the plot limits
    % xlim([min(out_pro), max(out_pro)]);
    % ylim([min([L1R, L2R,HR])-0.1, max([L1R, L2R,HR])+0.1]);

    % Turn on the grid
    grid on;

    % Hold off from further plotting
    hold off;

end 

function plot_time(percent_outlier,kernel_name,Time_data)
    % percent_outlier is the vector with the percentage of outlier
    % kernel name contains the info about the tested kernel name
    % Time_data contains the time data to be plot  
   
    
    % Number of kernel function 
    Nb_kernel=length(kernel_name)-1;
    Legend_labels=cell(1,Nb_kernel);
    % Create a figure
    figure;
    
    for i=2:Nb_kernel+1
        KernelName = kernel_name{i};
        [color,name]=KernelFunction.(KernelName).info_plot();
        plot(percent_outlier, Time_data{i}, color);  % Blue line
        hold on;
        Legend_labels{1,i-1}= name;

    end

    % Add labels and title
    xlabel('Percentage of outlier');
    ylabel('time (s)');
    title('Computation time in function of percentage of outlier');

    % Add a legend
    legend(Legend_labels);

    % Turn on the grid
    grid on;

    % Hold off from further plotting
    hold off;

end 

function plot_ARE(percent_outlier,kernel_name,ARE_data)
    % percent_outlier is the vector with the percentage of outlier
    % kernel name contains the info about the tested kernel name
    % ARE_data contains the absolute rotational error data to be plot  
   
    
    % Number of kernel function 
    Nb_kernel=length(kernel_name)-1;
    Legend_labels=cell(1,Nb_kernel);
    % Create a figure
    figure;
    
    for i=2:Nb_kernel+1
        KernelName = kernel_name{i};
        [color,name]=KernelFunction.(KernelName).info_plot();
        plot(percent_outlier, log(ARE_data{i}), color);  % Blue line
        hold on;
        Legend_labels{1,i-1}= name;

    end

    % Add labels and title
    xlabel('Percentage of outlier');
    ylabel('log of ARE (rad)');
    title('Absolute rotational error in function of percentage of outlier');

    % Add a legend
    legend(Legend_labels);

    % Turn on the grid
    grid on;

    % Hold off from further plotting
    hold off;

end 

function plot_ATE(percent_outlier,kernel_name,ATE_data)
    % percent_outlier is the vector with the percentage of outlier
    % kernel name contains the info about the tested kernel name
    % ATE_data contains the average translational error data to be plot  
   
    
    % Number of kernel function 
    Nb_kernel=length(kernel_name)-1;
    Legend_labels=cell(1,Nb_kernel);
    % Create a figure
    figure;
    
    for i=2:Nb_kernel+1
        KernelName = kernel_name{i};
        [color,name]=KernelFunction.(KernelName).info_plot();
        plot(percent_outlier, log(ATE_data{i}), color);  % Blue line
        hold on;
        Legend_labels{1,i-1}= name;

    end

    % Add labels and title
    xlabel('Percentage of outlier');
    ylabel('log of ATE (m)');
    title('Average translation error in function of percentage of outlier');

    % Add a legend
    legend(Legend_labels);

    % Turn on the grid
    grid on;

    % Hold off from further plotting
    hold off;

end 




