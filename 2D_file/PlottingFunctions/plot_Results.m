function plot_Results(percent_outlier, Results)
    % percent_outlier is the vector with the percentage of outlier
    % Results is a cell matrix which contains the results of all the
    % computation on the form 
    % Kernel Function | ...   |  ...
    % Rank            | [...] |  [...]
    % Relaxation gap  | [...] |  [...]
    % ARE             | [...] |  [...]
    % ATE             | [...] |  [...]
    

    plot_matrix_full_info(percent_outlier, Results);

    for indicator=2:length(Results(:,1))
        plot_comparison_info(Results{indicator,1},percent_outlier,Results(1,:),Results(indicator,:))
    end 


    
end


function plot_comparison_info(indicator_name,p_out,kernel_name,data)

   

   
    % Number of kernel function 
    Nb_kernel=length(kernel_name)-1;
    Legend_labels=cell(1,Nb_kernel);

     %Compute the mean of the data matrix 
    data_mean=zeros(1,length(p_out));

    % Create a figure
    figure('Name',indicator_name);
    
     for i=2:Nb_kernel+1
        for j=1:length(p_out)
            data_mean(j)=mean(data{i}(:,j));
        end

        KernelName = kernel_name{i};
        [color,name]=KernelFunction.(KernelName).info_plot();
        plot(p_out, data_mean, color);  
        hold on;
        Legend_labels{1,i-1}= name;

     end

    % Add labels and title
    xlabel('Percentage of outlier');
    ylabel(indicator_name);
    %title('Comparison of the stable rank of the rotation matrix');

    % Add a legend
    legend(Legend_labels);
    % Turn on the grid
    grid on;
    fig_nam=strcat(indicator_name ,'.fig');
    filename = fullfile(pwd, 'Results','current',fig_nam );
    savefig(gcf,filename);


    % Hold off from further plotting7
    hold off;


end


function plot_matrix_full_info(percent_outlier,data)
    
    kernel_name=data(1,:);
    Nb_kernel=length(kernel_name)-1;
    labels=cell(1,Nb_kernel);
    for i=2:Nb_kernel+1

          KernelName = kernel_name{i};
          [color,name]=KernelFunction.(KernelName).info_plot();
          labels{1,i-1}= name;

    end
    
    grpLabels = cell(1,length(percent_outlier));
    for i=1:length(percent_outlier)
         grpLabels{1,i}=num2str(percent_outlier(i));
    end
    

    figure('Name','Details of All run');
   
    subplot(2,2,1)
    boxplotGroup(data(2,2:Nb_kernel+1),'primaryLabels',labels,'SecondaryLabels',grpLabels,'Colors',[0 .5 0; 0 0 1; .9 0 .9; 1 0 0],'GroupType','betweenGroups');
    xlabel('Percentage of outlier')
    ylabel('Stable rank')
    subtitle('Stable rank')
  
    subplot(2,2,2)
    boxplotGroup(data(3,2:Nb_kernel+1),'primaryLabels',labels,'SecondaryLabels',grpLabels,'Colors',[0 .5 0; 0 0 1; .9 0 .9; 1 0 0],'GroupType','betweenGroups');
    xlabel('Percentage of outlier')
    ylabel('Relaxation gap')
    subtitle('Relaxation gap')
  
    subplot(2,2,3)
    boxplotGroup(data(5,2:Nb_kernel+1),'primaryLabels',labels,'SecondaryLabels',grpLabels,'Colors',[0 .5 0; 0 0 1; .9 0 .9; 1 0 0],'GroupType','betweenGroups');
    xlabel('Percentage of outlier')
    ylabel('ATE [m]')
    subtitle('Translational pose error')
 
    subplot(2,2,4)
    boxplotGroup(data(4,2:Nb_kernel+1),'primaryLabels',labels,'SecondaryLabels',grpLabels,'Colors',[0 .5 0; 0 0 1; .9 0 .9; 1 0 0],'GroupType','betweenGroups');
    xlabel('Percentage of outlier')
    ylabel('ARE [rad]')
    subtitle('Rotational pose error')

    fig_nam='Details_Monte_carlo.fig';
    filename = fullfile(pwd, 'Results','current',fig_nam );
    savefig(gcf,filename);
  

end 





