function plot_Results_3D(percent_outlier, Results)
    % percent_outlier is the vector with the percentage of outlier
    % Results is a cell matrix which contains the results of all the
    % computation on the form 
    % Kernel Function | ...   |  ...
    % Rank            | [...] |  [...]
    % Time            | [...] |  [...]
    % ARE            | [...] |  [...]
    % ATE            | [...] |  [...]

    %plot_matrix_full_info(percent_outlier, Results);

    % for indicator=2:length(Results(:,1))
    %     plot_comparison_info(Results{indicator,1},percent_outlier,Results(1,:),Results(indicator,:))
    % end 

    plot_valid_pose(percent_outlier, Results);

end

function plot_valid_pose(percent_outlier, Results)
    
 figure('Name','Valid poses');
    y = [2 2 3; 2 5 6; 2 8 9; 2 11 12];

    kernel_name=Results(1,:);
    Nb_kernel=length(kernel_name)-1;

    y =zeros(length(percent_outlier),Nb_kernel);

    for i=1:Nb_kernel
        for j=1:length(percent_outlier)
            sum=0;
            Rank_mat=Results{2,i+1};
            for k=1:size(Rank_mat,1)
                if Rank_mat(k,j)>2.8 && Rank_mat(k,j)<3.2
                    sum=sum+1;
                end 
            end 
            y(j,i)=sum;
        end
    end 
    h=bar(percent_outlier',y);
    
    % set 3 display names for the 3 handles
    set(h, {'DisplayName'}, {'Identity';'L1';'L2';'Huber'})
    xlabel('Percentage of outliers')
    ylabel('Number of validated solution')



    % Legend will show names for each color
    legend() 

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
   
    subplot1=subplot(2,2,1);
    h=boxplotGroup(data(2,2:Nb_kernel+1),'primaryLabels',labels,'SecondaryLabels',grpLabels,'Colors',[0 .5 0; 0 0 1; .9 0 .9; 1 0 0],'GroupType','betweenGroups','interGroupSpace',2);
    

    ylabel('Stable rank')
    dmax= getMaxValueFromCellArray(data(2,2:Nb_kernel+1));
    dmin=getMinValueFromCellArray(data(2,2:Nb_kernel+1));
    ylim(subplot1,[dmin-0.01*dmin dmax+0.01*dmax]);
    subtitle('Percentage of outlier')

    h.axis.XTickLabelRotation = 90; 
    h.axis.XAxis.FontName = 'fixedwidth';
    h.axis.XAxis.FontWeight = 'bold';
      
    subplot2=subplot(2,2,2);
    j=boxplotGroup(data(3,2:Nb_kernel+1),'primaryLabels',labels,'SecondaryLabels',grpLabels,'Colors',[0 .5 0; 0 0 1; .9 0 .9; 1 0 0],'GroupType','betweenGroups','interGroupSpace',2);
    ylabel('Relaxation gap')
    dmax= getMaxValueFromCellArray(data(3,2:Nb_kernel+1));
    dmin=getMinValueFromCellArray(data(3,2:Nb_kernel+1));
    ylim(subplot2,[dmin-0.001*dmin dmax+0.01*dmax]);
    subtitle('Percentage of outlier')

    j.axis.XTickLabelRotation = 90; 
    j.axis.XAxis.FontName = 'fixedwidth';
    j.axis.XAxis.FontWeight = 'bold';
  
    subplot3=subplot(2,2,3);
    a=boxplotGroup(data(5,2:Nb_kernel+1),'primaryLabels',labels,'SecondaryLabels',grpLabels,'Colors',[0 .5 0; 0 0 1; .9 0 .9; 1 0 0],'GroupType','betweenGroups','interGroupSpace',2);
    ylabel('ATE [m]')
    dmax= getMaxValueFromCellArray(data(5,2:Nb_kernel+1));
    dmin=getMinValueFromCellArray(data(5,2:Nb_kernel+1));
    ylim(subplot3,[dmin-0.01*dmin dmax+0.01*dmax]);
    subtitle('Percentage of outlier')

    a.axis.XTickLabelRotation = 90; 
    a.axis.XAxis.FontName = 'fixedwidth';
    a.axis.XAxis.FontWeight = 'bold';
 
    subplot4=subplot(2,2,4);
    z=boxplotGroup(data(4,2:Nb_kernel+1),'primaryLabels',labels,'SecondaryLabels',grpLabels,'Colors',[0 .5 0; 0 0 1; .9 0 .9; 1 0 0],'GroupType','betweenGroups','interGroupSpace',2);
    ylabel('ARE [rad]')
    dmax= getMaxValueFromCellArray(data(4,2:Nb_kernel+1));
    dmin=getMinValueFromCellArray(data(4,2:Nb_kernel+1));
    ylim(subplot4,[dmin-0.01*dmin dmax+0.01*dmax]);
    subtitle('Percentage of outlier')

    z.axis.XTickLabelRotation = 90; 
    z.axis.XAxis.FontName = 'fixedwidth';
    z.axis.XAxis.FontWeight = 'bold';

    fig_nam='Details_Monte_carlo.fig';
    filename = fullfile(pwd, 'Results','current',fig_nam );
    savefig(gcf,filename);
  

end 

function maxVal = getMaxValueFromCellArray(cellArray)
    %Powered by ChatGPT
    % Check if the input is a cell array
    if ~iscell(cellArray)
        error('Input must be a cell array.');
    end
    
    % Initialize the maxVal to a very small number
    maxVal = -inf;
    
    % Loop through each matrix in the cell array
    for i = 1:numel(cellArray)
        % Check if the element is a matrix
        if isnumeric(cellArray{i})
            % Find the maximum value in the current matrix
            currentMax = max(cellArray{i}(:));
            
            % Update the overall maxVal if the currentMax is greater
            if currentMax > maxVal
                maxVal = currentMax;
            end
        else
            % Skip non-matrix elements
            continue;
        end
    end
end

function minVal = getMinValueFromCellArray(cellArray)
    % Check if the input is a cell array
    if ~iscell(cellArray)
        error('Input must be a cell array.');
    end
    
    % Initialize the minVal to a very large number
    minVal = inf;
    
    % Loop through each matrix in the cell array
    for i = 1:numel(cellArray)
        % Check if the element is a matrix
        if isnumeric(cellArray{i})
            % Find the minimum value in the current matrix
            currentMin = min(cellArray{i}(:));
            
            % Update the overall minVal if the currentMin is smaller
            if currentMin < minVal
                minVal = currentMin;
            end
        else
            % Skip non-matrix elements
            continue;
        end
    end
end










