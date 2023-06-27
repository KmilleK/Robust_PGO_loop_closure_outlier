function Infected_measurements = Update_measurements(varargin)
%%%% Update_measurement adds outlier loop closure edge to the measurements
%
% Infected_measurements : array of Measurement3D data with outliers  
%
%  Update_measurements(poses,measurements,p_out,n) return data with the wanted type and 
%  default parameters for synthetic datasets 
%   poses: ground truth pose of the dataset
%   measurements: actual measurement data
%   p_out: percentage of outlier in the loop closure set  

% Update_measurements(poses,measurements,p_out,n,n_lc) return data with the wanted type and 
%  number of loop closure to be n_lc for synthetic datasets 
%   n_lc: number of loop closure 
%
%%%% Author: KESSELER CAMILLE 

% Checking input of the function and initialize the parameters if needed 
    numArgs=nargin;
    if numArgs==3 
        poses=varargin{1};
        measurements=varargin{2};                        
        p_out=varargin{3};                         
        n_lc=count_loop_closure(measurements);                       
    elseif numArgs==5
        poses=varargin{1};
        measurements=varargin{2};                        
        p_out=varargin{3};                         
        n_lc=varargin{4};
    else
        error('Update measurement function do not have the correct number of input');
    end
    
    n=length(poses);

    % Number of additional measurement to add in function of all
    % measurement 
       
    n_add=int32(p_out*n_lc);

    d=3;
    lowerBound= -(n)^(1/3)/4;
    upperBound= (n)^(1/3)/4; 

    % Copy the inliers measurements 
    %Infected_measurements= Measurement.empty(length(measurements)+n_add,0);
    %Infected_measurements(1:length(measurements))=measurements;
    
    Infected_measurements=[measurements,Measurement.empty(n_add)];
    
    %Create the outliers measurements
    
    for i=n+n_lc:length(measurements)+n_add                                                       % vector of measurement for loop closure        
        
        % Index selection
        ini=0;
        DoubleLoopClosure=true;
        while ini<=0 || DoubleLoopClosure
            ini= randi([1, n]);
            DoubleLoopClosure=false;
            for j=n+1:i-1
                if Infected_measurements(j).i==ini
                    DoubleLoopClosure=true;
                end
            end
        end

        fin=0;
        while fin==ini || fin<=0 || fin>n
            fin=randi([1,n]);
        end

        % loop closure noise creation
        R_noise=eul2rotm(lowerBound*ones(1,d) + (upperBound - lowerBound) * rand(1,d));
        R_m_gt=poses(ini).R'*poses(fin).R*R_noise;
        t_noise= lowerBound*ones(d,1) + (upperBound - lowerBound) * rand(d,1);
        t_m_gt=poses(ini).R'*(poses(fin).t-poses(ini).t)+t_noise;
        loop_closure=Measurement('LoopClosure',ini,fin,R_m_gt,t_m_gt);
        Infected_measurements(i)=loop_closure;
    end
    

end

function n_lc=count_loop_closure(measurements)

    n_lc=0;
    for i=1:length(measurements)
        if strcmpi(strtrim(measurements(i).type),'LoopClosure')
            n_lc=n_lc+1;
        end 
    end

end 