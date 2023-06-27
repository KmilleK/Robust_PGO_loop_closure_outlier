function measurements =PoseToMeasurement(poses,noiseActivation,LoopClosureActivation)
    % Perform the necessary computation or transformation
    % to derive the measurement vector from the pose vector.
    
    global n d w_r w_t;

    % Ground truth measurement edges: 
    if (LoopClosureActivation)
        measurements= Measurement.empty(n,0);
    else
        measurements= Measurement.empty(n-1,0);
    end

    if (noiseActivation)  % measurement with noise 

        % vector of measurement for odometry
        for i = 1:n-1
            R_m_gt=poses(i+1).R-poses(i).R+w_r^2*randn(1);
            t_m_gt=transpose(poses(i).generateRotationMatrix())*(poses(i+1).t-poses(i).t)+w_t^2*randn(2, 1);     
            measurements(i)=Measurement('Odometry',i,i+1,R_m_gt,t_m_gt);
        end
        if (LoopClosureActivation)

            % vector of measurement for loop closure 
            % To do more complexe set of data latter with wrong outlier
            % rate
            R_m_gt=poses(n).R-poses(1).R+w_r^2*randn(1);
            %R_m_gt=4.5;
            t_m_gt=transpose(poses(1).generateRotationMatrix())*(poses(n).t-poses(1).t)+w_t^2*randn(2, 1);      
           
            measurements(n)=Measurement('LoopClosure',1,n,R_m_gt,t_m_gt);

        end 

    else       % no noise in the measurement vector just checking if code ok 
        % vector of measurement for odometry
        for i = 1:n-1
            R_m_gt=poses(i+1).R-poses(i).R;
            t_m_gt=transpose(poses(i).generateRotationMatrix())*(poses(i+1).t-poses(i).t);      
            measurements(i)=Measurement('Odometry',i,i+1,R_m_gt,t_m_gt);
        end
        if (LoopClosureActivation)
            % vector of measurement for loop closure 
            % To do more complexe set of data latter with wrong outlier
            % rate
            R_m_gt=poses(n).R-poses(1).R;     
            %R_m_gt=4.5;
            t_m_gt=transpose(poses(1).generateRotationMatrix())*(poses(n).t-poses(1).t);
            measurements(n)=Measurement('LoopClosure',1,n,R_m_gt,t_m_gt);
            %display(measurements(n).generateRotationMatrix())

        end 
    
    end
        


end