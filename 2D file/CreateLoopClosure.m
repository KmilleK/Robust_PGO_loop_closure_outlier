function loop_closure =CreateLoopClosure(poses)
    % Create a loop closure measurements

    global n d w_r w_t n_lc p_out;

    lowerBound= -(n)^(1/3)/4;
    upperBound= (n)^(1/3)/4; 

    % Select the starting index randomly for the loop closure 
    i= randi([1, n]);

    % select the ending index from a close region of the starting one 
    threshold=int8(n/2);
    j=i;
    while j==i || j<=0 || j>n
        j=randi([i-threshold, i+threshold]);
    end
   

    %Choose between outlier and inliers
    if binornd(1,p_out)==0    %inliers loop closure
        R_noise=eul2rotm(w_r^2*randn(1,d));
        R_m_gt=poses(i).R'*poses(j).R*R_noise;
        t_m_gt=poses(i).R'*(poses(j).t-poses(i).t)+w_t^2*randn(d, 1);
        loop_closure=Measurement('LoopClosure',i,j,R_m_gt,t_m_gt);

    else           %outliers loop closure
        R_noise=eul2rotm(lowerBound*ones(1,d) + (upperBound - lowerBound) * rand(1,d));
        R_m_gt=poses(i).R'*poses(j).R*R_noise;
        t_noise= lowerBound*ones(d,1) + (upperBound - lowerBound) * rand(d,1);
        t_m_gt=poses(i).R'*(poses(j).t-poses(i).t)+t_noise;
        loop_closure=Measurement('LoopClosure',i,j,R_m_gt,t_m_gt);
    end
    
    


end