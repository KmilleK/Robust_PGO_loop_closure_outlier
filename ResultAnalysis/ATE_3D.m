function  error=ATE_3D(GroundTruthPose,optimizedPose)
    % Compute the Absolute Trajectory Error.
    
    % Checking Ground truth data and optimize data same size 
    if (length(GroundTruthPose)~=length(optimizedPose))
        error("data not same size in ARE computation");
    end 
    
    R_0g=GroundTruthPose(1).R;
    R_1g=optimizedPose(1).R;

    t_0g= R_0g'*GroundTruthPose(1).t;
    t_1g= R_1g*optimizedPose(1).t;

    ATE=0;
    for i=1:length(GroundTruthPose)          
        GT_pt=R_0g'*GroundTruthPose(i).t-t_0g;
        OP_pt=R_1g'*optimizedPose(i).t-t_1g;
        ATE =ATE + (GT_pt-OP_pt)'*(GT_pt-OP_pt); 
    end

    error=ATE/length(GroundTruthPose);

    % disp('ATE:');
    % disp(error);

  
end