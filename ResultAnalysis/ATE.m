function  error=ATE(GroundTruthPose,optimizedPose)
    % Compute the Absolute Trajectory Error.
    
    % Checking Ground truth data and optimize data same size 
    if (length(GroundTruthPose)~=length(optimizedPose))
        error("data not same size in ARE computation");
    end 

    ATE=0;

    for i=1:length(GroundTruthPose)       
        ATE =ATE + (GroundTruthPose(i).t -optimizedPose(i).t)'*(GroundTruthPose(i).t -optimizedPose(i).t); 

    end

    error=ATE/length(GroundTruthPose);

  
end