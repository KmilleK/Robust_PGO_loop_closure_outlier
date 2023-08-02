function  error=ARE(GroundTruthPose,optimizedPose)
    % Compute the Absolute Rotation Error.
    
    % Checking Ground truth data and optimize data same size 
    if (length(GroundTruthPose)~=length(optimizedPose))
        error("data not same size in ARE computation");
    end 
    
    % Putting the first rotation to the identity and rotate all other

    R_0g=GroundTruthPose(1).R;
    R_1g=optimizedPose(1).R;
    ARE_matrix=0;
    ARE_angle=0;

    for i=1:length(GroundTruthPose)
            ARE_matrix=ARE_matrix+norm((R_0g'*GroundTruthPose(i).R)'*R_1g'*optimizedPose(i).R-ones(2),"fro");
            ARE_angle=ARE_angle+abs(so2(R_1g'*optimizedPose(i).R).theta-so2(R_0g'*GroundTruthPose(i).R).theta);
    end
    

    error=ARE_angle/length(GroundTruthPose);


    % disp('ARE matrix:');
    % disp(ARE_matrix/length(GroundTruthPose));
    % disp('ARE angle:');
    % disp(ARE_angle/length(GroundTruthPose));
    % disp('log(ARE):');
    % disp(log(error));

  
end
