function  error=ARE_3D(GroundTruthPose,optimizedPose)
    % Compute the Absolute Rotation Error.
    
    % Checking Ground truth data and optimize data same size 
    if (length(GroundTruthPose)~=length(optimizedPose))
        error("data not same size in ARE computation");
    end 
    
    ARE_quat=0;
    
    % Putting the first rotation to the identity and rotate all other
    R_0g=GroundTruthPose(1).R;
    R_1g=optimizedPose(1).R;


    for i=1:length(GroundTruthPose)
          ARE_quat=ARE_quat+computeRotationalErrorquat(R_1g'*optimizedPose(i).R,R_0g'*GroundTruthPose(i).R);
    end
    

    % disp('ARE quat:');
    % disp(ARE_quat);

    error=ARE_quat/length(GroundTruthPose);

  
end



function rotationalError = computeRotationalErrorquat(R1, R2)
    % R1, R2: 3x3 rotation matrices representing the orientations
    
    % Convert rotation matrices to quaternions
    q1 = quatnormalize(rotm2quat(R1));
    q2 = quatnormalize(rotm2quat(R2));

    % Compute the quaternion dot product
    dotProduct = dot(q1, q2);

    % Take the absolute value of the dot product to avoid numerical issues
    dotProduct = abs(dotProduct);

    % Compute the angular difference between the orientations
    rotationalError = 2 * acos(dotProduct) ; 
end