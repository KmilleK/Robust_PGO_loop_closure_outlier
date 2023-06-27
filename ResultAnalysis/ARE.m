function  error=ARE(GroundTruthPose,optimizedPose)
    % Compute the Absolute Rotation Error.
    
    % Checking Ground truth data and optimize data same size 
    if (length(GroundTruthPose)~=length(optimizedPose))
        error("data not same size in ARE computation");
    end 
    
    ARE_quat=0;
    ARE_mat=0;
    for i=1:length(GroundTruthPose)
        if size(optimizedPose(1).R,1)==1
            ARE_quat=ARE_quat+abs(wrapToPi(optimizedPose(i).R)-wrapToPi(GroundTruthPose(i).R));
        else
            ARE_quat=ARE_quat+computeRotationalErrorquat(optimizedPose(i).R,GroundTruthPose(i).R);
            ARE_mat=ARE_mat+computeRotationalErrorMat(optimizedPose(i).R,GroundTruthPose(i).R); 
        end
    end
    

    % disp('ARE quat:');
    % disp(ARE_quat);
    % disp('ARE mat:');
    % disp(ARE_mat);
    % 
    error=ARE_quat/length(GroundTruthPose);

  
end

function rotationalError = computeRotationalErrorMat(R1, R2)
    % R1, R2: 3x3 rotation matrices representing the orientations
    
    % Compute the matrix dot product
    dotProduct = trace(R1' * R2);

    % Take the absolute value of the dot product to avoid numerical issues
    dotProduct = abs(dotProduct);

    % Clamp the dot product value between -1 and 1
    dotProduct = min(max(dotProduct, -1), 1);

    % Compute the angular difference between the orientations
    rotationalError = acos(dotProduct); % Convert to degrees
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