function  varargout=RotationCVXProblem(measurements)
    % Perform the necessary computation or transformation
    % to derive the measurement vector from the pose vector.
    
    % Start the timer
    tic

    cvx_clear
    global n d w_r w_t;
    
    cvx_begin sdp
    
        variable X(2*n,2*n) symmetric;
        objective=0;
        for i=1:length(measurements)
            % display(measurements(i).i)
            % display(measurements(i).j)
            % display(measurements(i).generateRotationMatrix())
            if strcmpi(strtrim(measurements(i).type), 'Odometry')

                objective=objective+(w_r/2)*sum(sum_square_abs(X(2*measurements(i).i-1:2*measurements(i).i,2*measurements(i).j-1:2*measurements(i).j)-measurements(i).generateRotationMatrix()));
            elseif strcmpi(strtrim(measurements(i).type), 'LoopClosure')
                objective=objective+(w_r/2)*sum(sum_square_abs(X(2*measurements(i).i-1:2*measurements(i).i,2*measurements(i).j-1:2*measurements(i).j)-measurements(i).generateRotationMatrix()));

            else

            end
        end
        minimize (objective)
        X >= 0;
        for k=1:n 
           X(2*k-1:2*k,2*k-1:2*k)==eye(2);
        end
        
    cvx_end


%stable rank of solution check

%disp(['rank of X solutions:',num2str(rank(X))]);

stable_rank=(norm(X, 'fro')/max(svd(X)))^2;
%disp(['stable rank of X solutions:',num2str(stable_rank)]);

R_anch=RoundingProcedure(X); 

%Poses information storage 
poses= Pose.empty(n,0);

for i=1:n
    poses(i)=Pose(i,R_anch(i));
end
% Stop the timer and display the elapsed time
elapsedTimeRotation = toc;
%save('result.mat', 'elapsedTimeRotation','-append');
% Save the workspace data
save('workspace_rotation.mat');

varargout{1} = poses;
varargout{2} = stable_rank;
varargout{3} = elapsedTimeRotation;

end