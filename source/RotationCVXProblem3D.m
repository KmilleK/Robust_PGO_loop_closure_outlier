function  varargout =RotationCVXProblem3D(measurements,robust_loss_type,n,w_r)
    % Perform the necessary computation or transformation
    % to derive the measurement vector from the pose vector.

    %Silent cvx solver 
    cvx_quiet(true);
    % Start the timer
    tic
    
    cvx_clear
    %global n d w_r w_t;
    d=3;                                                                   % 3D problem 
    cvx_begin sdp
    
        variable X(d*n,d*n) symmetric;
        objective=0;
        for i=1:length(measurements)

            if strcmpi(strtrim(measurements(i).type), 'Odometry')
                objective=objective+(w_r/2)*sum(sum_square_abs(X(d*measurements(i).i-(d-1):d*measurements(i).i,d*measurements(i).j-(d-1):d*measurements(i).j)-measurements(i).R));
                %objective=objective+(w_r/2)*norm(X(d*measurements(i).i-(d-1):d*measurements(i).i,d*measurements(i).j-(d-1):d*measurements(i).j)-measurements(i).R,'fro');
            elseif strcmpi(strtrim(measurements(i).type), 'LoopClosure')
                error=X(d*measurements(i).i-(d-1):d*measurements(i).i,d*measurements(i).j-(d-1):d*measurements(i).j)-measurements(i).R;
                switch  robust_loss_type
                    case KernelFunction.Identity
                        objective=objective+(w_r/2)*sum(sum_square_abs(X(d*measurements(i).i-(d-1):d*measurements(i).i,d*measurements(i).j-(d-1):d*measurements(i).j)-measurements(i).R));
                    case KernelFunction.L1
                        objective=objective+(w_r/2)*norm(error,1);
                    case KernelFunction.L2
                        objective=objective+(w_r/2)*norm(error,'fro');
                    case KernelFunction.Huber
                        objective=objective+sum(huber_circ(error,[],1/w_r));
                        %objective=objective+huber((w_r/2)*norm(error,'fro'));
                    %case KernelFunction.Charbonnier
                     %   objective=objective+sqrt((w_r/2)^2*sum(sum_square_abs(error))+1)-1;                         
                    otherwise
                        disp('wrong robust loss function type');
                end
            else
                disp('wrong type of edge');

            end
        end
        minimize (objective);
        X >= 0;
        for k=1:n 
           X(d*k-(d-1):d*k,d*k-(d-1):d*k)==eye(d);
        end
        
    cvx_end


%stable rank of solution check

% 
% disp(['Frobenius norm of  X :',num2str(norm(X, 'fro'))]);
% disp(['Spectral norm of X :',num2str(norm(X,2))]);


stable_rank=(norm(X, 'fro')/norm(X,2))^2;
% disp(['Stable rank of X :',num2str(stable_rank)]);


%Poses information storage 
poses= Pose.empty(n,0);

R_rounded=RoundingProcedure(X,d,n);

for i=1:n

    poses(i)=Pose(i,R_rounded(d*i-(d-1):d*i,:));
   
end

elapsedTimeRotation = toc;

varargout{1} = poses;
varargout{2} = stable_rank;
varargout{3} = elapsedTimeRotation;


end