function  varargout =RotationCVXProblem3D(measurements,edge_separation,robust_loss_type,n,w_r)
    % Perform the necessary computation or transformation
    % to derive the measurement vector from the pose vector.

    d=3;

    % Start the timer
    tic
    
    if edge_separation
        X=cvx_solver_separate(measurements,robust_loss_type,n,d,w_r);
    else 
        X=cvx_solver_join(measurements,robust_loss_type,n,d,w_r);
    end
  
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

function X=cvx_solver_separate(measurements,robust_loss_type,n,d,w_r)

    %Silent cvx solver 
    cvx_quiet(true);

    cvx_clear

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


end

function X=cvx_solver_join(measurements,robust_loss_type,n,d,w_r)
    
    %Silent cvx solver 
    cvx_quiet(true);

    cvx_clear

    cvx_begin sdp
    
        variable X(d*n,d*n) symmetric;
        objective=0;
        for i=1:length(measurements)
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
                otherwise
                    disp('wrong robust loss function type');
            end
            
        end
        minimize (objective);
        X >= 0;
        for k=1:n 
           X(d*k-(d-1):d*k,d*k-(d-1):d*k)==eye(d);
        end
        
    cvx_end

end

function  R=RoundingProcedure(X,d,n)
    % Perform the necessary computation to obtain the rotation matrix
    % estimate from X the solution of the convex relaxation problem
    
    
  
    % Rank d approximation of the X matrix via SVD 
  [U, S, V] = svd(X);
  Ud = U(:, 1:d);

  %projection onto SO(d) 

  R_proj=zeros(n*d,d);
  Rot_Final=zeros(1,n);
  R_anchor=zeros(n*d,n*d);

  for i=1:n
      T_i=Ud(d*i-(d-1):d*i,:);         % Approximation of the Rotation of pose i 
      
      [Ui, ~, Vi] = svd(T_i);      % SVD of the estimate

      Ri = Ui * Vi';

      if det(Ri) < 0
            Ri(:, end) = -Ri(:, end);
      end

      R_proj(d*i-(d-1):d*i,:)=transpose(Ri);

      if i==1
        R_save=Ri;
      end

      R_anchor(d*i-(d-1):d*i,d*i-(d-1):d*i)=R_save;

    
  end 

  % Anchoring to the first rotation as the result is define at a constant
  if d==2
      for j=1:n
          Rot_Final(j)=atan2(R_proj(2*j,1), R_proj(2*j-1,1));
      end
      
      R=Rot_Final-Rot_Final(1);
  elseif d==3
      
      R=R_anchor*R_proj;
       
  end 




end 
