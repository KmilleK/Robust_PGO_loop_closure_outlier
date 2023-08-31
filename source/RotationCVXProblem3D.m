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
    
    elapsedTimeRotation = toc;

   stable_rank=(norm(X, 'fro')/norm(X))^2;
    % disp(['Stable rank of X :',num2str(stable_rank)]);

    
    tic    % Start the timer
    R_rounded=RoundingProcedure(X,2,n); 
    
    elapsedTimeRounding = toc;

    %Poses information storage 
    poses= Pose3D.empty(n,0);

    R_rounded=RoundingProcedure(X,d,n);

    for i=1:n
    
        poses(i)=Pose3D(i,R_rounded(d*i-(d-1):d*i,:),zeros(3,1));
       
    end
    
    elapsedTimeRotation = toc;
    
    varargout{1} = poses;
    varargout{2} = stable_rank;
    varargout{3} = relaxation_gap(R_rounded,X,measurements,robust_loss_type,w_r);
    varargout{4} = elapsedTimeRotation;
    varargout{5} = elapsedTimeRounding;

end

function X=cvx_solver_separate(measurements,robust_loss_type,n,d,w_r)

    %Silent cvx solver 
    

    cvx_clear
    cvx_quiet(true);

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
                        objective=objective+(w_r/2)*norm(error,'fro');
                    case KernelFunction.L1
                        objective=objective+(w_r/2)*norm(error,1);
                    case KernelFunction.L2
                        objective=objective+power(2,(w_r/2)*norm(error,'fro'));
                    case KernelFunction.Huber
                        objective=objective+huber_fro(error,'fro',1/w_r);                       
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
    %clean method 
   X_2=U(:, 1:d) * S(1:d, 1:d) * V(:, 1:d)';
   Ud=X_2(:,1:d);
 
  %projection onto SO(d) 

  R=zeros(n*d,d);


  for i=1:n
      T_i=Ud(d*i-(d-1):d*i,:);         % Approximation of the Rotation of pose i 
      
      [Ui, ~, Vi] = svd(T_i);      % SVD of the estimate

      Ri = Ui * Vi';
        
      if det(Ri) < 0
              detChan=eye(d);
              detChan(d,d)=-1;
              Ri= detChan*Ri;
      end

      R(d*i-(d-1):d*i,:)=transpose(Ri);

    
  end 

 



end 

function gap=relaxation_gap(R_i,X,measurements,robust_loss_type,w_r)

    %compute optimal value of problem 1

    f1=0;
    f5=0;

    for i=1:length(measurements)
            error1=R_i(3*measurements(i).i-(3-1):3*measurements(i).i,:)'*R_i(3*measurements(i).j-(3-1):3*measurements(i).j,:)-measurements(i).R;
            error5=X(3*measurements(i).i-(3-1):3*measurements(i).i,3*measurements(i).j-(3-1):3*measurements(i).j)-measurements(i).R;
            switch  robust_loss_type
                case KernelFunction.Identity
                    f1=f1+(w_r/2)*norm(error1,'fro');
                    f5=f5+(w_r/2)*norm(error5,'fro');
                case KernelFunction.L1
                    f1=f1+(w_r/2)*norm(error1,1);
                    f5=f5+(w_r/2)*norm(error5,1);
                case KernelFunction.L2
                    f1=f1+power(2,(w_r/2)*norm(error1,'fro'));
                    f5=f5+power(2,(w_r/2)*norm(error5,'fro'));                   
                case KernelFunction.Huber
                    f1=f1+huber_froRe(error1,1/w_r);
                    f5=f5+huber_froRe(error5,1/w_r);
                otherwise
                    disp('wrong robust loss function type');
            end
    end


    gap=f1-f5;

end

function y=huber_froRe(e,s)
    nm=norm(e,'fro');
    if abs(nm)<=s
        y=(nm^2)/2;
    else 
        y=s*(abs(nm)-s/2);
    end

end
