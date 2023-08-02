function  varargout=RotationCVXProblem2D(measurements,robust_loss_type,n,w_r)
    % Perform the necessary computation or transformation
    % to derive the measurement vector from the pose vector.
   
    cvx_clear
    cvx_quiet(true);


    tic    % Start the timer
    
    cvx_begin sdp
    
        variable X(2*n,2*n) symmetric;
        objective=0;
        for i=1:length(measurements)
            error=X(2*measurements(i).i-(2-1):2*measurements(i).i,2*measurements(i).j-(2-1):2*measurements(i).j)-measurements(i).R;
            switch  robust_loss_type
                case KernelFunction.Identity
                    objective=objective+(w_r/2)*power(2,norm(error,'fro'));
                case KernelFunction.L1
                    objective=objective+(w_r/2)*norm(error,1);
                case KernelFunction.L2
                    objective=objective+(w_r/sqrt(2))*norm(error,'fro');                   
                case KernelFunction.Huber
                    %objective=objective+sum(huber_circ(error,[],1/w_r));
                    objective=objective+huber_fro(error,'fro',1/w_r);
                otherwise
                    disp('wrong robust loss function type');
            end
        end
        minimize (objective)
        X >= 0;
        for k=1:n 
           X(2*k-1:2*k,2*k-1:2*k)==eye(2);
        end
        
    cvx_end

elapsedTimeRotation = toc;

%stable rank of solution check

% disp(['direct rank of X solutions:',num2str(max(X'*X))]);
% disp(['norm rank of X solutions:',num2str(norm(X))]);
% 

stable_rank=(norm(X, 'fro')/norm(X))^2;
% disp(['stable rank of X solutions:',num2str(stable_rank)]);

tic    % Start the timer
R_rounded=RoundingProcedure(X,2,n); 

elapsedTimeRounding = toc;

%Poses information storage 
poses= Pose.empty(n,0);

for i=1:n
    poses(i)=Pose(i,R_rounded(2*i-(2-1):2*i,:));
end

varargout{1} = poses;
varargout{2} = stable_rank;
varargout{3} = relaxation_gap(R_rounded,X,measurements,robust_loss_type,w_r);
varargout{4} = elapsedTimeRotation;
varargout{5} = elapsedTimeRounding;

end


function  R=RoundingProcedure(X,d,n)
    % Perform the necessary computation to obtain the rotation matrix
    % estimate from X the solution of the convex relaxation problem
    
    
      % Rank d approximation of the X matrix via SVD  
      [U, S, V] = svd(X);
      %basic method
      Ud = U(:, 1:d);
      %clean method 
      X_2=U(:, 1:d) * S(1:d, 1:d) * V(:, 1:d)';
      Ud_2=X_2(:,1:d);

      %projection onto SO(d) 

      R=zeros(n*d,d);
      % Rot_Final=zeros(1,n);
      % R_anchor=zeros(n*d,n*d);

      for i=1:n
          T_i=Ud_2(d*i-(d-1):d*i,:);         % Approximation of the Rotation of pose i 

          [Ui, ~, Vi] = svd(T_i);      % SVD of the estimate
            
          Ri = Ui * Vi';

          if det(Ri) < 0
              detChan=eye(d);
              detChan(d,d)=-1;
              Ri= detChan*Ri;
          end

          R(d*i-(d-1):d*i,:)=transpose(Ri);

          % if i==1
          %   R_save=Ri;
          % end
          % 
          % %R_anchor(d*i-(d-1):d*i,d*i-(d-1):d*i)=R_save;

      end 

      % Anchoring to the first rotation as the result is define at a constant
      % if d==2
      %     for j=1:n
      %         Rot_Final(j)=atan2(R_proj(2*j,1), R_proj(2*j-1,1));
      %     end
      % 
      %     R=Rot_Final-Rot_Final(1);
      % elseif d==3  
      % 
      %     R=R_anchor*R_proj;

      % end 

end 

function gap=relaxation_gap(R_i,X,measurements,robust_loss_type,w_r)

    %compute optimal value of problem 1

    f1=0;
    f5=0;

    for i=1:length(measurements)
            error1=R_i(2*measurements(i).i-(2-1):2*measurements(i).i,:)'*R_i(2*measurements(i).j-(2-1):2*measurements(i).j,:)-measurements(i).R;
            error5=X(2*measurements(i).i-1:2*measurements(i).i,2*measurements(i).j-1:2*measurements(i).j)-measurements(i).R;
            switch  robust_loss_type
                case KernelFunction.Identity
                    f1=f1+(w_r/2)*power(2,norm(error1,'fro'));
                    f5=f5+(w_r/2)*power(2,norm(error5,'fro'));
                case KernelFunction.L1
                    f1=f1+(w_r/2)*norm(error1,1);
                    f5=f5+(w_r/2)*norm(error5,1);
                case KernelFunction.L2
                    f1=f1+(w_r/sqrt(2))*norm(error1,'fro');
                    f5=f5+(w_r/sqrt(2))*norm(error5,'fro');                   
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
        y=nm^2/2;
    else 
        y=s*(abs(nm)-s/2);
    end

end

