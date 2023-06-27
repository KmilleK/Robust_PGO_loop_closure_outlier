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