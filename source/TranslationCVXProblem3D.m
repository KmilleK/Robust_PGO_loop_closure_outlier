function  varargout=TranslationCVXProblem3D(poses,measurements,edge_separation,robust_loss_type,n,w_t)
    % Perform the necessary computation or transformation
    % to derive the measurement vector from the pose vector.
    
    %Silent cvx solver 
    % cvx_quiet(true);

    

    
    d=3;

    % Start the timer
    tic
    
    %Translation computation 
    if edge_separation
        t=cvx_solver_separate(poses,measurements,robust_loss_type,n,d,w_t);
    else 
        t=cvx_solver_join(poses,measurements,robust_loss_type,n,d,w_t);
    end
   
    updatePoses=poses;
    
    for i=1:n
        updatePoses(i).t=t(d*i-(d-1):d*i,:);
    end 
    
    elapsedTimeTranslation = toc;
    varargout{1} = updatePoses;
    varargout{2} = elapsedTimeTranslation;


end

function t=cvx_solver_separate(poses,measurements,robust_loss_type,n,d,w_t)
    
    cvx_clear

     cvx_begin
    
        variable t(d*n,1) ;
        objective=0;
    
        for i=1:length(measurements)
             if strcmpi(strtrim(measurements(i).type), 'Odometry')
                termint=poses(measurements(i).i).R'*(t(d*measurements(i).j-(d-1):d*measurements(i).j,:)-t(d*measurements(i).i-(d-1):d*measurements(i).i,:))-measurements(i).t;
                objective=objective+w_t*sum_square(termint);
             elseif strcmpi(strtrim(measurements(i).type), 'LoopClosure')
                 termint=poses(measurements(i).i).R'*(t(d*measurements(i).j-(d-1):d*measurements(i).j,:)-t(d*measurements(i).i-(d-1):d*measurements(i).i,:))-measurements(i).t;
                 switch  robust_loss_type
                    case KernelFunction.Identity
                        objective=objective+w_t*sum_square(termint);
                    case KernelFunction.L1
                        objective=objective+w_t*norm(termint,1);
                    case KernelFunction.L2
                        objective=objective+w_t*norm(termint,2);
                    case KernelFunction.Huber
                        objective=objective+huber_circ(termint,[],1/w_t);
                    otherwise
                        disp('wrong robust loss function type');
                end
            
             else
                 disp('wrong type of edge');
                    
            end
        end

        minimize (objective);
        subject to
        for i=1:d
            t(i)==0;
        end
    cvx_end


end

function t=cvx_solver_join(poses,measurements,robust_loss_type,n,d,w_t)

cvx_clear

     cvx_begin
    
        variable t(d*n,1) ;
        objective=0;
    
        for i=1:length(measurements)
                 termint=poses(measurements(i).i).R'*(t(d*measurements(i).j-(d-1):d*measurements(i).j,:)-t(d*measurements(i).i-(d-1):d*measurements(i).i,:))-measurements(i).t;
                 switch  robust_loss_type
                    case KernelFunction.Identity
                        objective=objective+w_t*sum_square(termint);
                    case KernelFunction.L1
                        objective=objective+w_t*norm(termint,1);
                    case KernelFunction.L2
                        objective=objective+w_t*norm(termint,2);
                    case KernelFunction.Huber
                        objective=objective+huber_circ(termint,[],1/w_t);
                   % case KernelFunction.Charbonnier
                    %    objective=objective+sqrt((w_t)^2*sum_square(termint)+1)-1;
                    otherwise
                        disp('wrong robust loss function type');
                end
         end

        minimize (objective);
        subject to
        for i=1:d
            t(i)==0;
        end
    cvx_end



end