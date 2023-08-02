function  varargout=TranslationCVXProblem2D(poses_inter,measurements,robust_loss_type,n,w_t)
    % Perform the necessary computation or transformation
    % to derive the measurement vector from the pose vector.
    
    % Start the timer

cvx_clear
cvx_quiet(true);

%Translation computation 
tic

cvx_begin

    variable t(2*n,1) ;
    objective=0;

    for i=1:length(measurements)
            error=poses_inter(measurements(i).i).R'*t(2*measurements(i).j-1:2*measurements(i).j,:)-poses_inter(measurements(i).i).R'*t(2*measurements(i).i-1:2*measurements(i).i,:)-measurements(i).t;
            switch  robust_loss_type
                case KernelFunction.Identity
                    objective=objective+w_t*power(2,norm(error,2));
                case KernelFunction.L1
                    objective=objective+w_t*norm(error,1);
                case KernelFunction.L2
                    objective=objective+w_t*norm(error,2);                   
                case KernelFunction.Huber
                    %objective=objective+huber(sum(sum(error)));
                    objective=objective+huber_circ(error,[],1/w_t);   
                otherwise
                    disp('wrong robust loss function type');
            end
    end

    minimize (objective);
   
cvx_end

elapsedTimeTranslation = toc;

updatePoses=poses_inter;
for i=1:n
    updatePoses(i).t=t(2*i-1:2*i,:);
end 


varargout{1} = updatePoses;
varargout{2} = elapsedTimeTranslation;


end