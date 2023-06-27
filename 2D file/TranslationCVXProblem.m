function  varargout=TranslationCVXProblem(poses,measurements)
    % Perform the necessary computation or transformation
    % to derive the measurement vector from the pose vector.
    
    % Start the timer
tic

cvx_clear
global n d w_r w_t;

%Translation computation 

cvx_begin

    variable t(2*n,1) ;
    objective=0;

    for i=1:length(measurements)
         if strcmpi(strtrim(measurements(i).type), 'Odometry')
            termint=transpose(poses(measurements(i).i).generateRotationMatrix())*t(2*measurements(i).j-1:2*measurements(i).j,:)-transpose(poses(measurements(i).i).generateRotationMatrix())*t(2*measurements(i).i-1:2*measurements(i).i,:)-measurements(i).t;
            objective=objective+sum_square(termint);
         elseif strcmpi(strtrim(measurements(i).type), 'LoopClosure')
            termint=transpose(poses(measurements(i).i).generateRotationMatrix())*t(2*measurements(i).j-1:2*measurements(i).j,:)-transpose(poses(measurements(i).i).generateRotationMatrix())*t(2*measurements(i).i-1:2*measurements(i).i,:)-measurements(i).t;
            objective=objective+sum_square(termint);
             

         else
                
        end
    end

    minimize (objective);
    subject to 
        t(1)==0;
        t(2)==0;
     
cvx_end

updatePoses=poses;
for i=1:n
    updatePoses(i).t=t(2*i-1:2*i,:);
end 

elapsedTimeTranslation = toc;
% save('result.mat', 'elapsedTimeTranslation','-append');

save('workspace_translation.mat');
elapsedTimeTranslation = toc;
varargout{1} = updatePoses;
varargout{2} = elapsedTimeTranslation;


end