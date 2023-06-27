function plot_visual_solution(Final_poses)
    % Final poses: a vector of Pose objects representing the robot's
    % position after optimization

    % Plotting the robot
    figure;
    hold on;

    for i = 1:length(Final_poses)
        position = Final_poses(i).t;
        orientation = rotm2eul(Final_poses(i).R);
        
        % Plot the robot position
        scatter3(position(1), position(2), position(3), 'filled', 'b');
        text(position(1), position(2), position(3),  num2str(i), 'FontSize', 10, 'Color', 'b');

        % Plot the robot orientation
        scale = 0.5;  % Length of the orientation arrows
        plotArrow3(position, orientation, scale, 'r',  2);

        if i < numel(Final_poses)
            position2 = Final_poses(i+1).t;
            plot3([position(1), position2(1)], [position(2), position2(2)], [position(3), position2(3)], 'k--', 'LineWidth', 1);
        end
  
    end

    % Set plot settings
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Robot Position and Orientation optimization results');
    view(3);  % Set the view to 3D

    hold off;
  


end


function plotArrow3(position, angles, scale, color, linewidth)
    % Plot a 3D arrow given position, angles (roll, pitch, yaw), scale, color, and linewidth
    
    % Convert Euler angles to rotation matrix
    R =eul2rotm(angles);
    
    % Define arrow points
    arrowPoints = scale * [0, 0, 0; 1, 0, 0; 0.8, 0.1, 0; 0.8, -0.1, 0]';
    
    % Rotate and translate arrow points
    arrowPoints = R * arrowPoints + repmat(position, 1, size(arrowPoints, 2));
    
    % Plot arrow
    plot3(arrowPoints(1, :), arrowPoints(2, :), arrowPoints(3, :), 'Color', color, 'LineWidth', linewidth);
end

