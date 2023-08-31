function plot_visual_data(poses,measurements)
    % poses: a vector of Pose objects representing the robot's position and
    % orientation ground truth 
    % measurement are the noisy measurement between the node used before
    % optimization 


    % Define the size of the robot triangle
    robotSize = 0.2; % Change this value to adjust the size of the triangle
    base = robotSize * 2;
    height = robotSize * sqrt(3);

    % Plotting the pose 
    figure;
    hold on;

    for i = 1:length(measurements)
        measurement = measurements(i);
        if strcmpi(strtrim(measurements(i).type), 'Odometry')
            plot3([poses(measurement.i).t(1), poses(measurement.j).t(1)], ...
                  [poses(measurement.i).t(2), poses(measurement.j).t(2)], ...
                  [poses(measurement.i).t(3), poses(measurement.j).t(3)], ...
                  'Color', [200 200 200]/255, 'LineWidth', 0.8);
        else
            plot3([poses(measurement.i).t(1), poses(measurement.j).t(1)], ...
                  [poses(measurement.i).t(2), poses(measurement.j).t(2)], ...
                  [poses(measurement.i).t(3), poses(measurement.j).t(3)], ...
                  'red', 'LineWidth', 1.5);
        end
    end

    for i = 1:length(poses)
        position = poses(i).t;
        orientation = poses(i).R;
        
        % Plot the robot triangle
        points_x = [position(1), position(1) - base / 2, position(1) + base / 2, position(1)];
        points_y = [position(2) + robotSize, position(2) - height / 2, position(2) - height / 2, position(2) + robotSize];
        
        % Rotate the triangle points
        rotated_points = orientation * [points_x - position(1); points_y - position(2); zeros(1, 4)];
        points_x = rotated_points(1, :) + position(1);
        points_y = rotated_points(2, :) + position(2);
        points_z = rotated_points(3, :) + position(3);
        
        fill3(points_x, points_y, points_z, 'black'); % Use 'fill3' to draw a filled triangle

        % Define the robot's orientation line
        orientation_length = robotSize * 2; % Change this value to adjust the length of the orientation line
        orientation_x = position(1) + orientation_length * orientation(1, 2);
        orientation_y = position(2) + orientation_length * orientation(1, 1);
        orientation_z = position(3) + orientation_length * orientation(1, 3);

        % Plot the robot's orientation line
        plot3([position(1), orientation_x], [position(2), orientation_y], [position(3), orientation_z], 'b--', 'LineWidth', 2);
    end

    % Set plot settings
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Simulated pose graph');

    % Enable 3D rotation interactivity
    rotate3d on;

end
