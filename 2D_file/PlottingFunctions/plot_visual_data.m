function plot_visual_data(poses,measurements)
    % poses: a vector of Pose objects representing the robot's position and
    % orientation ground truth 
    % measurement are the noisy measurement between the node used before
    % optimization 
    
     % Define the size of the robot triangle
    robotSize = 0.5;                        % Change this value to adjust the size of the triangle
    base = robotSize * 2;
    height = robotSize * sqrt(3);

    % Plotting the pose 
    figure;
    hold on;

    
    for i = 1:length(measurements)
        measurement = measurements(i);
        if strcmpi(strtrim(measurements(i).type), 'Inlier')
            plot([poses(measurement.i).t(1), poses(measurement.j).t(1)],[poses(measurement.i).t(2), poses(measurement.j).t(2)],'Color',  [200 200 200]/255,'LineWidth', 0.8);
        else
            plot([poses(measurement.i).t(1), poses(measurement.j).t(1)],[poses(measurement.i).t(2), poses(measurement.j).t(2)],'red','LineWidth', 1.5);
        end
    end

    for i = 1:length(poses)
        position = poses(i).t;
        orientation = poses(i).R;
        
        % Plot the robot triangle
        points_x = [position(1), position(1) - base / 2, position(1) + base / 2, position(1)];
        points_y = [position(2) + robotSize, position(2) - height / 2, position(2) - height / 2, position(2) + robotSize];
        
        % Rotate the triangle points
        
        rotated_points = orientation * [points_x - position(1); points_y - position(2)];
        points_x = rotated_points(1, :) + position(1);
        points_y = rotated_points(2, :) + position(2);
        
        fill(points_x, points_y, 'black'); % Use 'fill' to draw a filled triangle


        % Define the robot's orientation line
        orientation_length = robotSize * 2; % Change this value to adjust the length of the orientation line
        orientation_x = position(1) + orientation_length * orientation(1,2);
        orientation_y = position(2) + orientation_length * orientation(1,1) ;

        % Plot the robot's orientation line
        plot([position(1), orientation_x], [position(2), orientation_y], 'b--','LineWidth', 2);
    end







    % Set plot settings
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Simulated pose graph');

end

function plot_noisy_measurement(measurements)
    % plot only the measurement information 

     % Plotting the robot
    figure;
    hold on;

    initialPosition = [0; 0; 0];
    initialOrientation = eye(3); % Identity matrix represents no rotation

    positions = [initialPosition];
    orientations = [initialOrientation];

    R_LC = [0,0;0,0;0,0];
    R_corrected= [0,0;0,0;0,0];

    orientations = initialOrientation;
    
    position_index=1;
    % Plotting the translations between nodes
    for i = 1:length(measurements)
        measurement = measurements(i);
        
        if strcmpi(strtrim(measurement.type), 'Odometry')

            currentPosition = positions(:,position_index);
            currentOrientation = orientations(:,3*position_index-2:3*position_index);

            newOrientation = currentOrientation * measurement.R;          
            newPosition = currentPosition + currentOrientation* measurement.t;
            
            positions = [positions, newPosition];
            orientations = cat(3, orientations, newOrientation);

            position_index=position_index+1;
        
        end

        if strcmpi(strtrim(measurement.type),'LoopClosure')

            R_LC(:,1)=positions(:,measurement.i);
            R_LC(:,2)=positions(:,measurement.i) + orientations(:,3*measurement.i-2:3*measurement.i)*measurement.t;
            plot3(R_LC(1, :),R_LC(2, :), R_LC(3, :), 'r');

            R_corrected(:,1)=R_LC(:,2);
            R_corrected(:,2)=positions(:,measurement.j);
            plot3(R_corrected(1, :),R_corrected(2, :), R_corrected(3, :), 'g--', 'LineWidth', 2);
           
        end



    end

    
    plot3(positions(1, :), positions(2, :), positions(3, :), 'b');
    

    % Set plot settings
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    titleText = 'Noisy measurement trajectory';
    title(titleText);
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

