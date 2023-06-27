function [poses,measurements] = g20Data(filename)
    
    % Open and read the G2O file
    fid = fopen(filename, 'r');
    data = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);

        % Initialize counters for vertices and edges
    vertex_count = 0;
    edge_count = 0;
    
    % Process the lines in the G2O file
    for i = 1:numel(data{1})
        line = data{1}{i};
        if startsWith(line, 'VERTEX_SE2')
            vertex_count = vertex_count + 1;
        elseif startsWith(line, 'EDGE_SE2')
    edge_count = edge_count + 1;
        end
    end
    


    %Poses information storage 
    poses= Pose.empty(vertex_count,0);

    %measurement information storage 
    measurements= Measurement.empty(edge_count,0);
    
    vertex_index = 1;
    edge_index = 1;
    % Process the lines in the G2O file
    for i = 1:numel(data{1})
        line = data{1}{i};
        if startsWith(line, 'VERTEX_SE2')
            % Extract vertex information
            parts = strsplit(line);
            vertex_id = str2double(parts{2});
            x = str2double(parts{3});
            y = str2double(parts{4});
            theta = str2double(parts{5});
            
            % Store vertex information
            poses(vertex_index)=Pose(vertex_id+1,theta,[x;y]);
            vertex_index = vertex_index+1;
 
        elseif startsWith(line, 'EDGE_SE2')
            % Extract edge information
            parts = strsplit(line);
            source_id = str2double(parts{2});
            target_id = str2double(parts{3});
            x = str2double(parts{4});
            y = str2double(parts{5});
            theta = str2double(parts{6});
            
            % Store edge information
           measurements(edge_index)=Measurement('Odometry',source_id+1,target_id+1,theta,[x;y]);
           edge_index=edge_index+1;
        end
    end

    plotRobotM3500(poses,'Robot Position and Orientation noisy');


end


