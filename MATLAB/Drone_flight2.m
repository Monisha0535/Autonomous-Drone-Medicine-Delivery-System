close all
clear all
clc

% Define the environment
environment_size = [100, 100, 100]; % Define the size of the environment

% Define the startingnd and ending points
start_point = [10, 10, 10];
end_point = [90, 90, 0]; % End point on the ground

% Define more obstacles for each flight phase
takeoff_obstacles = [
    15, 15, 15;
    20, 20, 20;
    25, 25, 10;
    30, 15, 20;
];

cruise_obstacles = [
    50, 50, 50;
    60, 60, 60;
    55, 55, 45;
    65, 45, 55;
];

landing_obstacles = [
    85, 85, 5;
    80, 80, 10;
    75, 75, 0;
    90, 70, 0;
];

% Define the cruise altitude
cruise_altitude = 50;

% Initialize the drone's position
drone_position = start_point;

% Define the step size and safety distance
step_size = 1;
safety_distance = 6;

% Find the path for each flight phase
takeoff_end_point = [start_point(1:2), cruise_altitude];
takeoff_path = find_path(start_point, takeoff_end_point, takeoff_obstacles, step_size, safety_distance);

cruise_start_point = takeoff_end_point;
cruise_end_point = [end_point(1:2), cruise_altitude];
cruise_path = find_path(cruise_start_point, cruise_end_point, cruise_obstacles, step_size, safety_distance);

landing_start_point = cruise_end_point;
landing_path = find_path(landing_start_point, end_point, landing_obstacles, step_size, safety_distance);

% Concatenate the paths
full_path = [takeoff_path; cruise_path(2:end, :); landing_path(2:end, :)];

% Generate time vector and path data
num_points = size(full_path, 1); % Number of points in the path
time_step = 0.1; % Time step for each point
time_vector = (0:num_points-1)' * time_step; % Time vector

% Create the data matrix (each column is a variable: x, y, z)
data_matrix = full_path;

% Combine the time vector and data matrix
path_data = [time_vector, data_matrix];

% Save to workspace variable
assignin('base', 'path_data', path_data);

% Add a time column to obstacles (time = 0)
takeoff_obstacles = [zeros(size(takeoff_obstacles, 1), 1), takeoff_obstacles];
cruise_obstacles = [zeros(size(cruise_obstacles, 1), 1), cruise_obstacles];
landing_obstacles = [zeros(size(landing_obstacles, 1), 1), landing_obstacles];

start_point = [zeros(size(start_point, 1), 1), start_point];
end_point = [zeros(size(end_point, 1), 1), end_point];

% Save obstacles to workspace variables
assignin('base', 'takeoff_obstacles', takeoff_obstacles);
assignin('base', 'cruise_obstacles', cruise_obstacles);
assignin('base', 'landing_obstacles', landing_obstacles);

assignin('base', 'start_point', cruise_obstacles);
assignin('base', 'end_point', end_point);

%%
% Pathfinding function (simplified)
function path = find_path(start_point, end_point, obstacles, step_size, safety_distance)
    % Nested function to check if a point is within the safety distance of an obstacle
    is_near_obstacle = @(point) any(sqrt(sum((obstacles - point).^2, 2)) < safety_distance);
    
    % Nested function to generate a random step
    random_step = @() step_size * (2*rand(1,3) - 1);
    
    % Initialize the path and current point
    path = start_point;
    current_point = start_point;
    
    % Loop to generate the path
    while norm(current_point - end_point) > step_size
        direction = (end_point - current_point) / norm(end_point - current_point);
        next_point = current_point + direction * step_size;
        if ~is_near_obstacle(next_point)
            current_point = next_point;
        else
            % Find a new direction to avoid the obstacle
            while is_near_obstacle(next_point)
                next_point = current_point + random_step();
            end
            current_point = next_point;
        end
        path = [path; current_point];
    end
    path = [path; end_point];
end
%%
function visualize_flight(path, takeoff_obstacles, cruise_obstacles, landing_obstacles)
    %#codegen
    coder.extrinsic('plot3', 'hold', 'grid', 'xlabel', 'ylabel', 'zlabel', 'title', 'axis', 'view', 'pause', 'drawnow');
    
    figure;
    hold on;
    plot3(path(:,1), path(:,2), path(:,3), 'bo-', 'MarkerSize', 5, 'LineWidth', 2); % Drone path in blue
    plot3(takeoff_obstacles(:,1), takeoff_obstacles(:,2), takeoff_obstacles(:,3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Takeoff obstacles in red
    plot3(cruise_obstacles(:,1), cruise_obstacles(:,2), cruise_obstacles(:,3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Cruise obstacles in red
    plot3(landing_obstacles(:,1), landing_obstacles(:,2), landing_obstacles(:,3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Landing obstacles in red
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on;
    title('Drone Flight Simulation');
    axis([0 100 0 100 0 100]);
    view(3);
    hold off;
end
