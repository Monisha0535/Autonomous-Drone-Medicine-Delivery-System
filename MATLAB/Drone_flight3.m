clear all
close all
clc

% prepare_data.m
% Environment and obstacle setup
environment_size = [100, 100, 100]; % Environment size
start_point = [10, 10, 10];
end_point = [90, 90, 0]; % End point on the ground

% Define obstacles for each flight phase
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

% Drone path parameters
cruise_altitude = 50;
step_size = 1;
safety_distance = 6;

% Generate paths for each flight phase
takeoff_end_point = [start_point(1:2), cruise_altitude];
takeoff_path = find_path(start_point, takeoff_end_point, takeoff_obstacles, step_size, safety_distance);

cruise_start_point = takeoff_end_point;
cruise_end_point = [end_point(1:2), cruise_altitude];
cruise_path = find_path(cruise_start_point, cruise_end_point, cruise_obstacles, step_size, safety_distance);

landing_start_point = cruise_end_point;
landing_path = find_path(landing_start_point, end_point, landing_obstacles, step_size, safety_distance);

% Concatenate paths
full_path = [takeoff_path; cruise_path(2:end, :); landing_path(2:end, :)];

% Generate time vector and path data
num_points = size(full_path, 1);
time_step = 0.1;
time_vector = (0:num_points-1)' * time_step;
path_data = [time_vector, full_path];

% Save data to workspace
assignin('base', 'path_data', path_data);

takeoff_obstacles = [zeros(size(takeoff_obstacles, 1), 1), takeoff_obstacles];
cruise_obstacles = [zeros(size(cruise_obstacles, 1), 1), cruise_obstacles];
landing_obstacles = [zeros(size(landing_obstacles, 1), 1), landing_obstacles];

assignin('base', 'takeoff_obstacles', takeoff_obstacles);
assignin('base', 'cruise_obstacles', cruise_obstacles);
assignin('base', 'landing_obstacles', landing_obstacles);

% Define the pathfinding function
function path = find_path(start_point, end_point, obstacles, step_size, safety_distance)
    is_near_obstacle = @(point) any(sqrt(sum((obstacles - point).^2, 2)) < safety_distance);
    random_step = @() step_size * (2*rand(1,3) - 1);
    
    path = start_point;
    current_point = start_point;
    
    while norm(current_point - end_point) > step_size
        direction = (end_point - current_point) / norm(end_point - current_point);
        next_point = current_point + direction * step_size;
        if ~is_near_obstacle(next_point)
            current_point = next_point;
        else
            while is_near_obstacle(next_point)
                next_point = current_point + random_step();
            end
            current_point = next_point;
        end
        path = [path; current_point];
    end
    path = [path; end_point];
end
