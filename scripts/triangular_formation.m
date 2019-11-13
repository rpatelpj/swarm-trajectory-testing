%% Triangular Formation
clear; close all; clc;

%% Setup

% Initialize Robotarium object
N = 3;
initial_position = [         -0.75,           -1.25, -1;
                    -sind(60).*0.5,  -sind(60).*0.5,  0;
                                 0,               0,  0];
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_position);

% Select the number of iterations for the experiment
iterations = 700;

% Initialize data container
dxu = zeros(2, N);

%% Run
for t = 1:iterations
    %% Get recent poses from the Robotarium
    x = r.get_poses();

    %% Run arc differential equation

    % Set linear and angular velocities
    dxu(1, :) = 0.13;
    dxu(2, :) = pi./3 .* cos(t.*r.time_step);

    %% Send velocities to agents

    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);

    % Send the previously set velocities to the agents
    r.step();
end

%% Determine simulation errors
r.debug();