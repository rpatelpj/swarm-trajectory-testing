%% Circular Path
clear; close all; clc;

%% Setup

% Initialize Robotarium object
N = 1;
initial_position = [0; 0; 0];
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_position);

% Select the number of iterations for the experiment
iterations = 500;

% Set body velocity
global vB
vB = [0.15; pi./4];

% Plot position map
[~, x] = ode45(@pathdiffeq, [0:r.time_step:r.time_step*iterations], initial_position);
hold on; plot(x(:,1), x(:,2));

% Initialize data container
dxu = zeros(2, N);

%% Run
for t = 1:iterations
    %% Get recent poses from the Robotarium
    x = r.get_poses();

    %% Run
    
    % Set linear and angular velocities
    dxu(1) = vB(1);
    dxu(2) = vB(2);

    %% Send velocities to agents

    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);

    % Send the previously set velocities to the agents
    r.step();
end

%% Determine simulation errors
r.debug();

%% Position Map Function
function xdot = pathdiffeq(t, x)
    global vB
    xdot = zeros(3,1);
    xdot(1) = cos(x(3)) .* vB(1);
    xdot(2) = sin(x(3)) .* vB(1);
    xdot(3) = vB(2);
end