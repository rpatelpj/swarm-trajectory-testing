%% Triangular Formation (Position Control)
clear; close all; clc;

%% Setup

% Initialize Robotarium object
N = 3;
initial_position = [   0.5,     0.7,     0.3;
                    0.1732, -0.1732, -0.1732;
                         0,       0,       0];
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_position);

% Initialize data containers
dxi = zeros(2, N);
dxu = zeros(2, N);

% Draw triangle visual
hold on; tri = patch(initial_position(1, :), initial_position(2, :), 'y');

% Initialize functions
init_checker = create_is_initialized();
position_controller = create_si_position_controller();
si_barrier_certificate = create_si_barrier_certificate();
si_to_uni_dynamics = create_si_to_uni_dynamics();

% Define rigid body
gAs = [1, 0, 0;
       0, 1, 0.1732;
       0, 0, 1];
gBs = [1, 0, 0.2;
       0, 1, -0.1732;
       0, 0, 1];
gCs = [1, 0, -0.2;
       0, 1, -0.1732;
       0, 0, 1];

% Define trajectory
num_waypoints = 100;
thetas = linspace(0, 2.*pi, num_waypoints);
waypoints = [0.5.*cos(thetas);
             0.5.*sin(thetas)];
plot(waypoints(1, :), waypoints(2, :));

%% Run
for t = 1:num_waypoints
    % Calculate next waypoint position
    g = newOrigin(waypoints(1, t), waypoints(2, t), thetas(t));
    gA = g*gAs; dA = gA(1:2, 3);
    gB = g*gBs; dB = gB(1:2, 3);
    gC = g*gCs; dC = gC(1:2, 3);

    % Get initial location data for while loop condition
    x=r.get_poses();
    r.step();

    while(~init_checker(x, [dA, dB, dC; [x(3, 1), x(3, 2), x(3, 3)]]))
        %% Get recent poses from the Robotarium
        x = r.get_poses();

        % Update triangle visual
        tri.Vertices(:, 1) = x(1, :)';
        tri.Vertices(:, 2) = x(2, :)';

        %% Run

        % Convert positions to velocities
        dxi(:, 1) = position_controller(x(1:2, 1), dA);
        dxi(:, 2) = position_controller(x(1:2, 2), dB);
        dxi(:, 3) = position_controller(x(1:2, 3), dC);

        %% Send velocities to agents

        % Set velocities of agents 1,...,N
        dxi = si_barrier_certificate(dxi, x(1:2, :));
        dxu = si_to_uni_dynamics(dxi, x);
        r.set_velocities(1:N, dxu);

        % Send the previously set velocities to the agents
        r.step();
    end
end

%% Determine simulation errors
r.debug();

function g = newOrigin(x, y, theta)
    g = [cos(theta), -sin(theta), x;
         sin(theta),  cos(theta), y;
                  0,           0, 1];
end