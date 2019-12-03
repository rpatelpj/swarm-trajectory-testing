%% Triangular Formation with Leader
clear; close all; clc;

%% Setup

% Initialize Robotarium object
N = 3;
initial_position = [   0.5,     0.7,     0.3;
                    0.1732, -0.1732, -0.1732;
                     pi./2,   pi./2,   pi./2];
dx = -0.6; dy = -0.2; % Distance from origin
initial_position(1, :) = initial_position(1, :) + dx;
initial_position(2, :) = initial_position(2, :) + dy;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_position);

% Select the number of iterations for the experiment
iterations = 500;

% Initialize data containers
dxi = zeros(2, N);
dxu = zeros(2, N);
A_sum = initial_position(1:2, 1);
B_sum = initial_position(1:2, 2);
C_sum = initial_position(1:2, 3);

% Initialize functions
init_checker = create_is_initialized();
si_to_uni_dynamics = create_si_to_uni_dynamics();

% Define rigid body
gAT = [1, 0, 0;
       0, 1, 0.1737;
       0, 0, 1];
gBT = [1, 0, 0.2;
       0, 1, -0.1737;
       0, 0, 1];
gCT = [1, 0, -0.2;
       0, 1, -0.1737;
       0, 0, 1];

% Draw triangle visual
hold on; tri = patch(initial_position(1, :), initial_position(2, :), 'y');

% Plot trajectory
theta = linspace(0, 2.*pi, iterations);
xp = 0.5*cos(theta) + dx;
yp = 0.5*sin(theta) + dy;
plot(xp, yp);

%% Run
for t = 1:iterations
    % Get initial location data for while loop condition
    x = r.get_poses();
    r.step();

    % Compute corner velocities
    theta = (t/iterations).*2.*pi;
    thetadot = 2.*pi/iterations;
    xdot = -0.5.*thetadot.*sin(theta);
    ydot = 0.5.*thetadot.*cos(theta);
    rdot = [-sin(theta), -cos(theta);
             cos(theta), -sin(theta)];
    gTTndot = [rdot.*thetadot, [xdot; ydot];
                         0, 0,            0];
    gATndot = gTTndot*gAT; vA = gATndot(1:2, 3);
    vB = -(norm(x(1:2, 1) - x(1:2, 2)).^2 - 0.4.^2).*(x(1:2, 2) - x(1:2, 1)) ...
         -(norm(x(1:2, 3) - x(1:2, 2)).^2 - 0.4.^2).*(x(1:2, 2) - x(1:2, 3));
    vC = -(norm(x(1:2, 1) - x(1:2, 3)).^2 - 0.4.^2).*(x(1:2, 3) - x(1:2, 1)) ...
         -(norm(x(1:2, 2) - x(1:2, 3)).^2 - 0.4.^2).*(x(1:2, 3) - x(1:2, 2));
    A_sum = A_sum + vA;

    % Update triangle visual
    tri.Vertices(1, :) = tri.Vertices(1, :) + vA';
    tri.Vertices(2, :) = tri.Vertices(2, :) + vB';
    tri.Vertices(3, :) = tri.Vertices(3, :) + vC';

    while(~init_checker(x, [A_sum, x(1:2, 2), x(1:2, 3); [x(3, 1), x(3, 2), x(3, 3)]]))
        % Get recent poses from the Robotarium
        x = r.get_poses();

        % Set velocities of agents 1,...,N
        dxu = si_to_uni_dynamics([vA, vB, vC], x);
        r.set_velocities(1:N, dxu);

        % Send the previously set velocities to the agents
        r.step();
    end
end

%% Determine simulation errors
r.debug();