%% Triangular Formation
clear; close all; clc;

%% Setup

% Initialize Robotarium object
N = 3;
initial_position = [   0.5,     0.7,     0.3;
                    0.1732, -0.1732, -0.1732;
                     pi./2,   pi./2,   pi./2];
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_position);

% Select the number of iterations for the experiment
iterations = 500;

% Initialize data containers
dxi = zeros(2, N);
dxu = zeros(2, N);
A_cum = initial_position(1:2, 1);
B_cum = initial_position(1:2, 2);
C_cum = initial_position(1:2, 3);

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
xp = 0.5*cos(theta);
yp = 0.5*sin(theta);
plot(xp, yp);

%% Run
for t = 1:iterations
    % Get initial location data for while loop condition
    x = r.get_poses();
    r.step();
    
    % Compute trajectory
    theta = (t/iterations).*2.*pi;
    % xp = 0.5.*cos(theta);
    % yp = 0.5.*sin(theta);
    % gTO = newCenter(xp, yp, theta, false);
    % gAO = gTO*gAT; dAO = gAO(1:2, 3);                       
    % gBO = gTO*gBT; dBO = gBO(1:2, 3);
    % gCO = gTO*gCT; dCO = gCO(1:2, 3);
    % tri.Vertices = [dAO, dBO, dCO]';

    % Compute corner velocities
    global thetadot;
    thetadot = 2.*pi/iterations;
    xdot = -0.5.*thetadot.*sin(theta);
    ydot = 0.5.*thetadot.*cos(theta);
    gTOdot = HC(xdot, ydot, theta, true);
    gAOdot = gTOdot*gAT; vA = gAOdot(1:2, 3);
    gBOdot = gTOdot*gBT; vB = gBOdot(1:2, 3);
    gCOdot = gTOdot*gCT; vC = gCOdot(1:2, 3);
    A_cum = A_cum + vA;
    B_cum = B_cum + vB;
    C_cum = C_cum + vC;
    % tri.Vertices(1, :) = tri.Vertices(1, :) + vA';
    % tri.Vertices(2, :) = tri.Vertices(2, :) + vB';
    % tri.Vertices(3, :) = tri.Vertices(3, :) + vC';

    while(~init_checker(x, [A_cum, B_cum, C_cum; [x(3, 1), x(3, 2), x(3, 3)]]))
        % Get recent poses from the Robotarium
        x = r.get_poses();

        % Update triangle visual
        tri.Vertices(:, 1) = x(1, :)';
        tri.Vertices(:, 2) = x(2, :)';

        % Set velocities of agents 1,...,N
        dxu = si_to_uni_dynamics([vA, vB, vC], x);
        r.set_velocities(1:N, dxu);

        % Send the previously set velocities to the agents
        r.step();
    end
end

%% Determine simulation errors
r.debug();

%% Homogeneous Coordinates Function
function g = HC(x, y, theta, vel)
    if (vel)
        global thetadot;
        II = [0, 1;
              -1, 0];
        r = [cos(theta), -sin(theta);
             sin(theta),  cos(theta)];
        dr = [-sin(theta), -cos(theta);
              cos(theta),  -sin(theta)];
        g = [dr.*thetadot, [x; y]; [0, 0, 0]];
    else
        g = [cos(theta), -sin(theta), x;
             sin(theta),  cos(theta), y;
                      0,           0, 1];
    end
end