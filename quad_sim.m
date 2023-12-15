%% Initializations
clc; clear; close all;

% Quadrotor parameters
g = 9.81;   % The gravitational acceleration [m/s^2]
l =  0.2;   % Distance from the center of mass to each rotor [m]
m =  0.5;   % Total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48]; % Mass moment of inertia [kg m^2]
mu = 3.0;   % Maximum thrust of each rotor [N]
sigma = 0.01; % The proportionality constant relating thrust to torque
p = [g l m I mu sigma];

% Initial conditions for quadrotor
z0 = zeros(12,1);

% External forces and moments
r = [0; 0; 0];
n = [0; 0; 0];

% Control inputs (constant for now, but can be changed later)
u = [1; 0.9; 1.9; 1.5];

% Intruder initialization
intruder_pos = [0; 0; 5]; % Initial position of the intruder
intruder_start = intruder_pos;
intruder_speed = 3; % Adjust this value for the intruder's speed

% Time vector
t = linspace(0, 10, 1000);

%% Linearization 
x0 = [0; 0; 0; % Position (assuming hover at origin)
      0; 0; 0; % Orientation (level)
      0; 0; 0; % Linear velocity (hover - no velocity)
      0; 0; 0];% Angular velocity (hover - no rotation)
u0 = [m*g; % Thrust to counteract gravity
      0;   % No rotational thrust
      0;
      0];

% This is a conceptual representation
%[A, B] = linmod('quadrotor', x0, u0);
% Note: 'quadrotor' here should be a Simulink model or compatible representation

% Define symbolic variables
syms z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 real
syms u1 u2 u3 u4 real
syms p1 p2 p3 p4 p5 p6 p7 p8 real
syms r1 r2 r3 n1 n2 n3 real
syms g l m I11 I22 I33 mu sigma real % Parameters

% Symbolic state and input vectors
z_sym = [z1; z2; z3; z4; z5; z6; z7; z8; z9; z10; z11; z12];
u_sym = [u1; u2; u3; u4];
p_sym = [p1; p2; p3; p4; p5; p6; p7; p8];
r_sym = [r1; r2; r3];
n_sym = [n1; n2; n3];

% Evaluate the quadrotor function symbolically
dz_sym = quadrotor(0, z_sym, u_sym, p_sym, r_sym, n_sym);

% Compute the Jacobians
A_sym = jacobian(dz_sym, z_sym);
%B_sym = jacobian(dz_sym, u_sym)

% Assume each control input ui contributes to the total thrust and torques
% Total thrust (collective thrust from all rotors)
B_sym(9, 1) = 1/m;   % Thrust from u1 affects z acceleration
B_sym(9, 2) = 1/m;   % Thrust from u2
B_sym(9, 3) = 1/m;   % Thrust from u3
B_sym(9, 4) = 1/m;   % Thrust from u4

% Torques due to differential thrusts
% Roll torque (depends on differential thrust between rotors 1 & 3 and 2 & 4)
B_sym(10, 2) =  l/I(1); % Roll torque due to u1
B_sym(10, 4) = -l/I(1); % Roll torque due to u3

% Pitch torque (depends on differential thrust between rotors 1 & 4 and 2 & 3)
B_sym(11, 1) =  -l/I(2); % Pitch torque due to u2
B_sym(11, 3) = l/I(2); % Pitch torque due to u4

% Yaw torque (depends on torque induced by each rotor)
% Assuming a proportionality factor 'sigma' relating thrust to torque
B_sym(12, 1) =  sigma/I(3); % Yaw torque due to u1
B_sym(12, 2) = -sigma/I(3); % Yaw torque due to u2
B_sym(12, 3) =  sigma/I(3); % Yaw torque due to u3
B_sym(12, 4) = -sigma/I(3); % Yaw torque due to u4


% Substitute in the hover condition and parameter values
A_hover = subs(A_sym, [z_sym; u_sym; p_sym; r_sym; n_sym], [x0; u0; p'; r; n]);
B_hover = subs(B_sym, [sigma, m, l, I(1), I(2), I(3)], [0.01, 0.5, 0.2, 1.24, 1.24, 2.48]);


% Debug: Check if substitution is successful and complete
disp('Substituted A matrix:');
disp(A_hover);

disp('Substituted B matrix:');
disp(B_hover);

% Convert to numeric matrices
A_hover_numeric = double(A_hover);
B_hover_numeric = double(B_hover);

% Define weights for different state variables
position_weight = [5,5,5];
orientation_weight = [1000,1000,1];
velocity_weight = [10,10,10];
angular_velocity_weight = [100,100,1];

% Integral weight
Ki = 1;

% Define weights for control inputs
control_input_weight = 1; % You can adjust this to tune the input cost

% Define the state cost matrix Q
Q = diag([position_weight,...        % Position weights (x, y, z)
          orientation_weight,...     % Orientation weights (phi, theta, psi) 
          velocity_weight,...        % Velocity weights (xdot, ydot, zdot)
          angular_velocity_weight]); % Angular velocity weights (phidot, thetadot, psidot)

% Define the control input cost matrix R
R = control_input_weight * eye(4); % Assuming 4 control inputs

% Implement LQR controller
K = lqr(A_hover_numeric, B_hover_numeric, Q, R);


%% Main Simulation Loop
global quadrotor_state
global quadrotor_state_next
quadrotor_state = z0; % Initialize state vector for quadrotor
path = [z0'];
intruder_path = [intruder_pos'];
dt = t(2) - t(1); % Time step based on the time vector

intruder_direction = [1; 1; 0]; % Initial direction for the intruder
change_direction_interval = 100; % Change direction every 50 iterations


% Integral controller
integral_error = 0;

for k = 1:length(t)-1
    % Define desired state (example: hover at a height of 5 meters)
    position = intruder_pos;
    hover = [7.5; 7.5; 10];
    z_desired = [hover; zeros(9,1)]; % [position; orientation; linear velocity; angular velocity]

     % Calculate error between current state and desired state
    error = quadrotor_state - z_desired;

    % Update integral error
    integral_error = integral_error + error*dt;


    % Calculate control input using LQR (u = -K*error)
    u = -K * error; %- Ki.*integral_error;

    
    recordError(:,k) = error;
    
    % Ensure quadrotor_state is a column vector
    current_state = quadrotor_state;


    % Directly call quadrotor function to get the derivative
    dz = quadrotor(t(k), current_state, u, p, r, n);

    change = dz*dt;

    % Explicit Euler method to update the state
    quadrotor_state_next = current_state + change;

   % Update the state for the next iteration
    quadrotor_state = quadrotor_state_next;

    path = [path;quadrotor_state'];

     % Change the direction of the intruder every fixed number of iterations
    if mod(k, change_direction_interval) == 0
        intruder_direction = [2*rand(2, 1) - 1; 0]; % Change in X and Y, not Z
        intruder_direction = intruder_direction / norm(intruder_direction); % Normalize direction
    end

    % Move the intruder in the current direction
    intruder_pos = intruder_pos + intruder_direction * intruder_speed * dt;
    intruder_pos = max(min(intruder_pos, 10), -10); % Keep within bounds

    intruder_path = [intruder_path; intruder_pos'];

    % Pause for real-time visualization
    %pause(0.01);
end

%%Added by arturo
recordError(:,1000) = error;


%% Plotting the results
for i = 1:4
    ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
                'Xlim',[t(1), t(end)],...
                'TickLabelInterpreter','LaTeX','FontSize',14);
    xlabel(ax(i),'t','Interpreter','LaTeX','FontSize',14);        
end

plot(ax(1), t, path(:,1:3), 'LineWidth', 1.5);
legend(ax(1), {'$x_1$', '$x_2$', '$x_3$'}, 'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(1), '${\bf x}$','Interpreter','LaTeX','FontSize',14);
xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',14);

plot(ax(3), t, path(:,4:6), 'LineWidth', 1.5);
legend(ax(3), {'$\phi$', '$\theta$', '$\psi$'}, 'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(3), '\boldmath$\alpha$','Interpreter','LaTeX','FontSize',14);

plot(ax(2), t, path(:,7:9), 'LineWidth', 1.5);
legend(ax(2), {'$\dot{x}_1$', '$\dot{x}_2$', '$\dot{x}_3$'}, 'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(2), '$\dot{\bf x}$','Interpreter','LaTeX','FontSize',14);

plot(ax(4), t, path(:,10:12), 'LineWidth', 1.5);
legend(ax(4), {'$\omega_1$', '$\omega_2$', '$\omega_3$'}, 'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(4), '\boldmath$\omega$','Interpreter','LaTeX','FontSize',14);


%% Animation
animation_fig = figure;

airspace_box_length = 20;

animation_axes = axes('Parent', animation_fig,...
    'NextPlot','add','DataAspectRatio',[1 1 1],...
    'Xlim',airspace_box_length*[-0.5 0.5],...
    'Ylim',airspace_box_length*[-0.5 0.5],...
    'Zlim',airspace_box_length*[0 1],...
    'box','on','Xgrid','on','Ygrid','on','Zgrid','on',...
    'TickLabelInterpreter','LaTeX','FontSize',14);

view(animation_axes, 3);

% Initialize intruder plot
intruder_plot = plot3(animation_axes, intruder_start(1), intruder_start(2), intruder_start(3), ...
                      'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

N = 10;
Q = linspace(0,2*pi,N)';
circle = 0.3*l*[cos(Q) sin(Q) zeros(N,1)];
circle = subs(circle, l, 0.2);
l = double(0.2);
loc = l*[1 0 0; 0 1 0; -1 0 0; 0 -1 0];

% Convert loc and circle to numeric arrays if they are symbolic
if any(arrayfun(@(x) isa(x, 'sym'), loc(:)))
    loc = double(loc);
end

if any(arrayfun(@(x) isa(x, 'sym'), circle(:)))
    circle = double(circle);
end

silhouette = plot3(0,0,0, '--', 'Color', 0.5*[1 1 1], 'LineWidth', 1 ,...
    'Parent', animation_axes);
body = plot3(0,0,0, 'Color',lines(1), 'LineWidth', 2,...
        'Parent', animation_axes);
for i=1:4
    rotor(i) = plot3(0,0,0, 'Color', lines(1), 'LineWidth', 2,...
        'Parent', animation_axes);
end

tic;
for k=1:length(t)
    % Ensure current state is numeric
    current_state = path(k, :);
    if any(cellfun(@(x) isa(x, 'sym'), num2cell(current_state)))
        error('current_state contains symbolic values.');
    end
    
    R = [ cos(path(k,5))*cos(path(k,6)), sin(path(k,4))*sin(path(k,5))*cos(path(k,6)) - cos(path(k,4))*sin(path(k,6)), sin(path(k,4))*sin(path(k,6)) + cos(path(k,4))*sin(path(k,5))*cos(path(k,6));
          cos(path(k,5))*sin(path(k,6)), cos(path(k,4))*cos(path(k,6)) + sin(path(k,4))*sin(path(k,5))*sin(path(k,6)), cos(path(k,4))*sin(path(k,5))*sin(path(k,6)) - sin(path(k,4))*cos(path(k,6));
                     -sin(path(k,5)),                                 sin(path(k,4))*cos(path(k,5)),                                 cos(path(k,4))*cos(path(k,5))];
    for i=1:4
        % Ensure loc and circle are numeric
        if any(arrayfun(@(x) isa(x, 'sym'), loc(:))) || any(arrayfun(@(x) isa(x, 'sym'), circle(:)))
            error('loc or circle contains symbolic values.');
            %l = double(0.2);
        end
        ctr(i,:) = path(k,1:3) + loc(i,:)*R';
        pose = ones(N,1)*path(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
         
    end
    set(silhouette,'XData', [0, path(k,1), path(k,1), path(k,1)],...
        'YData', [0, 0, path(k,2), path(k,2)],...
        'ZData', [0, 0, 0, path(k,3)]);
    set(body, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );

     % Update intruder position
    set(intruder_plot, 'XData', intruder_path(k,1), 'YData', intruder_path(k,2), 'ZData', intruder_path(k,3));

    pause(t(k)-toc);
    pause(0.01);
end


figure
plot(t', recordError(1,:)', 'g', t', recordError(2,:)', 'b--', t', recordError(3,:)', 'r');
