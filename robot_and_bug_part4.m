clc; clear; close all;

% Robot's equations of motion
A = [zeros(2), eye(2); zeros(2,4)];
B = [zeros(2); eye(2)];

dr = @(t, r, u, g) A*r + B*u + g;

u = @(r, rd, ud, K) ud + K*(rd - r);

% Using Pole-Placement to define K
p = [-2, -2, -3, -3];
K = place(A, B, p);

% Bug kinematics
a = 0.3; w = 10;
db = @(t,b) [a*w*cos(t*w)*cos(t) - sin(t)*(a*sin(t*w) + 1);
             cos(t)*(a*sin(t*w) + 1) + a*w*cos(t*w)*sin(t)];

% Initial Condition
r0 = zeros(4,1);    % Robot's initial state
b0 = [0; 2];        % Bug's initial state

% Problem parameters
epsilon = 0.1;

%% Phase I: Pursue
tspan_I = [0 10];
z0_I = [r0; b0];

options = odeset('Event', @(t,z) catchBug(t, z, epsilon),...
    'RelTol',1e-6);

[t_I, z_I, te, ze] = ode45( @(t, z) augmentedSystem(t, z, dr, db, u, K),...
    tspan_I, z0_I, options);

%% Phase II: Return

if(isempty(te)) % if the robot failed to catch the bug
    % Decoupling the augmented state vector z from only phase I
    t = t_I;
    r = z_I(:,1:4);
    b = z_I(:,[5, 6]);
else
    tspan_II = [te, tspan_I(end)];
    r0_II = z_I(end,1:4)';
    
    [tk, c] = dist(tspan_II, [0.1 0.5], 2);
    g = @(t)[0; 0; c(find(tk <= t, 1, 'last'),:)'];
    
    rd = zeros(4,1); ud = zeros(2,1);
    
    [t_II, r_II] = ode45(@(t,r) dr(t, r, u(r, rd, ud, K), g(t) ),...
        tspan_II, r0_II);
    
    
    % Decoupling the augmented state vector z from phase I and phase II
    t = [t_I; t_II];
    r = [z_I(:,1:4); r_II];
    b = [z_I(:,[5, 6]); r_II(:,[1,2])]; % Since the bug is captured by the robot,
                                  % bug's states are equal to the robot's.
end


%% Results

plotNorm(t_I, z_I(:,1:4), z_I(:,[5, 6]), epsilon);
animate(t, r, b, epsilon, te);

%% External Functions

function dz = augmentedSystem(t, z, dr, db, u, K)

% Decouple r and b states from the augmented state z
r = z(1:4, 1);
b = z([5, 6], 1);

ud = @(t) [0; cos(t)/2];
dz = [ dr(t, r, u(r, [b; db(t, b)], ud(t) , K), zeros(4,1) ) ; db(t, b) ];

end


function [value,isterminal,direction] = catchBug(t, z, epsilon)

value = norm( z([5, 6]) - z([1, 2]) ) > epsilon;
isterminal = 1; % = 1 -> the integration is to terminate.
direction = 0;

end


function [tk, c] = dist(tspan, tau, gamma)

% tspan: The domain of the piecewise constant function 
% tau = [tau_min, tau_max]: minimum and maximum time intervals
% gamma: Maximum norm of the c at every t

dim = 2;    % The dimension of c at every t
p = 2;      % p-norm of c at every t. p = 2 -> l2 norm or Euclidean norm

delta_t = diff(tau);

tk = tspan(1);
while(tk(end) < tspan(2))
    tk(end+1,1) = tk(end) + delta_t*rand + tau(1);
end

N = length(tk);
c = 2*rand(N-1, dim) - 1;
c = gamma*rand(N-1, 1).*(c./vecnorm(c,p,2));
end


function plotNorm(t,r,b, epsilon)

norm_error = vecnorm( b - r(:,[1,2]), 2, 2);

ax = axes('Parent', figure('Name','Norm of the Error'),...
    'NextPlot','Add','Box','on', 'XGrid', 'on', 'YGrid', 'on',...
    'Xlim', t([1, end]), 'Ylim', [0, 1.2*max(norm_error)],...
    'TickLabelInterpret','LaTeX','FontSize',14);


plot(ax, t, norm_error, 'LineWidth', 2);
plot(ax, t([1 end]), epsilon*[1 1], 'k', 'LineWidth', 1);
xlabel('$t$', 'Interpreter','LaTeX','FontSize',14);
ylabel('$\big\| [b_1(t) - r_1(t),\, b_2(t) - r_2(t)]^T \big\|$',...
    'Interpreter','LaTeX','FontSize',14);
end


function animate(t, r, b, epsilon, te)

ax = axes('Parent', figure('Name','Robot following bug: animation'),...
    'NextPlot','Add','Box','on',...
    'Xlim', 1.2*[min(min([r(:,1) b(:,1)])), max(max([r(:,1), b(:,1)]))],...
    'Ylim', 1.2*[min(min([r(:,2) b(:,2)])), max(max([r(:,2), b(:,2)]))],...
    'DataAspectRatio', [1 1 1],...
    'TickLabelInterpret','LaTeX','FontSize',14);

if(isempty(te))
    te = inf;
end

robotColor = [0, 0.4 0.7];
bugColor = [0.9, 0.3 0.1];


Q = linspace(0, 2*pi, 15)';
circle = epsilon*[cos(Q) sin(Q)];

plot(ax,[-1 1 NaN 0 0], [0 0 NaN -1 1],'k');

bug = plot(ax, 0, 0, 'o', 'Color', bugColor,...
    'MarkerFaceColor', bugColor, 'MarkerSize', 12,...
    'DisplayName','Bug');
bugTrace = plot(ax, 0, 0, '-', 'Color', bugColor);

epsilonNeigborhood = plot(ax, 0, 0, 'k', 'LineWidth', 1);

robot = plot(ax, 0, 0, 's', 'Color', robotColor,...
    'MarkerFaceColor', robotColor, 'MarkerSize', 9,...
    'DisplayName','Robot');
robotTrace = plot(ax, 0, 0, '-', 'Color', robotColor);

legend([bug, robot], 'Location', 'northeast');

for k=1:length(t) 
    
    set(bug,'XData', b(k,1),'YData', b(k,2));
    
    if(t(k) <= te)
        set(bugTrace,'XData', b(1:k,1),'YData', b(1:k,2));
        set(epsilonNeigborhood,'XData', b(k,1) + circle(:,1),...
            'YData', b(k,2) + circle(:,2));
    else
        set(epsilonNeigborhood,'visible','off');
    end
    
    set(robot,'XData', r(k,1),'YData', r(k,2));
    set(robotTrace,'XData', r(1:k,1),'YData', r(1:k,2));
    
    pause(0.1);
end

end

