clear
close all
clc

% CasADi v3.5.5
addpath('/home/prenithreddy/Documents/prenith_ws/GNC/gnc_simulator/sim_ws/MATLAB/GNC_Algorithms/Control/MPC/casadi')

import casadi.*

T = 10; %[s]
N = 100; % prediction horizon
n = 0.001133; % [s^-1]

u_max = 1; u_min = -u_max;

x = SX.sym('x'); y = SX.sym('y'); z = SX.sym('z'); xDot = SX.sym('xDot'); yDot = SX.sym('yDot'); zDot = SX.sym('zDot');
states = [x;y;z;xDot;yDot;zDot]; n_states = length(states);

ux = SX.sym('ux'); uy = SX.sym('uy'); uz = SX.sym('uz');
controls = [ux;uy;uz]; n_controls = length(controls);
rhs = [states(4);states(5);states(6);3*n^2*states(1)+2*n*states(5)+controls(1);-2*n*states(4)+controls(2);-n^2*states(3)+controls(3)];

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);
% parameters (which include at the initial state of the robot and the reference state)

X = SX.sym('X',n_states,(N+1));
% A vector that represents the states over the optimization problem.

obj = 0; % Objective function
g = [];  % constraints vector

Q = 1e5*eye(n_states,n_states); % weighing matrices (states)
R = 1e1*eye(n_controls, n_controls ); % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g;st-P(1:n_states)]; % initial condition constraints (difference between estimated initial state and actual initial state)
for k = 1:N
    st = X(:,k);  con = U(:,k);  % state vector and control vector
    obj = obj+(st-P(n_states+1:2*n_states))'*Q*(st-P(n_states+1:2*n_states)) + con'*R*con; % calculate obj
    %obj = obj+(st-P(9*k-1:9*k+1))'*Q*(st-P(9*k-1:9*k+1)) + ...
    %          (con-P(9*k+2:9*k+3))'*R*(con-P(9*k+2:9*k+3)) ; % calculate obj
    % the number 9 is (n_states+n_controls)
    st_next = X(:,k+1);
    f_value = f(st,con);
    st_next_euler = st+ (T*f_value);
    g = [g;st_next-st_next_euler]; % compute constraints (multiple shooting constraint on dynamics)
end

% Add constraints for collision avoidance
% obs_x = 0; % meters
% obs_y = 0; % meters
% obs_z = 0; % meters
% obs_a = 10; % meters
% obs_b = 10; % meters
% obs_c = 10; % meters
% for k = 1:N+1   % box constraints due to the map margins
%     g = [g ; -(X(1,k)-obs_x)^2/(obs_a)^2 - (X(2,k)-obs_y)^2/(obs_b)^2 - (X(3,k)-obs_z)^2/(obs_c)^2 + 1];
% end

% make the decision variable one column  vector
OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 200;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
args.lbg(1:n_states*(N+1)) = 0; % equality constraints
args.ubg(1:n_states*(N+1)) = 0; % equality constraints

% args.lbg(n_states*(N+1)+1 : n_states*(N+1)+ (N+1)) = -500; % inequality constraints
% args.ubg(n_states*(N+1)+1 : n_states*(N+1)+ (N+1)) = 0; % inequality constraints
poslim = 15; % position limit
vellim = 0.1; % velocity limit
args.lbx(1:n_states:n_states*(N+1),1) = -poslim; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = poslim; %state x upper bound
args.lbx(2:n_states:n_states*(N+1),1) = -poslim; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = poslim; %state y upper bound
args.lbx(3:n_states:n_states*(N+1),1) = -poslim; %state z lower bound
args.ubx(3:n_states:n_states*(N+1),1) = poslim; %state z lower bound
args.lbx(4:n_states:n_states*(N+1),1) = -vellim; %state xdot lower bound
args.ubx(4:n_states:n_states*(N+1),1) = vellim; %state xdot upper bound
args.lbx(5:n_states:n_states*(N+1),1) = -vellim; %state ydot lower bound
args.ubx(5:n_states:n_states*(N+1),1) = vellim; %state ydot upper bound
args.lbx(6:n_states:n_states*(N+1),1) = -vellim; %state zdot lower bound
args.ubx(6:n_states:n_states*(N+1),1) = vellim; %state zdot upper bound

args.lbx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = u_min; %ux lower bound
args.ubx(n_states*(N+1)+1:n_controls:n_states*(N+1)+n_controls*N,1) = u_max; %ux upper bound
args.lbx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = u_min; %uy lower bound
args.ubx(n_states*(N+1)+2:n_controls:n_states*(N+1)+n_controls*N,1) = u_max; %uy upper bound
args.lbx(n_states*(N+1)+3:n_controls:n_states*(N+1)+n_controls*N,1) = u_min; %uz lower bound
args.ubx(n_states*(N+1)+3:n_controls:n_states*(N+1)+n_controls*N,1) = u_max; %uz upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
w = n/sqrt(2);
r = 10;
phi = 0;
theta = 45;
psi = 0;
dcm = [cos(psi)*cos(phi)-sin(psi)*sin(phi)*cos(theta), cos(psi)*sin(phi)+sin(psi)*cos(theta)*cos(phi), sin(psi)*sin(theta);
       -sin(psi)*cos(phi)-cos(psi)*sin(phi)*cos(theta), -sin(psi)*sin(phi)+cos(psi)*cos(theta)*cos(phi), cos(psi)*sin(theta);
        sin(theta)*sin(phi), -sin(theta)*cos(phi), cos(theta)];
x0 = [dcm*[r*cos(w*t0); r*sin(w*t0); 0.0;]; dcm*[-r*w*sin(w*t0); r*w*cos(w*t0); 0.0]];    % initial condition.
xs = [dcm*[r*cos(w*(t0+T)); r*sin(w*(t0+T)); 0.0]; dcm*[-r*w*sin(w*(t0+T)); r*w*cos(w*(t0+T)); 0.0]]; % Reference posture.
xxs(:,1) = xs;
xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,n_controls);        % three control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

sim_tim = 2*pi/w; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

% the main simulaton loop... it works as long as the number of mpc steps is
% less than its maximum value.

tic
while mpciter < sim_tim / T%) (norm((x0-xs),2) > 1e-2 &&
    args.p   = [x0;xs]; % set the values of the parameters vector
    % initial value of the optimization variables
    %----------------------------------------------------------------------
    args.x0  = [reshape(X0',n_states*(N+1),1);reshape(u0',n_controls*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)'; % get controls only from the solution
    xx1(:,1:n_states,mpciter+1)= reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; % get solution TRAJECTORY
    % subplot(3,1,1)
    % plot(u(:,1),'b')
    % subplot(3,1,2)
    % plot(u(:,2),'b')
    % subplot(3,1,3)
    % plot(u(:,3),'b')
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u,f);
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    xs = [dcm*[r*cos(w*(t0+T)) ; r*sin(w*(t0+T)); 0.0;]; dcm*[-r*w*sin(w*(t0+T)); r*w*cos(w*(t0+T)); 0.0]]; % Reference posture.
    xxs(:,mpciter+1) = xs;
    mpciter
    mpciter = mpciter + 1;
end
toc

ss_error = norm((x0-xs),2)

figure(1)
plot3(xx(1,:), xx(2,:), xx(3,:),'g','LineWidth',2)
hold on;
plot3(xxs(1,:), xxs(2,:), xxs(3,:),'r--','LineWidth',2)
% ellipsoid(obs_x, obs_y, obs_z, obs_a, obs_b, obs_c)
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
legend('Chaser Trajectory', 'Guidance Trajectory')
% for i=1:mpciter
%     plot3(xx(1,1:i), xx(2,1:i), xx(3,1:i),'r','LineWidth',6)
%     hold on;
%     drawnow; pause(0.0001);
% end


figure
subplot(3,1,1)
plot(u_cl(:,1),'b')
legend('X Control')
subplot(3,1,2)
plot(u_cl(:,2),'b')
legend('Y Control')
subplot(3,1,3)
plot(u_cl(:,3),'b')
legend('Z Control')
%Draw_MPC_PS_Obstacles (t,xx(3,:),xx1(:,3,:),u_cl,xs,N)