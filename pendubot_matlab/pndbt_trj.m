% ------------------------------------------------------------------------
% Energy based control of the pendubot
% ------------------------------------------------------------------------
% swing up initial condition is x0 = [-pi/2,0,0,0]' and 
% final condition is xT = [pi/2,0,0,0]';
% ------------------------------------------------------------------------
clc; clear;
run('pndbt_dnmcs.m')

% ------------------------------------------------------------------------
% System Initialization
% ------------------------------------------------------------------------

n = 50000; % number of iterations 
T = 5; % Final time of the simulation
delta_t = T/n; % iterations step
eta = 0.3146;
Ke = 1;  % Lyapunov constant 1
Kd = 0.9;  % Lyapunov constant 2
Kp = 5.5;  % Lyapunov constant 3

eta * Ke * plnr.theta(4)^2 * g^2;
g = 9.81;
theta_tot = [];
qd_tot = [];
u_tot = [];
L = [];
Energ = [];
E_top = (plnr.theta(4) + plnr.theta(5)) * g;
perb  = pi/10;
% q = [pi/2  - perb ; 0 + perb];
q = [pi/3 ; -pi/8];
q_d = [0; 0];

ode_optns_up = odeset('RelTol',1e-10,'AbsTol',1e-10);

if ~check_K(q, q_d, plnr, Ke, Kd, Kp, E_top)
    disp('the constants of the Lyapunov function candidate do not satisfy the conditions of convergence!!!'); 
else
    % ------------------------------------------------------------------------
    % System Energy Feedback Control 
    % ------------------------------------------------------------------------
    disp('calling ode45')
    for i= 1:n

        q_tld = q(1) - pi/2;    
        E =  Energie(q, q_d, plnr);      
        E_tld = E - E_top;
        F = plnr.theta(2) * plnr.theta(3) * sin(q(2)) * (q_d(1) + q_d(2))^2 + plnr.theta(3)^2 * cos(q(2)) * sin(q(2)) * ...
            q_d(1)^2 - plnr.theta(2) * plnr.theta(4) * cos(q(1)) * g + plnr.theta(3) * plnr.theta(5) * cos(q(2)) * cos(q(1) + q(2)) * g;

        u = (-Kd * F - (plnr.theta(1) * plnr.theta(2) - plnr.theta(3)^2 * cos(q(2))^2) * (q_d(1) + Kp * q_tld )) / ...
            ( (plnr.theta(1) * plnr.theta(2) - plnr.theta(3)^2 * cos(q(2))^2 ) * Ke * E_tld + Kd * plnr.theta(2) );

        % applying the control input to the dynamics
        odefun = @(t,x)ode_pendubot(t,x,u);
        [t,x] = ode45(odefun, [0,delta_t], [q;q_d], ode_optns_up);

        q = x(end,1:2)';
        q_d = x(end,3:4)';
        
        theta_tot = [theta_tot q];
        qd_tot = [qd_tot q_d];
        u_tot = [u_tot u];
        L = [L Lyap(q, q_d, plnr, Ke, Kd, Kp, E_top)];
        Energ = [Energ  Energie(q, q_d, plnr)];

    end
    pendubot_visualize(theta_tot(1:2,1:100:end),plnr)
    
    % ploting 
    figure
    plot(theta_tot(1, :)-pi/2, 'DisplayName', 'q1');
    hold on 
    plot(theta_tot(2, :), 'DisplayName', 'q2');
    legend

    figure
    plot(theta_tot(2, :), qd_tot(2, :), 'DisplayName', 'Phase portrait q2');
    legend

    figure
    plot(L, 'DisplayName', 'Lyapunov');
    legend
    
    figure
    plot(Energ-E_top, 'DisplayName', 'Energie tld')
    legend
end

% save('pdbt_T_60.mat', 'q_tot', 'L', 'Energ')


%% Partial Feedback Linearization Control

clc; clear;
run('pndbt_dnmcs.m')

% ------------------------------------------------------------------------
% System Initialization
% ------------------------------------------------------------------------
n = 20000; % number of iterations (outer loop)
m = 1; % rate of the inner loop
T = 2; % Final time of the simulation
delta_t = T/(n*m); % iterations step
Kp = 45;  % constant 2
Kd = 5.8;  % constant 3

g = 9.81;
theta_tot = []; 
qd_tot = [];
u_tot = [];
q = [-pi/2 ; 0];
q_d = [0; 0];
alpha = 0;
ode_optns_up = odeset('RelTol',1e-10,'AbsTol',1e-10);

% ------------------------------------------------------------------------
% Partial Feedback Linearization Control 
% ------------------------------------------------------------------------

for i= 1:n
    D = M_mtrx_fcn(q);
    C = C_mtrx_fcn(q, q_d);  
    g = g_vctr_fcn(q);
    d11 = D(1,1) - D(1,2) * D(2,1) / D(2,2);
    c11 = C(1,1) - D(1,2) * C(2,1) / D(2,2);
%     c12 = C(1,2) - D(1,2) * C(2,2) / D(2,2);
    c12 = C(1,2);
    phi1 = g(1) -  D(1,2) * g(2) / D(2,2);
    
    v1 = -Kd * q_d(1) + Kp * (pi/2  + alpha * atan(q_d(2)) - q(1));
    for j  = 1:m
        u = d11 * v1 + c11 * q_d(1) + c12 * q_d(2) + phi1 ; 

        % applying the control input to the dynamics
        odefun = @(t,x)ode_pendubot(t,x,u);
        [t,x] = ode45(odefun, [0,delta_t], [q;q_d], ode_optns_up);

        q = x(end,1:2)';
        q_d = x(end,3:4)';  

        theta_tot = [theta_tot q];
        qd_tot = [qd_tot q_d];
        u_tot = [u_tot u];
    end
end

pendubot_visualize(theta_tot(1:2,1:100:end),plnr)

figure
plot(theta_tot(1, :)-pi/2, 'DisplayName', 'q1');
hold on 
plot(theta_tot(2, :), 'DisplayName', 'q2');
legend













%% stablization of the pendubot around the top equlibrium position
clear; clc;
run('pndbt_dnmcs.m')

% ------------------------------------------------------------------------
% System Initialization
% ------------------------------------------------------------------------
n = 50000; % number of iterations 
T = 5; % Final time of the simulation
delta_t = T/n; % iterations step
g = 9.81; % gravity
q = [-pi/2; pi]; % intial positions 
q_d = [0; 0]; % initial velocities
q_s = [-pi/2; pi];  % stabilization point
up = 0;
ode_optns_up = odeset('RelTol',1e-10,'AbsTol',1e-10, 'Event', @(t,x)div_event_up(t,x));
ode_optns_down = odeset('RelTol',1e-10,'AbsTol',1e-10, 'Event', @(t,x)div_event_down(t,x));
theta_tot = []; 

% Weight matrices for states and inputs
% Q = [50,0,0,0;...
%      0,50,0,0;...
%      0,0,0.01,0;...
%      0,0,0,01];
% R = 0.01;
Q = [1,0,0,0;...
     0,1,0,0;...
     0,0,0,0;...
     0,0,0,0];
R = 0.1;

% Linearized system matrices 
A = A_mtrx_fcn(q, q_d,0);
B = B_vctr_fcn(q, q_d);

% Determining the coefficients of the LQR
[K,S,P] = lqr(A,B,Q,R)    

% ------------------------------------------------------------------------
% System Stablization  
% ------------------------------------------------------------------------
perturb = 0;
q = [-pi/2 + perturb; pi + perturb];
q_d = [0; 0];
for i= 1:n  
    u = -K * [q-q_s; q_d];
    if i > 30 && i<40
        u_ext = Jacob_fcn(q)' * [5; 0; 0];
        u  = u_ext(1);
    end
    % applying the control input to the dynamics
    odefun = @(t,x)ode_pendubot(t,x,u);
    if up
        [t,x,te,ye,ie] = ode45(odefun, [0,delta_t], [q;q_d], ode_optns_up);
    else
        [t,x,te,ye,ie] = ode45(odefun, [0,delta_t], [q;q_d], ode_optns_down);
    end
    
    if isempty(te)
        q = x(end,1:2)';
        q_d = x(end,3:4)';
        theta_tot = [theta_tot q];
    else
        disp('system diverging !!!')
        break
    end    
end
if isempty(te)
    pendubot_visualize(theta_tot(1:2,1:1000:end),plnr)
end








%% Transverse linearization of the pendubot around nominal trajectories

clear; clc;
run('pndbt_dnmcs.m')

% ------------------------------------------------------------------------
% System Initialization
% ------------------------------------------------------------------------
n = 50000; % number of iterations 
T = 5; % Final time of the simulation
delta_t = T/n; % iterations step
g = 9.81; % gravity

ode_optns = odeset('RelTol',1e-10,'AbsTol',1e-10);

theta_tot = []; 
theta_d_tot = []; 

% ------------------------------------------------------------------------
% alpha bera gamma equation dynamics   
% ------------------------------------------------------------------------
phi0 = -pi/2; 
thta0 = 0;
k  = 0.5;
theta = 0;
theta_d = -10;

for i= 1:n

    % applying the control input to the dynamics
    odefun = @(t,x)ode_alfa_bta_gma(t,x,k,phi0,thta0);
    [t,x] = ode45(odefun, [0,delta_t], [theta;theta_d], ode_optns);
    
    theta = x(end,1);
    theta_d = x(end,2);
    theta_tot = [theta_tot theta];
    theta_d_tot = [theta_d_tot theta_d];

end
q_tot = zeros(2, length(theta_tot));
q_tot(2,:) = theta_tot;
q_tot(1,:) = phi0 + k*(theta_tot - thta0);

figure
plot(theta_d_tot, theta_tot, 'DisplayName', 'Phase portrait');
legend

%pendubot_visualize(q_tot(1:2,1:1000:end),plnr)




