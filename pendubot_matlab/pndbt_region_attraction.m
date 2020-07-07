%% Pendubot region of attraction 
clc; clear; close all; 

run('pndbt_dnmcs.m');

%%
% ------------------------------------------------------------------------
% controller deisgn
% ------------------------------------------------------------------------

clc; 
controller_types = {'pp', 'lqr'};
controller = controller_types{2};

q = [-pi/2; pi]; % equilibrium position
q_d = [0; 0]; % equilibrium velocity
x_eq = [q;q_d];

% Linearized system matrices 
A = A_mtrx_fcn(q, q_d,0);
B = B_vctr_fcn(q, q_d);

% Controller coefficients
if strcmp(controller, 'pp')
    % desing state feedback controller using pole placement
    poles = [-2, -3, -4, -5];
    K = place(A, B, poles);

elseif strcmp(controller, 'lqr')
    % design lqr controller
    Q = [1,0,0,0;...
         0,1,0,0;...
         0,0,0,0;...
         0,0,0,0];
    R = 0.1;
    [K, P, ~] = lqr(A, B, Q, R);
end

if strcmp(controller, 'pp')
    % Closed loop system
    A_tilde = A - B*K;
    % Solve continous lyapunov equation for linearized system
    % V = delta_x' P delta_x   AP + PA' + Q = 0
    Q = [1,0,0,0;...
         0,1,0,0;...
         0,0,0,0;...
         0,0,0,0];
    P = lyap(A_tilde', Q);
end    

% domain of attraction wiht respect to theta and theta_d
dom_att = [];
V_d_tot = [];
theta_range = pi-pi/20:0.01:pi+pi/20;
theta_d_range = -10:0.1:10;

for theta = theta_range
   for theta_d =  theta_d_range
        x = [-pi/2; theta; 0; theta_d];
        u = -K * (x - x_eq);
        V_d = lyap_deriv_function(x,P,x_eq,u);
        V = lyap_function(x,P,x_eq);
        if V_d < 0 && V>0
            dom_att = [dom_att x];
            V_d_tot = [V_d_tot V_d];
        end
   end
end

%{
figure
MARKER_SIZE = 10; %marker size of the scatter function
scatter(dom_att(2,:), dom_att(4,:),MARKER_SIZE,'k','filled', 'DisplayName', 'domaine of attraction')
hold on
scatter(pi, 0, 100,'r','filled','DisplayName', 'equilibrium')
xlabel('$\theta$','interpreter','latex')
ylabel('$\dot{\theta}$','interpreter','latex')
xlim([theta_range(1)  theta_range(end)])
ylim([theta_d_range(1)  theta_d_range(end)])
legend
grid on
%}

%%
clc; close all;
% System Initialization
n = 5000; % number of iterations 
T = 5; % Final time of the simulation
delta_t = T/n; % iterations step
q_s = [-pi/2; pi];  % stabilization point
q_d = [0; 0];
q_tot = [];
ode_optns = odeset('RelTol',1e-10,'AbsTol',1e-10);
ode_optns_down = odeset('RelTol',1e-10,'AbsTol',1e-10, 'Event', @(t,x)div_event_down(t,x));

thta_lim = 0.1;
phi_lim = 0.1;

thta0 = -thta_lim:0.1:thta_lim;
phi0 = -phi_lim:0.1:phi_lim;
figure

for thta= thta0
    for phi= phi0
        q = [-pi/2 + phi; pi + thta];
        for i= 1:n  
            u = -K * [q-q_s; q_d];    
            % applying the control input to the dynamics
            odefun = @(t,x)ode_pendubot(t,x,u);
            [t,x,te,ye,ie] = ode45(odefun, [0,delta_t], [q;q_d], ode_optns_down);
            if isempty(te)
                q = x(end,1:2)';
                q_d = x(end,3:4)';
                q_tot = [q_tot q];
            else
                disp('system diverging !!!')
                break
            end    
        end
        figure
        plot(q_tot(1,:)+pi/2,'b')
        hold on        
        plot(q_tot(2,:)-pi,'b')
        figure
        plot(q_tot(1,:)+pi/2, q_tot(2,:)-pi,'b')
        %axis([-phi_lim phi_lim -thta_lim thta_lim])
        grid on; drawnow; pause(5e-1);
    end
end

%%
clc; close all;
for ii = 1:1
    qq = rand(4);
    Q = qq*qq';
%     P = lyap(A', Q);
    [R,D] = eig(P);

    c = 0.01;
    stable = 1;
    while stable
        a = sqrt(c/D(1,1));b = sqrt(c/D(2,2));
        d = sqrt(c/D(3,3));e = sqrt(c/D(4,4));
        r_tot = [];
 
        for phi1 = 0:0.01:pi
            for phi2 = 0:0.01:pi
                for phi3 = 0:0.01:2*pi
                    r = R*[a.*cos(phi1); b.*sin(phi1).*cos(phi2) ; ...
                       d.*sin(phi1).*sin(phi2).*cos(phi3); e.*sin(phi1).*sin(phi2).*sin(phi3)];
                    r_tot = [r_tot r];
                    x = r(:,i);
                    u = -K * (x - x_eq);
                    if lyap_deriv_function(x,P,x_eq,u) > 0 
                        stable = 0;
                        break	
                    end
                end
            end
        end
        c = c+ 0.01;
    end
    plot(r_tot(1,:),r_tot(2,:),'LineWidth',2,'Color','red')
    hold on
end


%% ------------------------------------------------------------------------
% System Visualization
% ------------------------------------------------------------------------
clc;

% System Initialization
n = 5000; % number of iterations 
T = 5; % Final time of the simulation
delta_t = T/n; % iterations step
q_s = [-pi/2; pi];  % stabilization point

ode_optns_down = odeset('RelTol',1e-10,'AbsTol',1e-10, 'Event', @(t,x)div_event_down(t,x));

q_tot = []; 
% ------------------------------------------------------------------------
% System Stablization  
% ------------------------------------------------------------------------
perturb = 0.01;
q = [-pi/2 + perturb; pi + perturb];
q_d = [0; 0];
ind = 10;
q = dom_att(1:2,ind);
q_d = dom_att(3:4,ind);

for i= 1:n  
    u = -K * [q-q_s; q_d];    
    % applying the control input to the dynamics
    odefun = @(t,x)ode_pendubot(t,x,u);
    [t,x,te,ye,ie] = ode45(odefun, [0,delta_t], [q;q_d], ode_optns_down);

    if isempty(te)
        q = x(end,1:2)';
        q_d = x(end,3:4)';
        q_tot = [q_tot q];
    else
        disp('system diverging !!!')
        break
    end    
end
pendubot_visualize(q_tot(1:2,1:1000:end),plnr)
% close all;


function V = lyap_function(x,P,x_eq)
    V = (x-x_eq)' * P * (x-x_eq);
end

function V_d = lyap_deriv_function(x,P,x_eq,u)
    V_d  = f_fcn(x(1:2), x(2:3),u)' * P * (x-x_eq) + (x-x_eq)' * P * f_fcn(x(1:2), x(2:3),u);
end




