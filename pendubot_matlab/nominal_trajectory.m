% clc; clear all;  close all;
% traj = 3;
% freq = 100;

if traj == 2
    % second trajectory with initial conditions thta0 = 0 thta_d0 = 9.69 
    % phi = -pi/2 and phi_d = 0.969
    phi0 = -pi/2; 
    thta0 = 0.0;
    k  = 0.0;
    theta =  0;
    theta_d = 15;
elseif traj == 1
    % first trajectory with initial conditions thta0 =1.2 thta_d0= 0
    phi0 = -pi/2; 
    thta0 = 0.0;
    k  = 0.1;
    theta =  2.5;
    theta_d = 0;
elseif traj == 0
    % first trajectory with initial conditions thta0 =1.2 thta_d0= 0
    phi0 = -pi/2; 
    thta0 = 0.0;
    k  = 0.1;
    theta =  2.0;
    theta_d = 0;
elseif traj == -1
    % first trajectory with initial conditions thta0 =1.2 thta_d0= 0
    phi0 = -pi/2; 
    thta0 = 0.0;
    k  = 0.1;
    theta =  1.5;
    theta_d = 0;
elseif traj == 3
    % first trajectory with initial conditions thta0 =1.2 thta_d0= 0
    phi0 = -pi/2; 
    thta0 = pi;
    k  = -1.5;
    theta =  pi-0.5;
    theta_d = 0;
end

run('pndbt_dnmcs.m')

% ABG IN STATE SPACE FORM
dxdt = @(t,x)[x(2); (-gama_fcn(x(1)) - ...
    beta_fcn(x(1))*x(2)^2 )/alpha_fcn(x(1))];


% SOLVING  ABG FOR s
% tspan= 0:2e-2:15;
tspan= 0:(1/freq):15;
optns = odeset('RelTol',1e-9,'AbsTol',1e-9,'NormControl','on');
x0 = [theta; theta_d];

[t,x] = ode45( @(t,x)dxdt(t,x),tspan,x0,optns);
% x(:, 1) = wrapToPi2(x(:, 1));

x_2d = zeros(length(x(:,1)),1);

for i=1:length(x(:,1))
    x_2d(i) = (-gama_fcn(x(i,1)) - beta_fcn(x(i,1))...
        * x(i,2)^2)/alpha_fcn(x(i,1));
end

%{
% ploting phase portrait
figure
plot(x(:,1),x(:,2))
xlabel('$\theta$','Interpreter', 'latex')
ylabel('$\dot{\theta}$','Interpreter', 'latex')
grid on
hold on

fig = figure;
fig.GraphicsSmoothing = 'on';
subplot(3,1,1)
    plot(t,x(:,1),'k','LineWidth',1.5)
    xlabel('time,\ s','Interpreter', 'latex')
    ylabel('$x$','Interpreter', 'latex')
    grid on
subplot(3,1,2)
    plot(t,x(:,2),'k','LineWidth',1.5)
    xlabel('$t,\ s$','Interpreter', 'latex')
    ylabel('$\dot{x}$','Interpreter', 'latex')
    grid on
subplot(3,1,3)
plot(t,x_2d,'k','LineWidth',1.5)
xlabel('$t,\ s$','Interpreter', 'latex')
ylabel('$\ddot{x}$','Interpreter', 'latex')
grid on
%}

q_tot = zeros(2, length(x));

q_tot(2,:) = x(:,1);
q_tot(1,:) = phi0 + k*(x(:,1) - thta0);

% visualise the pendubot trajectory
% pendubot_visualize(q_tot(1:2,1:10:end),plnr)


return

%% ploting the phase portrait space
clc; close all; 
i = 1;

for theta = -10:1:10
    for theta_d = -20:1:20
        % tspan= 0:2e-2:15;
        tspan= 0:(1/freq):15;
        optns = odeset('RelTol',1e-9,'AbsTol',1e-9,'NormControl','on');
        x0 = [theta; theta_d];

        [t,x] = ode45( @(t,x)dxdt(t,x),tspan,x0,optns);
        x_(:,:,i) = x;
        i = i+1;    
    end
end
    
figure
for i = 1:size(x_, 3)  
    if max(x_(:,1,i))< 10 & min(x_(:,1,i))> -10 ... 
            & max(x_(:,2,i)) < 20 & min(x_(:,2,i)) > -20
        y = x_(:,1,i);
        plot(x_(:,1,i),x_(:,2,i))
        xlabel('$\theta$','Interpreter', 'latex')
        ylabel('$\dot{\theta}$','Interpreter', 'latex')
        xlim([-2  10])
        ylim([-18 18])
        grid on
        hold on
    end
end

%%
clc; close all; 
i = 1;

for theta = -50:1:50
    for theta_d = -30:1:30
        % tspan= 0:2e-2:15;
        tspan= 0:(1/freq):15;
        optns = odeset('RelTol',1e-9,'AbsTol',1e-9,'NormControl','on');
        x0 = [theta; theta_d];

        [t,x] = ode45( @(t,x)dxdt(t,x),tspan,x0,optns);
        x_(:,:,i) = x;
        i = i+1;    
    end
end
    
figure
for i = 1:size(x_, 3)  
    if max(x_(:,1,i))< 50 & min(x_(:,1,i))> -50 ... 
            & max(x_(:,2,i)) < 30 & min(x_(:,2,i)) > -30
        y = x_(:,1,i);
        plot(x_(:,1,i),x_(:,2,i))
        xlabel('$\theta$','Interpreter', 'latex')
        ylabel('$\dot{\theta}$','Interpreter', 'latex')
        xlim([0  6.2])
        ylim([-18 18])
        grid on
        hold on
    end
end