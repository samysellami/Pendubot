clc; clear all;  close all;

% parameters of alpha beta gamma equation


% second trajectory with initial conditions thta0 = 0 thta_d0 = 7.16 
phi0 = -pi/2; 
thta0 = 0.0;
k  = 0.0;
theta =  0;
theta_d = 15;

% first trajectory with initial conditions thta0 =1.2 thta_d0= 0
% phi0 = -pi/2; 
% thta0 = 0.0;
% k  = 0.5;
% theta =  1.2;
% theta_d = 0;

run('pndbt_dnmcs.m')


% ABG IN STATE SPACE FORM
dxdt = @(t,x)[x(2); (-gama_fcn(x(1)) - ...
    beta_fcn(x(1))*x(2)^2 )/alpha_fcn(x(1))];

% SOLVING  ABG FOR s

tspan= 0:1e-2:15;
optns = odeset('RelTol',1e-9,'AbsTol',1e-9,'NormControl','on');
x0 = [theta; theta_d];

[t,x] = ode45( @(t,x)dxdt(t,x),tspan,x0,optns);
x(:, 1) = wrapToPi2(x(:, 1));


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
