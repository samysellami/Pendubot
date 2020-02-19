%clc; clear all;  close all;

% parameters of alpha beta gamma equation
% phi0 = -pi/2; 
% thta0 = 0;
% k  = 0.5;

run('pndbt_dnmcs.m')

% theta =  1;
% theta_d = 0.0;

% ABG IN STATE SPACE FORM
dxdt = @(t,x)[x(2); (-gama_fcn(x(1)) - ...
    beta_fcn(x(1))*x(2)^2 )/alpha_fcn(x(1))];

% SOLVING  ABG FOR s

tspan= [0, 20];
optns = odeset('RelTol',1e-9,'AbsTol',1e-9,'NormControl','on');
x0 = [theta; theta_d];

[t,x] = ode45( @(t,x)dxdt(t,x),tspan,x0,optns);

% Interpolating results
t_iterp = (t(1):1e-2:t(end))';
x_iterp(:,1) = interp1(t,x(:,1),t_iterp);
x_iterp(:,2) = interp1(t,x(:,2),t_iterp);
x_2d = zeros(length(x_iterp(:,1)),1);

for i=1:length(x_iterp(:,1))
    x_2d(i) = (-gama_fcn(x_iterp(i,1)) - beta_fcn(x_iterp(i,1))...
        * x_iterp(i,2)^2)/alpha_fcn(x_iterp(i,1));
end

% ploting phase portrait
figure
plot(x_iterp(:,1),x_iterp(:,2))
xlabel('$x$','Interpreter', 'latex')
ylabel('$\dot{x}$','Interpreter', 'latex')
grid on

%{
fig = figure;
fig.GraphicsSmoothing = 'on';
subplot(3,1,1)
    plot(t_iterp,x_iterp(:,1),'k','LineWidth',1.5)
    xlabel('time,\ s','Interpreter', 'latex')
    ylabel('$x$','Interpreter', 'latex')
    grid on
subplot(3,1,2)
    plot(t_iterp,x_iterp(:,2),'k','LineWidth',1.5)
    xlabel('$t,\ s$','Interpreter', 'latex')
    ylabel('$\dot{x}$','Interpreter', 'latex')
    grid on
subplot(3,1,3)
plot(t_iterp,x_2d,'k','LineWidth',1.5)
xlabel('$t,\ s$','Interpreter', 'latex')
ylabel('$\ddot{x}$','Interpreter', 'latex')
grid on
%}

q_tot = zeros(2, length(x_iterp));
q_tot(2,:) = x_iterp(:,1);
q_tot(1,:) = phi0 + k*(x_iterp(:,1) - thta0);

% visualise the pendubot trajectory
%pendubot_visualize(q_tot(1:2,1:10:end),plnr)

