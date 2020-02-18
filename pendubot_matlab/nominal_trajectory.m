clc; clear all;  close all;


phi0 = pi/2; 
thta0 = pi;
k  = -2;
g = 9.81; % gravity

run('pndbt_dnmcs.m')

theta = 0.3;
theta_d = 0;

% ABG IN STATE SPACE FORM
dxdt = @(t,x)[x(2); (-gama_fcn(x(1)) - ...
    beta_fcn(x(1))*x(2)^2 )/alpha_fcn(x(1))];

% SOLVING  ABG FOR s

tspan= [0, 50];
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
        *x_iterp(i,2)^2)/alpha_fcn(x_iterp(i,1));
end

%{
figure
plot(x_iterp(:,1),x_iterp(:,2))
xlabel('$x$','Interpreter', 'latex')
ylabel('$\dot{x}$','Interpreter', 'latex')
grid on

fig = figure;
% fig.Units = 'centimeters';
% fig.InnerPosition = [10, 10, 8, 8]; %[left bottom width height]
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


q_tot = zeros(2, length(x_iterp));
q_tot(2,:) = x_iterp(:,1);
q_tot(1,:) = phi0 + k*(x_iterp(:,1) - thta0);

pendubot_visualize(q_tot(1:2,1:10:end),plnr)
%}
syms s
Phi = [phi0 + k * (s - thta0), s]';
Phi_prm = diff(Phi, s);

matlabFunction(Phi(1),'File','autogen/Phi_fcn','Vars',{s});
matlabFunction(Phi_prm(1),'File','autogen/Phi_prm_fcn','Vars',{s});


