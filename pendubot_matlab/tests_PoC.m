% -----------------------------------------------------------------------%
% Linearization of the dynamics along norminal trajectory
% -----------------------------------------------------------------------%
% Phi = f(s)   postion of the first link 
% Theta = s   position of the second link 

clc; clear all;  close all;

run('nominal_trajectory.m');
% return
redesign_controller = 1;
if redesign_controller
    
    run('generate_AB_mtrcs.m');
    run('design_controller_PoC.m');

else
        
    load('mat_files/K_mtrx.mat');
    K_gusev = K_mtrx;

end

% COMPUTE u FROM FORWARD DYNAMICS
u = zeros(length(x),1);
B = [1;0];

for i = 1:length(x)
   qq = [ phi0 + k*(x(i,1) - thta0); x(i,1)];
   
   qq_d = [k* x(i,2); x(i,2)];
   
   qq_2d = [k * x_2d(i); x_2d(i)];
   
   M = M_mtrx_fcn(qq);
   C = C_mtrx_fcn(qq, qq_d);  
   G = g_vctr_fcn(qq);
   
   u(i) = [1, 0]*(M * qq_2d + C * qq_d +G);

end

% Find one period of s 
% [~,locs] = findpeaks(mod(x(:, 1),2*pi));
[~,locs] = findpeaks(x(:, 1));
T = t(locs(2)) - t(locs(1));
s_str = x(locs(1):locs(2),1);
s_d_str = x(locs(1):locs(2),2);
s_2d_str = x_2d(locs(1):locs(2));

Phi_str = phi0 + k * (s_str - thta0);
Phi_d_str = k * (s_d_str);
% Phi_str = mod(Phi_str,2*pi);

q_str = [ Phi_str s_str Phi_d_str s_d_str];
save('mat_files/q_str3.mat','q_str')

% COMPUTE NOMINAL TORQUES FROM U_ff
s = x(:,1);
s_d = x(:,2);

% u_str = U_ff(s,s_d,0,0);
u_str = U_full(s,s_d,0,0,0);

% COMPARE TORQUES
if norm(u - u_str)<1e-6
    disp('TORQUES ARE CONSISTENT: ')
    disp('u = U(s,s_d,y,y_d)')
else
    disp('TORQUES ARE NOT CONSISTENT!!! ')
    return
end


%% INITIAL CONDITION AND PARAMETERS OF SIMULATION
clc; close all;
optns_id = odeset('RelTol',1e-12,'AbsTol',1e-12,'NormControl','on');
% Function that finds zero crossing
zci = @(v) find(v(:).*circshift(v(:), [-1 0]) <= 0);

n_T = length(s_str);
n_iter = 700;
add_distr = 0;

if add_distr
    dlta_x0 = [-0.05 0.02 0.05 0.01]';
else
    dlta_x0 = zeros(4,1);
end

% Initial condition of the nominal trajectory
thta_0 = 1.20;    
thta_d_0 = 0.0;

x0 = [Phi_fcn(thta_0) , thta_0 ,...
        Phi_prm_fcn(thta_0) * thta_d_0, thta_d_0]';

x0 = [Phi_fcn(x(locs(1),1)) , x(locs(1),1) ,...
        Phi_prm_fcn(x(locs(1),1)) * x(locs(1),2), x(locs(1),2)]';

% x0 = [Phi_fcn(x(1,1)) , x(1,1) ,...
%     Phi_prm_fcn(x(1,1)) * x(1,2), x(1,2)]';

x0_dstbd = x0 + dlta_x0;
% x0_dstbd = [-pi/2 0 0.5 8.0];  


%{
%% CONTROLLING LINEARIZED SYSTEM
% Generator of motion and its deriative
s_0 = x0_dstbd(2);
s_d_0 = x0_dstbd(4);

I_0 = Intg(s_0,s_d_0, s_str(1),s_d_str(1));
y_0 = x0_dstbd(1) - Phi_fcn(s_0);
y_d_0 = x0_dstbd(3) - Phi_prm_fcn(s_0)*s_d_0;

x_trsv = zeros(n_iter+1,3);
x_trsv(1,:) = [I_0,y_0,y_d_0];

for i = 1:n_iter
    t_span = [t(i), t(i+1)];
    
    j = 1+mod(i-1,n_T);
    % Time varying matrices A and B for nominal trajectory 
    A_lin = A_mtrx(s_str(j),s_d_str(j));
    B_lin = B_mtrx(s_str(j),s_d_str(j));
    u_cur = K_mtrx(:,:,j)*x_trsv(i,:)';
    [~,x_cur] = ode45( @(t,x)pndbt_lnrzd_dnmcs(t,x,A_lin,B_lin,u_cur), ...
                            t_span,x_trsv(i,:)',optns_id);
    x_trsv(i+1,:) = x_cur(end,:);
    clear x_cur
end

figure
subplot(3,1,1)
    plot(t(1:n_iter),x_trsv(1:n_iter,1))
    ylabel('$I$','interpreter','latex')
    grid minor
subplot(3,1,2)
    plot(t(1:n_iter),x_trsv(1:n_iter,2))
    ylabel('$y$','interpreter','latex')
    grid minor
subplot(3,1,3)
    plot(t(1:n_iter),x_trsv(1:n_iter,3))
    ylabel('$\dot{y}$','interpreter','latex')
    grid minor
%}

% CONTROLLING FULL NONLINEAR SYSTEM
% Allocate variables

x_inv_dnmcs_dstbd = zeros(n_iter+1,4);
y = zeros(n_iter,1);
y_d = zeros(n_iter,1);
I = zeros(n_iter,1);

u = zeros(n_iter,1);
% Set initial condition
x_inv_dnmcs_dstbd(1,:) = x0_dstbd;

for i = 1:n_iter
    t_span = [t(i), t(i+1)];
    
    % Generator of motion and its deriative
    s_cur = x_inv_dnmcs_dstbd(i,2);
    s_d_cur = x_inv_dnmcs_dstbd(i,4);
    
    q_cur = x_inv_dnmcs_dstbd(i,:);
    
    dlta_V = [s_cur, s_d_cur]-[s_str, s_d_str];
    vec_nrm = vecnorm(dlta_V,2,2);
    [~,idx] = min(vec_nrm);
    
    dlta_q = q_cur - q_str;
    dlta_q_nrm = vecnorm(dlta_q,2,2);
    [~,idx2] = min(dlta_q_nrm);

    % Compute Transverse coordinates
    I_cur = Intg(s_cur,s_d_cur, s_str(1),s_d_str(1));
    y_cur = x_inv_dnmcs_dstbd(i,1) - Phi_fcn(s_cur);
    y_d_cur = x_inv_dnmcs_dstbd(i,3) - Phi_prm_fcn(s_cur)*s_d_cur;
    x_trsv_cur = [I_cur,y_cur,y_d_cur]'; 
    
    y(i) = y_cur;
    y_d(i) = y_d_cur;
    I(i) = I_cur;    
    % Input that consistes of feedfowfard term and feedback terms
%     u_ffrd = U_ff(s_cur,s_d_cur,y_cur,y_d_cur)'; 
    u_fbck = K_mtrx(:,:,idx) * x_trsv_cur;
%     u_cur = u_ffrd + inv(N_fcn(s_cur,s_d_cur)) * u_fbck;
    u_cur = U_full(s_cur,s_d_cur,y_cur,y_d_cur,u_fbck);
    
    u(i) = u_cur;
    
    % Integration of dynamics from t_{i} to t_{i+1} with u = u_{i}
    [~,x_cur] = ode45( @(t,x) pndbt_dnmcs(t,x,u_cur),...
                        t_span,x_inv_dnmcs_dstbd(i,:)',optns_id);
   
    x_inv_dnmcs_dstbd(i+1,:) = x_cur(end,:);
end


figure
subplot(3,1,1)
    plot(t(1:n_iter),I(1:n_iter))
    ylabel('$I$','interpreter','latex')
    grid on
    grid minor
subplot(3,1,2)
    plot(t(1:n_iter),y(1:n_iter))
    ylabel('$y$','interpreter','latex')
    grid on
    grid minor
subplot(3,1,3)
    plot(t(1:n_iter),y_d(1:n_iter))
    ylabel('$\dot{y}$','interpreter','latex')
    grid on
    grid minor

figure
scatter(x_inv_dnmcs_dstbd(1,1), x_inv_dnmcs_dstbd(1,3),'DisplayName', 'staring point')
hold on
plot(Phi_str,Phi_d_str,'k','LineWidth',2,'DisplayName', 'Phase portrait Phi')
plot(x_inv_dnmcs_dstbd(:,1),x_inv_dnmcs_dstbd(:,3),'DisplayName', 'Phase portrait Phi dnmcs')
scatter(x_inv_dnmcs_dstbd(end,1), x_inv_dnmcs_dstbd(end,3), 'DisplayName','ending point')
legend
    
figure
scatter(x_inv_dnmcs_dstbd(1,2), x_inv_dnmcs_dstbd(1,4), 'DisplayName','starting point')
hold on
plot(s_str,s_d_str,'k','LineWidth',2,'DisplayName', 'Phase portrait Theta')
plot(x_inv_dnmcs_dstbd(:,2),x_inv_dnmcs_dstbd(:,4),'DisplayName', 'Phase portrait Theta dnms')
scatter(x_inv_dnmcs_dstbd(end,2), x_inv_dnmcs_dstbd(end,4), 'DisplayName','ending point')
legend

figure
plot(t(1:n_iter),u)
%}

pendubot_visualize(x_inv_dnmcs_dstbd(1:10:end,1:2)',plnr)


%% FUNCTIONS

function   dxdt = pndbt_dnmcs(t,x,u)
    M = M_mtrx_fcn(x(1:2));
    C = C_mtrx_fcn(x(1:2), x(3:4));  
    G = g_vctr_fcn(x(1:2));
        
    B = [1;0];
    
    dxdt = zeros(4,1);
    dxdt(1:2) = x(3:4);
    dxdt(3:4) = M\(B * u - C * x(3:4) - G);
end

function dxdt = pndbt_lnrzd_dnmcs(t,x_trsv,A,B,u)
    dxdt = zeros(3,1);
    dxdt = A * x_trsv + B * u;
end

