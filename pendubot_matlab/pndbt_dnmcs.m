% clc; clear all; close all;

% Path to the urdf file 
% addpath(genpath('/home/sami/Documents/MasterThesis/Bot pendulum//BotPendulum_Energy based control/pdbt_files/utils'))
% addpath(genpath('/home/sami/Documents/MasterThesis/Bot pendulum//BotPendulum_Energy based control/pdbt_files/planar2DOF'))

%-------------------------------------------------------------------------
% Loading file from urdf
% ------------------------------------------------------------------------
plnr = xml2struct('planar_manip.urdf');

% ------------------------------------------------------------------------
% Extracting parameters of the robot
% ------------------------------------------------------------------------
for i = 1:2
% axis of rotation of a joint i in coordinate system of joint i    
   axis_of_rot = str2num(plnr.robot.joint{i}.axis.Attributes.xyz)';
   link_mass = str2double(plnr.robot.link{i+1}.inertial.mass.Attributes.value);
% poistion of the com in frame attached to link
   com_pos = str2num(plnr.robot.link{i+1}.inertial.origin.Attributes.xyz)';
% inertial parameters of the link expressed in coordinate system attached
% the center of mass.
   ixx = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.ixx);
   ixy = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.ixy);
   ixz = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.ixz);
   iyy = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.iyy);
   iyz = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.iyz);
   izz = str2double(plnr.robot.link{i+1}.inertial.inertia.Attributes.izz);
% the inertia tensor wrt the frame oriented as the body frame and with the
% origin in the COM
   link_inertia = [ixx, ixy, ixz; ixy, iyy iyz; ixz, iyz, izz];
% manipulator regressor                               
   plnr.m(i) = sym(link_mass);
   plnr.k(:,i) = sym(axis_of_rot);
   plnr.r_com(:,i) = sym(com_pos);
   plnr.I(:,:,i) = sym(link_inertia);
end

%{
% syms m1 m2 l1 l2 lc1 lc2 I1 I2 real
% 
% plnr.m(1) = m1; 
% plnr.m(2) = m2; 
% plnr.I(:,:,1) = [0 0 0 ; 0 0 0; 0 0 I1];
% plnr.I(:,:,2) = [0 0 0 ; 0 0 0; 0 0 I2];
% plnr.r_com(:,1) = [lc1; 0; 0];
% plnr.r_com(:,2) = [lc2; 0; 0];
% syms theta1 theta2 theta3 theta4 theta5
% plnr.theta(1) = theta1;
% plnr.theta(2) = theta2;
% plnr.theta(3) = theta3;
% plnr.theta(4) = theta4;
% plnr.theta(5) = theta5;

% % parammeters of the book non linear control of underactuated mechanical systems  
% plnr.theta(1) = 0.034;
% plnr.theta(2) = 0.0125;
% plnr.theta(3) = 0.01;
% plnr.theta(4) = 0.215; 
% plnr.theta(5) = 0.073;

% parammeters of the article Hybrid Control for the Pendubot N°1
% plnr.theta(1) = 0.0308;
% plnr.theta(2) = 0.0106;
% plnr.theta(3) = 0.0095;
% plnr.theta(4) = 0.2086; 
% plnr.theta(5) = 0.0630;

% parammeters of the article Hybrid Control for the Pendubot N°2
% plnr.theta(1) = 0.0260;
% plnr.theta(2) = 0.0119;
% plnr.theta(3) = 0.0098;
% plnr.theta(4) = 0.1673; 
% plnr.theta(5) = 0.0643;

% parammeters of the article THE SWING U P CONTROL FOR THE PENDUBOT
% plnr.theta(1) = 0.0799;
% plnr.theta(2) = 0.0244;
% plnr.theta(3) = 0.0205;
% plnr.theta(4) = 0.42126; 
% plnr.theta(5) = 0.10630;
%}

l1 =  str2num(plnr.robot.link{1,2}.visual.geometry.cylinder.Attributes.length);
plnr.theta(1) = vpa( plnr.m(1) * norm(plnr.r_com(:,1))^2 + plnr.m(2) * l1^2 + plnr.I(3,3,1)  );
plnr.theta(2) = vpa( plnr.m(2) * norm(plnr.r_com(:,2))^2 + plnr.I(3,3,2) ) ;
plnr.theta(3) = vpa( plnr.m(2) * l1 * norm(plnr.r_com(:,2)) );
plnr.theta(4) = vpa( plnr.m(1) * norm(plnr.r_com(:,1)) + plnr.m(2) * l1 ); 
plnr.theta(5) = vpa( plnr.m(2) * norm(plnr.r_com(:,2)) );


% ------------------------------------------------------------------------
% Symbolic generilized coordiates, their first and second deriatives
% ------------------------------------------------------------------------
q_sym = sym('q%d',[2,1],'real');
qd_sym = sym('qd%d',[2,1],'real');
q2d_sym = sym('q2d%d',[2,1],'real');
syms u;
g  = 9.81;
%------------------------------------------------------------------------
% Dynamics of the robot in classical form for control
%------------------------------------------------------------------------

T_0k(:,:,1) = sym(eye(4));
Jv_0k = sym(zeros(3,2,2));
Jw_0k = sym(zeros(3,2,2));
K = sym(0); P = sym(0);

for i = 1:2
        % Transformation from parent link frame p to current joint frame
        R_pj = sym(RPY(str2num(plnr.robot.joint{i}.origin.Attributes.rpy)));
        p_pj = sym(str2num(plnr.robot.joint{i}.origin.Attributes.xyz))';
        R_pj(abs(R_pj)<1e-5) = sym(0);
        T_pj = [R_pj, p_pj; sym(zeros(1,3)), sym(1)];
        % Tranformation from joint frame of the joint that rotates body k to
        % link frame. The transformation is pure rotation
        R_jk = Rot(q_sym(i),plnr.k(:,i));
        p_jk = sym(zeros(3,1));
        T_jk = [R_jk, p_jk; sym(zeros(1,3)),sym(1)];
        % Transformation from parent link frame p to current link frame k
        T_pk(:,:,i) = T_pj*T_jk;
        T_0k(:,:,i+1) = T_0k(:,:,i)*T_pk(:,:,i);
        z_0k(:,i) = T_0k(1:3,1:3,i+1)*plnr.k(:,i);
        
        r_0k(:,i) = sym([eye(3),zeros(3,1)])*...
                        T_0k(:,:,i+1)*[plnr.r_com(:,i);sym(1)];
        
        for j = 1:i
           Jv_0k(:,j,i) = cross(z_0k(:,j),r_0k(:,i)-T_0k(1:3,4,j+1));
           Jw_0k(:,j,i) = z_0k(:,j);
        end
        
        K = K + sym(0.5)*qd_sym'*(plnr.m(i)*(Jv_0k(:,:,i)'*Jv_0k(:,:,i)) + ...
                Jw_0k(:,:,i)'*T_0k(1:3,1:3,i+1)*plnr.I(:,:,i)*...
                T_0k(1:3,1:3,i+1)'*Jw_0k(:,:,i))*qd_sym;
        P = P + plnr.m(i)*sym([0;0;9.81])'*r_0k(:,i);
end

Lagr = K - P;
dLagr_dq = jacobian(Lagr,q_sym)';
dLagr_dqd = jacobian(Lagr,qd_sym)';
t1 = jacobian(dLagr_dqd,[q_sym;qd_sym])*[qd_sym; q2d_sym];
t1 = simplify(t1);
dnmcs = t1 - dLagr_dq;
M_sym = jacobian(dnmcs,q2d_sym);
n_sym = simplify(dnmcs - M_sym*q2d_sym);

% matlabFunction(M_sym,'File','autogen/M_mtrx_fcn','Vars',{q_sym});
% matlabFunction(n_sym,'File','autogen/n_vctr_fcn','Vars',{q_sym,qd_sym});

%-------------------------------------------------------------------------
% linearization of the system about the top position
% ------------------------------------------------------------------------
% d/dt x(t) = A x(t) + B u(t)   where: x  = [q1 q2 q_d1 q_d2]
%                                      d/dt x(t) = f(x, u)
%                                      A  = d/dx f(x,u)    B = d/du f(x, u)
    

D_thta = [plnr.theta(1) + plnr.theta(2) + 2 * plnr.theta(3) * cos(q_sym(2)), ...
          plnr.theta(2) + plnr.theta(3) * cos(q_sym(2)); ...
          plnr.theta(2) + plnr.theta(3) * cos(q_sym(2)), plnr.theta(2)];

C_thta = [-plnr.theta(3)* sin(q_sym(2)) * qd_sym(2), ...
          -plnr.theta(3)* sin(q_sym(2)) * qd_sym(2) - plnr.theta(3)* sin(q_sym(2)) * qd_sym(1); ...
           plnr.theta(3)* sin(q_sym(2)) * qd_sym(1), 0];

g_thta = [plnr.theta(4) * g * cos(q_sym(1)) + plnr.theta(5) * g * cos(q_sym(1) + q_sym(2)); ...
          plnr.theta(5) * g * cos(q_sym(1) + q_sym(2)) ];

n_thta =  C_thta * qd_sym + g_thta; 

f = [qd_sym; D_thta\([u; 0] - n_thta)];
A = jacobian(f,[q_sym; qd_sym]);
B = jacobian(f,u);
%
matlabFunction(D_thta,'File','autogen/M_mtrx_fcn','Vars',{q_sym});
matlabFunction(n_thta,'File','autogen/n_vctr_fcn','Vars',{q_sym,qd_sym});
matlabFunction(C_thta,'File','autogen/C_mtrx_fcn','Vars',{q_sym,qd_sym});
matlabFunction(g_thta,'File','autogen/g_vctr_fcn','Vars',{q_sym});

matlabFunction(A,'File','autogen/A_mtrx_fcn','Vars',{q_sym,qd_sym, u});
matlabFunction(B,'File','autogen/B_vctr_fcn','Vars',{q_sym,qd_sym});
matlabFunction(Jv_0k(:,:,2),'File','autogen/Jacob_fcn','Vars',{q_sym});


% -----------------------------------------------------------------------%
% Transverse linearization 
% -----------------------------------------------------------------------%

% Feedforward Input
syms U real
% Transverse coordinates
syms y real
syms y_d real
syms y_2d real
% (Partial feedback linearization) feedback variable
syms v real
syms s s_d s_2d     real
% syms phi0 thta0 k   real
% phi0 = -pi/2; 
% thta0 = 0;
% k  = 0.5;

% Virtual holonomic constraints
Phi = [phi0 + k * (s - thta0), s]';
Phi_prm = diff(Phi, s);
Phi_2prm = diff(Phi, s, 2);

% Annihilator of B matrix
B = [1;0];
B_anh = null(B')';

% Change of coordinates from q to Phi
M_phi = subs(D_thta,q_sym,Phi);
C_phi = subs(C_thta,[q_sym, qd_sym],[Phi, Phi_prm.*s_d]);
G_phi = subs(g_thta,q_sym,Phi);

% Obtaining alpha, beta, gamma expressions
alfa = B_anh * M_phi * Phi_prm;
bta = B_anh * ( M_phi * Phi_2prm + diff(C_phi,s_d) * Phi_prm );
gma = B_anh * G_phi;

matlabFunction(alfa,'File','autogen/alpha_fcn','Vars',{s});
matlabFunction(bta,'File','autogen/beta_fcn','Vars',{s});
matlabFunction(gma,'File','autogen/gama_fcn','Vars',{s});

matlabFunction(Phi(1),'File','autogen/Phi_fcn','Vars',{s});
matlabFunction(Phi_prm(1),'File','autogen/Phi_prm_fcn','Vars',{s});

% Change of coordinates to transverse
h = 0;
q_new = Phi + [y; h];

%q_new_dot = Lb*[s_d;y_d]
Lb = [Phi_prm, [1;0]];
%P = Lb_dot*[s_d;y_d] so that q_new_2d = Lb*[s_2d;y_2d] + P
P = [Phi_2prm*s_d, [0;0]]*[s_d;y_d];

M_sy = subs(D_thta,q_sym,q_new);
C_sy = subs(C_thta, [q_sym,qd_sym], [q_new,Lb*[s_d; y_d]]);
G_sy = subs(g_thta,q_sym,q_new);

% y dynamics y_2d = N*u + R
N = [0 1]*simplify(inv(Lb)*inv(M_sy)*B);
R = [0 1]*simplify(inv(Lb)*(-inv(M_sy)*(C_sy*Lb*[s_d;y_d]+G_sy) - P));

u = pinv(B)*(M_sy*(P+Lb*[s_2d;y_2d])+C_sy*Lb*[s_d;y_d]+G_sy);
    
%sustitute y_2d = 0 and s_2d from ABG
%U = pinv(B)*(M_sy*(P+Lb*[simplify((-bta*s_d^2 -gma)/alfa),0]')+C_sy*Lb*[s_d;y_d]+G_sy);
U = -inv(N) * R;
U_f = inv(N)*(v-R);
matlabFunction(N,'File','autogen/N_fcn','Vars',{s,s_d});
matlabFunction(U','File','autogen/U_ff','Vars',{s,s_d,y,y_d});
matlabFunction(U_f','File','autogen/U_full','Vars',{s,s_d,y,y_d,v});

F = R + N*U;

% Getting full dynamics for s
s_dyn = B_anh*( M_sy*( Lb*[s_2d;y_2d] + P ) + C_sy*Lb*[s_d;y_d]+G_sy );
s_abg = simplify(subs(s_dyn, [y,y_d,y_2d], zeros(1,3)));

%Verify that we have the same ABG
syms tt real
alfa2 = simplify(jacobian(s_abg,s_2d));
beta2  = simplify(jacobian(subs(s_abg,s_d^2,tt),tt));
gma2 = simplify(s_abg - alfa2*s_2d - beta2*s_d^2);

% To get alpha beta gamma explicitly on the left-hand side
% we add to both parts of the equation reduced dynamics
G = simplify((-s_dyn + (alfa*s_2d + bta*s_d^2 + gma)));
G_0 = simplify(subs(G, [y,y_d,y_2d], zeros(1,3)));

G = subs(G, y_2d,v);

temp = simplify(subs(G, [y_d; v], zeros(2,1)));
g_y = simplify( jacobian(temp,y)); clear temp;

temp = simplify(subs(G, [y; v], zeros(2,1)));
g_yd = simplify( jacobian(temp,y_d)); clear temp;
        
temp = simplify(subs(G, [y; y_d], zeros(2,1)));
g_v = simplify( jacobian(temp,v)); clear temp;

temp = simplify(subs(G, [y; y_d; v], zeros(3,1)));
g_I = simplify((s_d*simplify(jacobian(temp,s_d)) - ...
                s_2d*simplify(jacobian(temp,s)))/(2*(s_d^2 + s_2d^2)));


g_y = subs(g_y, y, 0);
g_yd = subs(g_yd, y_d, 0);
g_v = subs(g_v,v, 0);
            
a = simplify(2*s_d/(alfa));
a_11 = simplify(a*(g_I - simplify(bta)));
a_12 = simplify(a*g_y);
a_13 = simplify(a*g_yd);
b1 = simplify(a*g_v);

temp = simplify(subs(F, [y; y_d; v], zeros(3,1)));
A21 = simplify( (s_d*simplify(jacobian(temp,s_d)) - ...
                s_2d*simplify(jacobian(temp,s)))/(2*(s_d^2 + s_2d^2)));
temp = simplify(subs(F, [y_d; v], zeros(2,1)));
A22 = subs( jacobian(temp,y), y, 0 );
temp = simplify(subs(F, [y; v], zeros(2,1)));
A23 = subs( jacobian(temp,y_d), y_d, 0 );
% B2  = simplify(subs(N, [y; y_d], zeros(2,1)));
B2 = 1;    
A = [a_11 a_12 a_13;0 0 1; A21 A22 A23];
B = [b1;0;B2];

A = simplify(subs(A,s_2d,(-bta*s_d^2-gma)/alfa));

matlabFunction(A,'File','autogen/A_mtrx','Vars',{s,s_d});
matlabFunction(B,'File','autogen/B_mtrx','Vars',{s,s_d});






















return
%% ------------------------------------------------------------------------
% Testing Dynamics
% ------------------------------------------------------------------------
% %{
q = pi*rand(2,1);
qd = rand(2,1);am
q2d = rand(2,1);

rbt = importrobot('planar_manip.urdf');
rbt.DataFormat = 'column';
rbt.Gravity = [0 0 -9.81];

id_matlab = inverseDynamics(rbt,q,qd,q2d)
id_full = M_mtrx_fcn(q)*q2d + n_vctr_fcn(q,qd)

%}
