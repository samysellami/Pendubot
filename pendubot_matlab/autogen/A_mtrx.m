function A = A_mtrx(s,s_d)
%A_MTRX
%    A = A_MTRX(S,S_D)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    20-Feb-2020 12:45:22

t2 = cos(s);
t3 = t2.*3.0875e-3;
t4 = t3+6.51975e-3;
t5 = 1.0./t4;
t6 = sin(s);
A = reshape([s_d.*t5.*t6.*-3.0875e-3,0.0,0.0,s_d.*t5.*cos(s.*1.5).*-4.84614e-1,0.0,0.0,(s_d.^2.*t6.*-4.94e4)./(t2.*1.235e4+2.6079e4),1.0,0.0],[3,3]);
