function A = A_mtrx(s,s_d)
%A_MTRX
%    A = A_MTRX(S,S_D)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    19-Feb-2020 16:41:58

t2 = cos(s);
t3 = t2.*4.94e-3;
t4 = t3-8.693e-4;
t5 = 1.0./t4;
t6 = sin(s);
A = reshape([s_d.*t5.*t6.*7.904e-3,0.0,0.0,s_d.*t5.*cos(s.*2.0e-1).*-4.84614e-1,0.0,0.0,s_d.^2.*t5.*t6.*-1.976e-2,1.0,0.0],[3,3]);