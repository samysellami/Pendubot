function B = B_mtrx(s,s_d)
%B_MTRX
%    B = B_MTRX(S,S_D)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    19-Feb-2020 10:01:29

B = [s_d.*(cos(s).*2.05e2+2.44e2).*(-1.0./1.22e2);0.0;1.0];
