function B = B_mtrx(s,s_d)
%B_MTRX
%    B = B_MTRX(S,S_D)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    19-Feb-2020 16:41:58

t2 = cos(s);
B = [(s_d.*(t2.*6.175e-3+4.3465e-3).*2.0)./(t2.*4.94e-3-8.693e-4);0.0;1.0];