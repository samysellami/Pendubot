function N = N_fcn(s,s_d)
%N_FCN
%    N = N_FCN(S,S_D)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    02-Mar-2020 15:08:23

t2 = cos(s);
N = ((t2.*2.375e31+5.015192307692308e31).*-4.0e-21)./(t2.^2.*1.17325e9-3.511459113e9);
