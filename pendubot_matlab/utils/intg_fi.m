function f = intg_fi(s0,s_fi)
% 

g = gama_fcn(s_fi);
a = alpha_fcn(s_fi);
f = intg_psi(s_fi,s0) * 2 *( g / a);

end

