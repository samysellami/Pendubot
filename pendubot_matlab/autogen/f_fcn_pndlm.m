function f = f_fcn_pndlm(in1,u)
%F_FCN_PNDLM
%    F = F_FCN_PNDLM(IN1,U)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    24-Apr-2020 12:38:48

thta = in1(1,:);
thta_d = in1(2,:);
f = [thta_d;u./1.0e1-sin(thta).*(4.9e1./5.0)];
