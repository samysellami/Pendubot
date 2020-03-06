function g_thta = g_vctr_fcn(in1)
%G_VCTR_FCN
%    G_THTA = G_VCTR_FCN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    06-Mar-2020 13:56:02

q1 = in1(1,:);
q2 = in1(2,:);
t2 = q1+q2;
t3 = cos(t2);
t4 = t3.*2.42307e-1;
g_thta = [t4+cos(q1).*1.09533555;t4];
