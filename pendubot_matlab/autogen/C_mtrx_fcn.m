function C_thta = C_mtrx_fcn(in1,in2)
%C_MTRX_FCN
%    C_THTA = C_MTRX_FCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    06-Mar-2020 14:55:11

q2 = in1(2,:);
qd1 = in2(1,:);
qd2 = in2(2,:);
t2 = sin(q2);
t3 = qd2.*t2.*-6.175e-3;
C_thta = reshape([t3,qd1.*t2.*6.175e-3,t3-qd1.*t2.*6.175e-3,0.0],[2,2]);
