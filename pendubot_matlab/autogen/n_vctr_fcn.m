function n_thta = n_vctr_fcn(in1,in2)
%N_VCTR_FCN
%    N_THTA = N_VCTR_FCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    06-May-2020 14:35:40

q1 = in1(1,:);
q2 = in1(2,:);
qd1 = in2(1,:);
qd2 = in2(2,:);
t2 = sin(q2);
t3 = q1+q2;
t4 = cos(t3);
t5 = t4.*3.29616e-1;
n_thta = [t5+cos(q1).*1.19343555-qd2.*(qd1.*t2.*8.4e-3+qd2.*t2.*8.4e-3)-qd1.*qd2.*t2.*8.4e-3;t5+qd1.^2.*t2.*8.4e-3];
