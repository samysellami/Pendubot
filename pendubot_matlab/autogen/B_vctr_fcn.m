function B = B_vctr_fcn(in1,in2)
%B_VCTR_FCN
%    B = B_VCTR_FCN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    06-Mar-2020 14:55:12

q2 = in1(2,:);
t2 = cos(q2);
t3 = t2.^2;
t4 = t3.*1.17325e9;
t5 = t4-3.511459113e9;
t6 = 1.0./t5;
B = [0.0;0.0;(t6.*(t2.*1.65167e48+t3.*1.17325e15+4.092753336076923e48).*-2.0e-31)./(t2.*2.47e6+6.120533e6);t6.*(t2.*1.235e12+8.693e11).*1.538461538461538e-1];
