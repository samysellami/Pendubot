function out1 = Jacob_fcn(in1)
%JACOB_FCN
%    OUT1 = JACOB_FCN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    02-Mar-2020 11:02:35

q1 = in1(1,:);
q2 = in1(2,:);
t2 = sin(q1);
t3 = cos(q1);
t4 = sin(q2);
t5 = cos(q2);
t6 = t3.*t5.*9.49999999993591e-2;
out1 = reshape([t2.*(-2.499999999966269e-1)-t2.*t5.*9.499999999871821e-2-t3.*t4.*9.499999999871821e-2,0.0,t3.*2.499999999983134e-1+t6-t2.*t4.*9.49999999993591e-2,t2.*t5.*(-9.499999999871821e-2)-t3.*t4.*9.499999999871821e-2,0.0,t6-t2.*t4.*9.49999999993591e-2],[3,2]);
