function A = A_mtrx_fcn(in1,in2,u)
%A_MTRX_FCN
%    A = A_MTRX_FCN(IN1,IN2,U)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    18-Feb-2020 14:24:50

q1 = in1(1,:);
q2 = in1(2,:);
qd1 = in2(1,:);
qd2 = in2(2,:);
t2 = cos(q2);
t3 = t2.^2;
t4 = t3.*4.2025e4;
t5 = t4-1.94956e5;
t6 = 1.0./t5;
t7 = qd1.^2;
t8 = sin(q2);
t9 = q1+q2;
t10 = sin(t9);
t11 = qd2.^2;
t12 = cos(t9);
t13 = qd1.*t8.*1.407937833506701e18;
t14 = qd2.*t8.*1.407937833506701e18;
t15 = sin(q1);
t16 = t15.*1.419119126341257e20;
t17 = t10.*1.172624185804515e20;
t18 = t2.*t11.*7.039689167533507e17;
t19 = cos(q1);
t20 = t8.^2;
t21 = qd1.*qd2.*t2.*1.407937833506701e18;
t22 = 1.0./t5.^2;
t23 = u.*3.433994715870003e19;
t24 = t8.*t11.*7.039689167533507e17;
t25 = t2.*t12.*3.008610238922723e19;
t26 = qd1.*qd2.*t8.*1.407937833506701e18;
t27 = qd1.*t2.*t8.*1.182898589626532e18;
t28 = qd2.*t2.*t8.*1.182898589626532e18;
A = reshape([0.0,0.0,t6.*(t16-t2.*t10.*3.008610238922723e19).*(-7.105427357601002e-14),t6.*(t16-t17-t2.*t10.*3.008610238922723e19+t2.*t15.*1.192292708606384e20).*7.105427357601002e-14,0.0,0.0,t6.*(t18+t21+t2.*t7.*7.039689167533507e17+t3.*t7.*5.914492948132659e17-t2.*t10.*3.008610238922723e19-t8.*t12.*3.008610238922723e19-t7.*t20.*5.914492948132659e17).*(-7.105427357601002e-14)-t2.*t8.*t22.*(t19.*-1.419119126341257e20+t23+t24+t25+t26+t7.*t8.*7.039689167533507e17+t2.*t7.*t8.*5.914492948132659e17).*5.972111694063642e-9,t6.*(t17-t18-t21-t2.*t7.*3.009178607269446e18-t3.*t7.*1.182898589626532e18+t2.*t10.*3.008610238922723e19-t3.*t11.*5.914492948132659e17+t8.*t12.*3.008610238922723e19+t7.*t20.*1.182898589626532e18-t8.*t19.*1.192292708606384e20+t11.*t20.*5.914492948132659e17+t8.*u.*2.885118511284224e19-qd1.*qd2.*t3.*1.182898589626532e18+qd1.*qd2.*t20.*1.182898589626532e18).*(-7.105427357601002e-14)+t2.*t8.*t22.*(t12.*1.172624185804515e20-t19.*1.419119126341257e20+t23+t24+t25+t26+t7.*t8.*3.009178607269446e18-t2.*t19.*1.192292708606384e20+t2.*u.*2.885118511284224e19+t2.*t7.*t8.*1.182898589626532e18+t2.*t8.*t11.*5.914492948132659e17+qd1.*qd2.*t2.*t8.*1.182898589626532e18).*5.972111694063642e-9,1.0,0.0,t6.*(t13+t14+t27).*(-7.105427357601002e-14),t6.*(t14+t28+qd1.*t8.*6.018357214538891e18+qd1.*t2.*t8.*2.365797179253064e18).*7.105427357601002e-14,0.0,1.0,t6.*(t13+t14).*(-7.105427357601002e-14),t6.*(t13+t14+t27+t28).*7.105427357601002e-14],[4,4]);
