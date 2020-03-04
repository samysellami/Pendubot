function A = A_mtrx_fcn(in1,in2,u)
%A_MTRX_FCN
%    A = A_MTRX_FCN(IN1,IN2,U)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    04-Mar-2020 11:11:31

q1 = in1(1,:);
q2 = in1(2,:);
qd1 = in2(1,:);
qd2 = in2(2,:);
t2 = cos(q2);
t3 = sin(q1);
t4 = q1+q2;
t5 = sin(t4);
t6 = t2.^2;
t7 = t2.*2.47e6;
t8 = t7+6.120533e6;
t9 = 1.0./t8;
t10 = t6.*1.17325e9;
t11 = t10-3.511459113e9;
t12 = 1.0./t11;
t13 = qd1.^2;
t14 = qd2.^2;
t15 = sin(q2);
t16 = t15.^2;
t17 = cos(t4);
t18 = cos(q1);
t19 = u.*4.092753336076923e48;
t20 = t17.*8.50851123293691e14;
t21 = t18.*-4.482938226386151e48;
t22 = t2.*t18.*-1.8091328678685e48;
t23 = t6.*u.*1.17325e15;
t24 = t13.*t15.*2.5272751850275e46;
t25 = t14.*t15.*2.5272751850275e46;
t26 = t6.*t17.*5.685733755e47;
t27 = t6.*t18.*-1.2851024340375e15;
t28 = t2.*u.*1.65167e48;
t29 = t2.*t17.*1.40889559014945e48;
t30 = qd1.*qd2.*t15.*5.054550370055e46;
t31 = t6.*t13.*t15.*1.44896375e46;
t32 = t6.*t14.*t15.*7.24481875e12;
t33 = t2.*t13.*t15.*4.610363896125e46;
t34 = t2.*t14.*t15.*1.019906225e46;
t35 = qd1.*qd2.*t6.*t15.*1.44896375e13;
t36 = qd1.*qd2.*t2.*t15.*2.03981245e46;
t37 = t19+t20+t21+t22+t23+t24+t25+t26+t27+t28+t29+t30+t31+t32+t33+t34+t35+t36;
t38 = qd1.*t15.*5.054550370055e46;
t39 = qd2.*t15.*5.054550370055e46;
t40 = qd2.*t2.*t15.*2.03981245e46;
t41 = qd2.*t6.*t15.*1.44896375e13;
t42 = 1.0./t11.^2;
t43 = qd2.*t15.*1.0735855e10;
t44 = qd2.*t2.*t15.*1.525225e10;
A = reshape([0.0,0.0,t9.*t12.*(t3.*4.482938226386151e48-t5.*8.50851123293691e14+t2.*t3.*1.8091328678685e48-t2.*t5.*1.40889559014945e48+t3.*t6.*1.2851024340375e15-t5.*t6.*5.685733755e47).*-2.0e-31,t12.*(t3.*9.52175193615e11-t5.*1.272410514531e12+t2.*t3.*1.35273940425e12-t2.*t5.*2.99249145e11).*1.538461538461538e-1,0.0,0.0,t9.*t12.*(t5.*8.50851123293691e14+t2.*t5.*1.40889559014945e48+t5.*t6.*5.685733755e47-t2.*t13.*2.5272751850275e46-t2.*t14.*2.5272751850275e46-t6.*t13.*4.610363896125e46-t6.*t14.*1.019906225e46+t13.*t16.*4.610363896125e46+t14.*t16.*1.019906225e46+t15.*t17.*1.40889559014945e48-t15.*t18.*1.8091328678685e48+t15.*u.*1.65167e48-qd1.*qd2.*t2.*5.054550370055e46-qd1.*qd2.*t6.*2.03981245e46+qd1.*qd2.*t16.*2.03981245e46-t2.*t6.*t13.*1.44896375e46-t2.*t6.*t14.*7.24481875e12+t2.*t13.*t16.*2.8979275e46+t2.*t14.*t16.*1.44896375e13+t2.*t15.*t17.*1.137146751e48-t2.*t15.*t18.*2.570204868075e15+t2.*t15.*u.*2.3465e15-qd1.*qd2.*t2.*t6.*1.44896375e13+qd1.*qd2.*t2.*t16.*2.8979275e13).*2.0e-31-1.0./t8.^2.*t12.*t15.*t37.*4.94e-25-t2.*t9.*t15.*t37.*t42.*4.693e-22,t12.*(t5.*1.272410514531e12+t2.*t5.*2.99249145e11-t2.*t13.*3.7794291275e10-t2.*t14.*5.3679275e9-t6.*t13.*1.525225e10-t6.*t14.*7.626125e9+t13.*t16.*1.525225e10+t14.*t16.*7.626125e9+t15.*t17.*2.99249145e11-t15.*t18.*1.35273940425e12+t15.*u.*1.235e12-qd1.*qd2.*t2.*1.0735855e10-qd1.*qd2.*t6.*1.525225e10+qd1.*qd2.*t16.*1.525225e10).*-1.538461538461538e-1+t2.*t15.*t42.*(t17.*1.272410514531e12-t18.*9.52175193615e11+u.*8.693e11+t2.*t17.*2.99249145e11-t2.*t18.*1.35273940425e12+t13.*t15.*3.7794291275e10+t14.*t15.*5.3679275e9+t2.*u.*1.235e12+qd1.*qd2.*t15.*1.0735855e10+t2.*t13.*t15.*1.525225e10+t2.*t14.*t15.*7.626125e9+qd1.*qd2.*t2.*t15.*1.525225e10).*3.61e8,1.0,0.0,t9.*t12.*(t38+t39+t40+t41+qd1.*t2.*t15.*9.22072779225e46+qd1.*t6.*t15.*2.8979275e46).*-2.0e-31,t12.*(t43+t44+qd1.*t15.*7.558858255e10+qd1.*t2.*t15.*3.05045e10).*1.538461538461538e-1,0.0,1.0,t9.*t12.*(t38+t39+t40+t41+qd1.*t2.*t15.*2.03981245e46+qd1.*t6.*t15.*1.44896375e13).*-2.0e-31,t12.*(t43+t44+qd1.*t15.*1.0735855e10+qd1.*t2.*t15.*1.525225e10).*1.538461538461538e-1],[4,4]);
